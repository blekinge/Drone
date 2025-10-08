# ap_bridge.py -- ArduPilot (SITL) <-> ESP32 PCBridge (NO CLI ARGS)
# Requires: pip install pymavlink pyserial
# 1) Set SERIAL_PORT to your ESP32 bridge COM port (e.g., "COM12" or "/dev/ttyUSB0")
# 2) Start ArduPilot SITL (e.g., Rover): sim_vehicle.py -v Rover --map --console
# 3) Run: python ap_bridge.py

import sys, json, time, threading
from pymavlink import mavutil
import serial

# ======== CONFIG (edit me) ========
SERIAL_PORT   = "COM6"                # <-- change to your ESP32 bridge port
BAUD          = 115200
SITL_ENDPOINT = "udpin:0.0.0.0:14550"   # if SITL is in WSL sending to Windows host, you can also use "udpin:0.0.0.0:14550"
CH_SERVO      = 1   # ArduPilot CH1 -> servo_us
CH_THROTTLE   = 3   # ArduPilot CH3 -> motor duty 0..1023
SERIAL_DEBUG_PEEK_SEC = 2.0
DEBUG_EVERY_SEC = 2.0
# ==================================


def _to_float(v, default=0.0):
    # robust number coercion: handles 25.08, "25.08", "25,08"
    try:
        if isinstance(v, (int, float)):
            return float(v)
        s = str(v).strip().replace(",", ".")
        return float(s)
    except Exception:
        return float(default)

def _to_int(v, default=0):
    try:
        if isinstance(v, bool):
            return int(v)
        if isinstance(v, (int,)):
            return int(v)
        # allow floats/strings like "12", "12.0", "12,0"
        return int(round(_to_float(v, default)))
    except Exception:
        return int(default)

# Simple counters + timers for debug
_stat = {"rx_mav":0, "tx_cmd":0, "tx_gps":0, "tx_batt":0}
_last = {"print":0.0, "hb":0.0}
_last["gps_tick"] = 0.0
GPS_PERIOD_SEC = 1.0   # send GPS_INPUT at ~1 Hz

def connect_sitl():
    print(f"[INFO] Connecting to SITL at {SITL_ENDPOINT} ...")
    mav = mavutil.mavlink_connection(SITL_ENDPOINT, autoreconnect=True, dialect='ardupilotmega')
    # Non-blocking heartbeat wait
    def wait_hb():
        try:
            mav.wait_heartbeat(timeout=10)
            print("[OK] SITL heartbeat received.")
        except Exception:
            print("[WARN] No SITL heartbeat yet; continuing (will keep trying in background).")
    threading.Thread(target=wait_hb, daemon=True).start()
    return mav

def open_serial():
    print(f"[INFO] Opening serial {SERIAL_PORT} @ {BAUD} ...")
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.02)
    print("[OK] Serial opened.")
    return ser

def send_gps_input(mav, tel):
    """
    Robust GPS_INPUT sender for mixed pymavlink builds.
    - Casts everything explicitly.
    - Skips send if no fix (sats<=0 or lat/lon==0.0).
    - Uses keyword args; falls back if 'yaw_accuracy' unsupported.
    """
    try:
        now_us = int(time.time() * 1e6)

        lat_f = _to_float(tel.get('lat', 0.0))
        lon_f = _to_float(tel.get('lon', 0.0))
        sats  = _to_int(tel.get('sats', 0))

        # No GPS yet? skip (avoids packer errors and noise)
        if sats <= 0 or (lat_f == 0.0 and lon_f == 0.0):
            return

        lat = int(lat_f * 1e7)
        lon = int(lon_f * 1e7)
        alt = float(0.0)

        # Ignore fields we don’t provide
        IGNORE = (1 | 2 | 4 | 8 | 16 | 32)  # vel/acc/yaw ignored

        fix_type = 3 if sats >= 6 else (2 if sats >= 4 else 1)
        hdop = float(1.0)
        vdop = float(2.0)

        # Try newer signature first (has yaw_accuracy), then fallback
        try:
            mav.mav.gps_input_send(
                time_usec=now_us,
                gps_id=0,
                ignore_flags=IGNORE,
                time_week_ms=0,
                time_week=0,
                fix_type=fix_type,
                lat=lat, lon=lon, alt=alt,
                hdop=hdop, vdop=vdop,
                vn=0.0, ve=0.0, vd=0.0,
                speed_accuracy=0.0,
                horiz_accuracy=0.0,
                vert_accuracy=0.0,
                satellites_visible=sats,
                yaw=0.0,
                yaw_accuracy=0.0,
            )
        except TypeError:
            mav.mav.gps_input_send(
                time_usec=now_us,
                gps_id=0,
                ignore_flags=IGNORE,
                time_week_ms=0,
                time_week=0,
                fix_type=fix_type,
                lat=lat, lon=lon, alt=alt,
                hdop=hdop, vdop=vdop,
                vn=0.0, ve=0.0, vd=0.0,
                speed_accuracy=0.0,
                horiz_accuracy=0.0,
                vert_accuracy=0.0,
                satellites_visible=sats,
                yaw=0.0,
            )

        _stat["tx_gps"] += 1

    except Exception as e:
        print(f"[WARN] gps_input send failed: {e}")
        print(f"        lat={lat_f} lon={lon_f} sats={sats}")



def send_battery(mav, tel):
    # BATTERY_STATUS: 10 args (some builds 10–11). Use 10 here.
    # id, battery_function, type, temperature, voltages[10], current_battery(cA), current_consumed, energy_consumed, battery_remaining, time_remaining
    try:
        voltage = float(tel.get('busV', 0.0) or 0.0)
        current_A = float(tel.get('current_mA', 0.0) or 0.0) / 1000.0

        v_mv = int(round(voltage * 1000.0))
        if v_mv < 0: v_mv = 0
        if v_mv > 65535: v_mv = 65535
        # Use 65535 for unknown cells per MAVLink convention
        voltages = [v_mv] + [65535]*9

        current_cA = int(round(current_A * 100.0))
        # int16 range clamp for safety
        if current_cA < -32768: current_cA = -32768
        if current_cA >  32767: current_cA =  32767

        mav.mav.battery_status_send(
            int(0),          # id
            int(0),          # battery_function
            int(0),          # type
            int(0),          # temperature (cdegC) unknown
            voltages,        # 10x mV
            int(current_cA), # current_battery (10mA units)
            int(-1),         # current_consumed (mAh) unknown
            int(-1),         # energy_consumed (hJ) unknown
            int(-1),         # battery_remaining (%) unknown
            int(-1)          # time_remaining (s) unknown
        )
        _stat["tx_batt"] += 1
    except Exception as e:
        print(f"[WARN] battery_status send failed: {e}")
        print(f"        V={voltage:.3f}V  I={current_A:.3f}A  voltages[0]={v_mv}mV  current_cA={current_cA}")


def rc_from_servo_output(msg):
    servo_us    = getattr(msg, f"servo{CH_SERVO}_raw", 1500)
    throttle_us = getattr(msg, f"servo{CH_THROTTLE}_raw", 1000)
    duty = int(max(0, min(1023, (throttle_us - 1000) * 1023.0 / 1000.0)))
    return int(servo_us), duty

def rc_from_rc_override(msg):
    def f(v, default): return v if v and v != 65535 else default
    servo_us    = f(msg.chan1_raw, 1500)
    throttle_us = f(msg.chan3_raw, 1000)
    duty = int(max(0, min(1023, (throttle_us - 1000) * 1023.0 / 1000.0)))
    return int(servo_us), duty

def sitl_reader(mav, ser: serial.Serial):
    while True:
        try:
            msg = mav.recv_match(blocking=False)
            if not msg:
                time.sleep(0.01)
                continue
            _stat["rx_mav"] += 1
            t = msg.get_type()

            # Light heartbeat + output debug
            if t == 'HEARTBEAT' and (time.time() - _last["hb"]) > 2.0:
                print("[MAV] HEARTBEAT")
                _last["hb"] = time.time()

            if t == 'SERVO_OUTPUT_RAW':
                s = getattr(msg, "servo1_raw", 1500)
                th = getattr(msg, "servo3_raw", 1000)
                print(f"[MAV] SERVO_OUTPUT_RAW ch1={s} ch3={th}")
                servo_us, duty = rc_from_servo_output(msg)
                ser.write(f"C {servo_us} {duty}\n".encode('ascii'))
                _stat["tx_cmd"] += 1
            elif t == 'RC_CHANNELS_OVERRIDE':
                servo_us, duty = rc_from_rc_override(msg)
                ser.write(f"C {servo_us} {duty}\n".encode('ascii'))
                _stat["tx_cmd"] += 1

        except Exception as e:
            print(f"[WARN] sitl_reader: {e}")
            time.sleep(0.2)

def serial_reader(mav, ser: serial.Serial):
    buf = b''
    last_print = 0.0
    while True:
        try:
            chunk = ser.read(512)
            if chunk:
                buf += chunk
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    line = line.strip()
                    if not line:
                        continue
                    # Debug peek every few seconds so you can see it's alive
                    if (time.time() - last_print) > SERIAL_DEBUG_PEEK_SEC:
                        try:
                            print(f"[SERIAL] {line[:120].decode(errors='ignore')}")
                        except:
                            pass
                        last_print = time.time()
                    if line.startswith(b'{'):
                        # First parse JSON safely
                        try:
                            tel = json.loads(line.decode('utf-8'))
                        except Exception as e:
                            print(f"[WARN] JSON parse failed: {e}")
                            continue
                    
                        # Then send MAVLink, each with its own try/except inside the functions
                        if tel.get('type') == 'telemetry':
                            send_gps_input(mav, tel)
                            send_battery(mav, tel)

            else:
                time.sleep(0.01)

            # Stats ticker
            now = time.time()
            if now - _last["print"] > DEBUG_EVERY_SEC:
                print(f"[STATS] rx_mav={_stat['rx_mav']}  tx_cmd={_stat['tx_cmd']}  tx_gps={_stat['tx_gps']}  tx_batt={_stat['tx_batt']}")
                _last["print"] = now

        except Exception as e:
            print(f"[WARN] serial_reader: {e}")
            time.sleep(0.5)

def main():
    try:
        ser = open_serial()
    except Exception as e:
        print(f"[ERROR] Opening serial failed: {e}")
        sys.exit(1)

    try:
        mav = connect_sitl()
    except Exception as e:
        print(f"[WARN] SITL connect failed now, continuing: {e}")
        mav = mavutil.mavlink_connection(SITL_ENDPOINT, autoreconnect=True)

    threading.Thread(target=serial_reader, args=(mav, ser), daemon=True).start()
    threading.Thread(target=sitl_reader,   args=(mav, ser), daemon=True).start()

    print("[OK] Bridge running. Reading serial and MAVLink.")
    while True:
        time.sleep(1)

if __name__ == '__main__':
    main()
