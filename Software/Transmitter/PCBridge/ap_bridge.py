# ap_bridge.py -- ArduPilot (SITL) <-> ESP32 PCBridge (NO CLI ARGS)
# Requires: pip install pymavlink pyserial
# 1) Set SERIAL_PORT to your ESP32 bridge COM port (e.g., "COM12" or "/dev/ttyUSB0")
# 2) Start ArduPilot SITL (e.g., Rover): sim_vehicle.py -v Rover --map --console
# 3) Run: python ap_bridge.py

import sys, json, time, threading
from pymavlink import mavutil
import serial

# ======== CONFIG (edit me) ========
SERIAL_PORT   = "COM10"                # <-- change to your ESP32 bridge port
BAUD          = 115200
SITL_ENDPOINT = "udp:127.0.0.1:14550"
CH_SERVO      = 1   # ArduPilot CH1 -> servo_us
CH_THROTTLE   = 3   # ArduPilot CH3 -> motor duty 0..1023
SERIAL_DEBUG_PEEK_SEC = 2.0
# ==================================

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
    try:
        lat = int(float(tel.get('lat', 0.0)) * 1e7)
        lon = int(float(tel.get('lon', 0.0)) * 1e7)
        vground = float(tel.get('speed_kmh', 0.0)) / 3.6
        mav.mav.gps_input_send(
            int(time.time()*1e6),
            0, 0, 0, 0, 0,
            lat, lon, 0,
            1.0, 0,
            vground, 0.0,
            0.0, 0.0,
            0, 0, 0,
            3,
            int(tel.get('sats', 8)),
            0
        )
    except Exception as e:
        print(f"[WARN] gps_input send failed: {e}")

def send_battery(mav, tel):
    try:
        voltage = float(tel.get('busV', 0.0))
        current = float(tel.get('current_mA', 0.0)) / 1000.0  # A
        mav.mav.battery_status_send(
            0, 0,
            [int(voltage*1000)] + [-1]*9,
            int(current*100),   # cA (10*mA units)
            -1, -1, -1, -1,
            0, 0, 0
        )
    except Exception as e:
        print(f"[WARN] battery_status send failed: {e}")

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
            t = msg.get_type()
            if t == 'SERVO_OUTPUT_RAW':
                servo_us, duty = rc_from_servo_output(msg)
                ser.write(f"C {servo_us} {duty}\n".encode('ascii'))
            elif t == 'RC_CHANNELS_OVERRIDE':
                servo_us, duty = rc_from_rc_override(msg)
                ser.write(f"C {servo_us} {duty}\n".encode('ascii'))
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
                        try:
                            tel = json.loads(line.decode('utf-8'))
                            if tel.get('type') == 'telemetry':
                                send_gps_input(mav, tel)
                                send_battery(mav, tel)
                        except Exception as e:
                            print(f"[WARN] JSON parse: {e}")
            else:
                time.sleep(0.01)
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
