#!/usr/bin/env python3
# Enkel PC-konsol til ESP32 sender/bridge
# - Viser NDJSON-telemetri
# - Sender kommandoer (mode/servo/motor/leds)
import sys, argparse, threading, json, time
import serial

def reader_thread(ser):
    buf = b""
    while True:
        try:
            chunk = ser.readline()
            if not chunk:
                time.sleep(0.01)
                continue
            line = chunk.decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            # prøv JSON først
            if line.startswith("{") and line.endswith("}"):
                try:
                    obj = json.loads(line)
                    if obj.get("type") == "telemetry":
                        print(f"[TEL] V={obj.get('busV')}V  I={obj.get('current_mA')}mA  "
                              f"P={obj.get('power_mW')}mW  lat={obj.get('lat')} lon={obj.get('lon')} "
                              f"v={obj.get('speed_kmh')}km/t sats={obj.get('sats')} age={obj.get('fixAge_ms')}ms")
                    else:
                        print("[ESP]", obj)
                except Exception:
                    print("[RAW]", line)
            else:
                print("[RAW]", line)
        except Exception as e:
            print("Reader error:", e)
            time.sleep(0.5)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="Seriel port (fx COM5 eller /dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    t = threading.Thread(target=reader_thread, args=(ser,), daemon=True)
    t.start()

    print("Kommandoer:")
    print("  mode=manual | mode=pc")
    print("  servo=1500  (µs)")
    print("  motor=512   (0..1023)")
    print("  leds=3      (bitmask 0..7)")
    print("Ctrl+C for at afslutte.")

    try:
        while True:
            line = input("> ").strip()
            if not line:
                continue
            ser.write((line+"\n").encode("utf-8"))
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

if __name__ == "__main__":
    main()
