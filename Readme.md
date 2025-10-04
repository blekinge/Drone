# Seminarprogram: **Arduino Pilot** (ESP32 + GPS, Servo, PWM, ESP-NOW)

> **Målgruppe:** Udviklings-, test- og prototyping-ingeniører samt avancerede hobbyister.
>
> **Resultat:** Indsigt i bygning af en fungerende “pilot”-controller på ESP32, der modtager kommandoer via ESP-NOW, styrer servo (ror) og motor (PWM), læser strøm/spænding fra INA219 og GPS-data, og udsender telemetri via ESP-NOW.

---

## Læringsmål
1. Forstå arkitekturen i en simpel “pilot” på ESP32 (sensorer → styring → aktuatorer → telemetri).
2. Opsætte og bruge I²C (INA219), UART (GPS), PWM/LEDC (motor), og servo-styring (PWM) på ESP32.
3. Kende ESP-NOW til lav-latens en-til-en/mesh/broadcast kommunikation uden AP.
4. Designe en robust datapakke (kommando/telemetri) og testprocedurer.
5. Anvende bedste praksis for strømforsyning, jording, EMI og failsafe.
6. ArduPilot implementation

---

## Forudsætninger
- Basal C/C++ og Arduino-IDE/PlatformIO-kendskab.
- Kendskab til multimeter/oscilloskop anbefales.
- Installeret: **Arduino IDE (2.x)** eller **PlatformIO**, ESP32 board support, biblioteker: *Adafruit INA219*, *TinyGPSPlus*, *ESP32Servo*.

---

## Udstyrsliste (pr. deltager/gruppe)
- ESP32-WROOM-32 dev board + USB-kabel.
- INA219B modul (I²C shunt-måler) **SDA=23, SCL=22**
- GPS-modul (fx NEO-6M) på **UART2 RX=16, TX=17** @ 9600 baud.
- Mikroservo (fx SG90) på **GPIO14** (separat 5V, fælles GND).
- PWM motor/ESC/controller på **GPIO4** (LEDC PWM).
- 3× LED med seriemodstande på **GPIO 12, 13, 15**.
- Strømforsyning 5V/2A (servo/motor), breadboard, ledninger, evt. step-down.

## Schematic

- [Modtager schematic](https://github.com/gert-lauritsen/Drone/blob/main/Controller/Modtager/TocBoatReceiver.pdf)

- [Sender Schematic](https://github.com/gert-lauritsen/Drone/blob/main/Controller/Sender/RcSender.pdf)

## Datapakker (reference)
```c
// Kommandoer (ESP-NOW → pilot)
typedef struct __attribute__((packed)) {
  uint16_t servo_us;   // 1000..2000 µs typisk
  uint16_t motor_duty; // 0..1023 ved 10-bit LEDC
  uint8_t  leds;       // bit0=GPIO12, bit1=GPIO13, bit2=GPIO15
} CommandPacket;

// Telemetri (pilot → ESP-NOW)
typedef struct __attribute__((packed)) {
  float    busV;      // V
  float    shuntV;    // mV
  float    current_mA;// mA
  float    power_mW;  // mW
  double   lat;       // °
  double   lon;       // °
  float    speed_kmh; // km/h
  uint8_t  sats;      // antal satellitter
  uint32_t fixAge_ms; // ms siden sidste fix
} TelemetryPacket;
```
