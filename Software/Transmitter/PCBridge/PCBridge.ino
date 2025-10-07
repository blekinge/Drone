// PCBridge.ino  (ESP32 by the PC): Serial <-> ESP-NOW bridge
// - Reads commands from Serial as:  C <servo_us> <motor_duty10>\n   (e.g. "C 1500 600")
// - Sends CommandPacket via ESP-NOW to the boat
// - Receives TelemetryPacket via ESP-NOW from the boat and prints NDJSON on Serial
//
// Board: ESP32 Wrover Module (or any ESP32)
// Serial: 115200 8N1

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// ===================== Shared packets (keep in sync with boat) =====================
typedef struct __attribute__((packed)) {
  uint16_t servo_us;     // 1000..2000
  uint16_t motor_duty;   // 0..1023
  uint8_t  leds;         // optional bitmask
} CommandPacket;

typedef struct __attribute__((packed)) {
  float    busV;        // battery voltage
  float    shuntV;      // shunt voltage
  float    current_mA;  // current
  float    power_mW;    // power
  double   lat;
  double   lon;
  float    speed_kmh;
  uint8_t  sats;
  uint32_t fixAge_ms;
} TelemetryPacket;

// ===================== Config =====================
// If you know the boat's MAC, put it here; otherwise we'll broadcast.
uint8_t boat_mac[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ===================== State =====================
volatile bool haveTelemetry = false;
TelemetryPacket lastTel{};

void onDataSent(const uint8_t*, esp_now_send_status_t){}

void onDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  if (len == sizeof(TelemetryPacket)) {
    memcpy((void*)&lastTel, data, sizeof(TelemetryPacket));
    haveTelemetry = true;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); // ensure clean state

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERR esp_now_init");
    while (true) delay(1000);
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  // Add peer (broadcast ok)
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, boat_mac, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  Serial.println("{\"type\":\"pcbridge\",\"status\":\"ready\"}");
}

static bool readCommandFromSerial(CommandPacket &cmd) {
  // Expect: C <servo_us> <motor_duty10>\n   e.g., "C 1500 600\n"
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line.trim();
      if (line.length() > 0 && line.charAt(0) == 'C') {
        int s1, s2;
        if (sscanf(line.c_str(), "C %d %d", &s1, &s2) == 2) {
          cmd.servo_us   = (uint16_t) constrain(s1, 500, 2500);
          cmd.motor_duty = (uint16_t) constrain(s2, 0, 1023);
          cmd.leds = 0;
          line = "";
          return true;
        }
      }
      line = "";
    } else {
      line += c;
      if (line.length() > 64) line = ""; // reset on garbage
    }
  }
  return false;
}

unsigned long lastTelPrint = 0;
const unsigned TEL_PERIOD_MS = 100;

void loop() {
  // Downlink: read commands from PC -> send to boat
  CommandPacket cmd{};
  if (readCommandFromSerial(cmd)) {
    esp_now_send(boat_mac, (uint8_t*)&cmd, sizeof(cmd));
  }

  // Uplink: print telemetry JSON at ~10 Hz
  if (haveTelemetry && (millis() - lastTelPrint) >= TEL_PERIOD_MS) {
    noInterrupts();
    TelemetryPacket t = lastTel;
    haveTelemetry = false;
    interrupts();
    lastTelPrint = millis();
    Serial.printf(
      "{\"type\":\"telemetry\",\"t_ms\":%lu,"
      "\"busV\":%.3f,\"shuntV\":%.3f,\"current_mA\":%.1f,\"power_mW\":%.1f,"
      "\"lat\":%.7f,\"lon\":%.7f,\"speed_kmh\":%.2f,"
      "\"sats\":%u,\"fixAge_ms\":%u}\n",
      (unsigned long)millis(),
      t.busV, t.shuntV, t.current_mA, t.power_mW,
      t.lat, t.lon, t.speed_kmh,
      (unsigned)t.sats, (unsigned)t.fixAge_ms
    );
  }
}
