
/*
  ESP32 RemoteControl – MAVLink <-> ESP-NOW bridge
  Arkitektur:
    PC (ArduPilot/GCS) <--(MAVLink over USB-Seriel)--> ESP32 RemoteControl <--(ESP-NOW)--> ESP32 Rover

  Funktion:
    - Modtager MAVLink fra PC (RC_CHANNELS_OVERRIDE) og oversætter til CommandPacket, sender via ESP-NOW til Rover.
    - Modtager TelemetryPacket fra Rover (ESP-NOW), oversætter til MAVLink (HEARTBEAT/GPS_RAW_INT/BATTERY_STATUS) og sender til PC.
    - Valgfrit failover: hvis ingen MAVLink RC override i T_OVR_MS → behold sidste kommando eller send neutral.

  Krav:
    - ESP32 Arduino core
    - mavlink/c_library_v2 i include-sti (common dialekt)
*/

#include <WiFi.h>
#include <esp_now.h>

// ---------- MAVLink ----------
extern "C" {
  #include "mavlink/common/mavlink.h"
}

// ---------- ESP-NOW pakker (skal matche Rover) ----------
typedef struct __attribute__((packed)) {
  uint16_t servo_us;   // 1000..2000
  uint16_t motor_duty; // 0..1023
  uint8_t  leds;       // bitmask bit0=GPIO12, bit1=GPIO13, bit2=GPIO15
} CommandPacket;

typedef struct __attribute__((packed)) {
  float    busV;
  float    shuntV;
  float    current_mA;
  float    power_mW;
  double   lat;
  double   lon;
  float    speed_kmh;
  uint8_t  sats;
  uint32_t fixAge_ms;
} TelemetryPacket;

uint8_t broadcastAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ---------- Konstanter ----------
static const uint8_t  SYSID  = 42;   // Vælg et ID til denne companion
static const uint8_t  COMPID = 201;  // Companion component-id
static const uint32_t BAUD   = 57600;

// Tidskonstanter
static const uint32_t T_HB_MS   = 1000; // heartbeat
static const uint32_t T_GPS_MS  = 200;  // 5 Hz
static const uint32_t T_BATT_MS = 1000; // 1 Hz
static const uint32_t T_OVR_MS  = 1200; // override timeout

// ---------- State ----------
volatile bool haveTelemetry = false;
TelemetryPacket lastTel = {};

CommandPacket outCmd = {1500, 0, 0};
uint32_t lastOverrideMs = 0;

uint32_t lastHbMs = 0, lastGpsMs = 0, lastBattMs = 0;
uint32_t lastCmdSendMs = 0;

// ---------- ESP-NOW callbacks ----------
void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  // optional debug
}

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len == sizeof(TelemetryPacket)) {
    memcpy((void*)&lastTel, incomingData, sizeof(TelemetryPacket));
    haveTelemetry = true;
  }
}

// ---------- MAVLink helpers (send) ----------
void mav_send_heartbeat() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_heartbeat_pack(
    SYSID, COMPID, &msg,
    MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID,
    0, 0, MAV_STATE_ACTIVE
  );
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void mav_send_gps(const TelemetryPacket& t) {
  mavlink_message_t msg; uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint8_t fix_type = (t.sats >= 3 && t.fixAge_ms < 3000) ? 3 : (t.sats > 0 ? 2 : 0);
  int32_t latE7 = (int32_t)(t.lat * 1e7);
  int32_t lonE7 = (int32_t)(t.lon * 1e7);
  int32_t alt_mm = 0;
  uint16_t eph = UINT16_MAX, epv = UINT16_MAX;
  uint16_t vel = (uint16_t)((t.speed_kmh * (1000.0f/3600.0f)) * 100.0f); // m/s*100
  uint16_t cog = UINT16_MAX;
  mavlink_msg_gps_raw_int_pack(SYSID, COMPID, &msg,
    millis(), fix_type, latE7, lonE7, alt_mm, eph, epv, vel, cog, t.sats);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void mav_send_batt(const TelemetryPacket& t) {
  mavlink_message_t msg; uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t voltages[10] = {0};
  voltages[0] = (uint16_t)(t.busV * 1000.0f);      // mV
  int16_t current_cA = (int16_t)(t.current_mA / 10.0f); // 10 mA enheder
  mavlink_msg_battery_status_pack(SYSID, COMPID, &msg,
    0, MAV_BATTERY_FUNCTION_ALL, MAV_BATTERY_TYPE_LIPO,
    INT16_MAX, voltages, current_cA, -1, -1, -1, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

// ---------- MAVLink input (PC -> RC) ----------
void handle_mavlink_message(const mavlink_message_t& msg) {
  if (msg.msgid == MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE) {
    mavlink_rc_channels_override_t rc;
    mavlink_msg_rc_channels_override_decode(&msg, &rc);

    // Map CH1 → servo (1000..2000 µs), CH3 → motor (0..1023), CH7-9 → LED
    uint16_t ch1 = rc.chan1_raw; // 1000..2000
    uint16_t ch3 = rc.chan3_raw;
    uint16_t ch7 = rc.chan7_raw, ch8 = rc.chan8_raw, ch9 = rc.chan9_raw;

    if (ch1 >= 900 && ch1 <= 2100) {
      outCmd.servo_us = ch1;
    }
    if (ch3 >= 900 && ch3 <= 2100) {
      long d = map(ch3, 1000, 2000, 0, 1023);
      if (d < 0) d = 0; if (d > 1023) d = 1023;
      outCmd.motor_duty = (uint16_t)d;
    }
    uint8_t m = 0;
    if (ch7 > 1500) m |= 0x01;
    if (ch8 > 1500) m |= 0x02;
    if (ch9 > 1500) m |= 0x04;
    outCmd.leds = m;

    lastOverrideMs = millis();
  }
}

// ---------- Setup ----------
void setup() {
  Serial.begin(BAUD); // MAVLink til/fra PC
  delay(200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    while (1) { delay(1000); }
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

// ---------- Loop ----------
void loop() {
  // MAVLink parse fra PC
  while (Serial.available()) {
    uint8_t c = (uint8_t)Serial.read();
    static mavlink_message_t msg;
    static mavlink_status_t status;
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      handle_mavlink_message(msg);
    }
  }

  uint32_t now = millis();

  // Send CommandPacket til Rover (50 ms cadence)
  if (now - lastCmdSendMs >= 50) {
    lastCmdSendMs = now;
    // hvis override timeout → hold sidste (eller implementer neutral her)
    esp_now_send(broadcastAddress, (uint8_t*)&outCmd, sizeof(outCmd));
  }

  // Forward telemetri til PC som MAVLink
  if (haveTelemetry) {
    haveTelemetry = false;
    // rytme: HB 1 Hz, GPS 5 Hz, BATT 1 Hz
    if (now - lastHbMs >= T_HB_MS) { lastHbMs = now; mav_send_heartbeat(); }
    if (now - lastGpsMs >= T_GPS_MS) { lastGpsMs = now; mav_send_gps(lastTel); }
    if (now - lastBattMs >= T_BATT_MS) { lastBattMs = now; mav_send_batt(lastTel); }
  }
}
