#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <ESP32Servo.h>

// ===================== Pin configuration =====================
#define I2C_SDA       23     // IMPORTANT: must be output-capable pins
#define I2C_SCL       22
#define GPS_RX        16     // UART2 RX
#define GPS_TX        17     // UART2 TX
#define SERVO_PIN     14
#define MOTOR_PWM_PIN 4
#define LED1_PIN      12     // bit0
#define LED2_PIN      13     // bit1
#define LED3_PIN      15     // bit2

#define MaxPuls       2500
#define MinPuls       500
#define DEBUG_TELEMETRY 1
// ===================== Motor PWM (LEDC) ======================
const int motorPwmChannel = 0;
const int MOTOR_PWM_FREQ_HZ = 20000; // 20 kHz (quiet for many motor drivers)
const int MOTOR_PWM_RES_BITS = 10;   // 0..1023 duty

// ===================== GPS ======================
HardwareSerial GPS_Serial(2);  // UART2
TinyGPSPlus gps;

// ===================== INA219 ===================
Adafruit_INA219 ina219;

// ===================== Servo ===================
Servo servoMotor;
uint16_t current_servo_us = 1500;

// ===================== Timing ==================
unsigned long lastTelemetryMs = 0;
const unsigned long TELEMETRY_PERIOD_MS = 2000;

// ===================== ESP-NOW payloads =====================
// Keep packets small (<250 bytes). Pack structs to avoid padding.
typedef struct __attribute__((packed)) {
  uint16_t servo_us;   // microseconds: 1000..2000
  uint16_t motor_duty; // 0..1023
  uint8_t  leds;       // bit0->GPIO12, bit1->GPIO13, bit2->GPIO15
} CommandPacket;

typedef struct __attribute__((packed)) {
  float    busV;         // INA219 bus voltage (V)
  float    shuntV;       // shunt voltage (mV)
  float    current_mA;   // current (mA)
  float    power_mW;     // power (mW)
  double   lat;          // latitude
  double   lon;          // longitude
  float    speed_kmh;    // GPS speed (km/h)
  uint8_t  sats;         // satellites in view (from GPS)
  uint32_t fixAge_ms;    // ms since last fix
} TelemetryPacket;

// Broadcast MAC (all FFs) to reach any peer listening for us.
uint8_t broadcastAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ===================== State =====================
volatile bool haveCommand = false;
CommandPacket lastCmd = {1500, 0, 0};

// ===================== ESP-NOW Callbacks =====================
void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  // Optional: debug status
  // Serial.printf("ESP-NOW send status: %s\n", status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void onDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  const uint8_t* mac = info->src_addr;  // if you need the sender’s MAC
  if (len == sizeof(CommandPacket)) {
    memcpy((void*)&lastCmd, incomingData, sizeof(CommandPacket));
    haveCommand = true;
  }
}

// ===================== Helpers =====================
void applyCommand(const CommandPacket& cmd) {
  // Servo
  uint16_t us = constrain(cmd.servo_us, 500, 2500); // wider bound, clamp safely
  servoMotor.writeMicroseconds(us);
  current_servo_us = us;

  // Motor PWM
  uint16_t duty = constrain(cmd.motor_duty, 0, (1<<MOTOR_PWM_RES_BITS)-1);
  ledcWrite(motorPwmChannel, duty);

  // LEDs bitmask: bit0->12, bit1->13, bit2->15
  digitalWrite(LED1_PIN, (cmd.leds & 0x01) ? HIGH : LOW);
  digitalWrite(LED2_PIN, (cmd.leds & 0x02) ? HIGH : LOW);
  digitalWrite(LED3_PIN, (cmd.leds & 0x04) ? HIGH : LOW);
}

static void debugPrintTelemetry(const TelemetryPacket& t) {
#if DEBUG_TELEMETRY
  // Human-readable, compact
  Serial.printf(
    "[TEL %lu] V=%.3fV  I=%.1fmA  P=%.1fmW  shunt=%.3fmV  "
    "lat=%.7f  lon=%.7f  v=%.2fkm/h  sats=%u  age=%ums\n",
    (unsigned long)millis(),
    t.busV, t.current_mA, t.power_mW, t.shuntV,
    t.lat, t.lon, t.speed_kmh, t.sats, (unsigned)t.fixAge_ms
  );

  // Machine-friendly JSON (NDJSON: one JSON object per line)
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
#endif
}

void sendTelemetry() {
  TelemetryPacket t = {};
  // INA219 readings
  t.busV      = ina219.getBusVoltage_V();
  t.shuntV    = ina219.getShuntVoltage_mV();
  t.current_mA= ina219.getCurrent_mA();
  t.power_mW  = ina219.getPower_mW();

  // GPS readings (only if we have a recent fix)
  t.lat       = gps.location.isValid() ? gps.location.lat() : 0.0;
  t.lon       = gps.location.isValid() ? gps.location.lng() : 0.0;
  t.speed_kmh = gps.speed.isValid()    ? gps.speed.kmph()   : 0.0;
  t.sats      = gps.satellites.isValid()? gps.satellites.value() : 0;
  t.fixAge_ms = gps.location.age();
  debugPrintTelemetry(t);
  esp_err_t res = esp_now_send(broadcastAddress, (uint8_t*)&t, sizeof(t));
  // Optional debug:
  // if (res != ESP_OK) Serial.printf("ESP-NOW telemetry send err: %d\n", res);
}

void feedGPS() {
  while (GPS_Serial.available()) {
    gps.encode(GPS_Serial.read());
  }
}

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);
  delay(200);

  // GPIO setup early (keep GPIO12 low at boot)
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(LED3_PIN, LOW);

  // I2C for INA219
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!ina219.begin()) {
    Serial.println("INA219 not found. Check wiring & I2C pins.");
  } else {
    // Optional: set calibration (default works for many shunts)
    // ina219.setCalibration_32V_2A();
  }

  // GPS UART2
  GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // Servo  
  ESP32PWM::allocateTimer(0); // Make sure timers are available for the servo (optional but safe)
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servoMotor.setPeriodHertz(50);  
  servoMotor.attach(SERVO_PIN,MinPuls,MaxPuls);
  servoMotor.writeMicroseconds(current_servo_us);

  // Motor PWM via LEDC
  int motorPwmChannel = ledcAttach(MOTOR_PWM_PIN, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);
  if (motorPwmChannel < 0) {
    Serial.println("Motor attach failed");
  }

  ledcWrite(motorPwmChannel, 0);

  // WiFi + ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); // ensure clean state

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    while (true) { delay(1000); }
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  // Add broadcast peer so we can send without knowing a specific MAC
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;     // current WiFi channel
  peerInfo.encrypt = false; // no encryption for broadcast
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add broadcast peer");
  }

  Serial.println("Setup complete.");
}

// ===================== Loop =====================
void loop() {
  // Keep GPS parser fed
  feedGPS();

  // Apply new commands if any
  if (haveCommand) {
    noInterrupts();
    CommandPacket cmd = lastCmd;
    haveCommand = false;
    interrupts();
    applyCommand(cmd);
  }

  // Send telemetry periodically
  unsigned long now = millis();
  if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = now;
    sendTelemetry();
  }
}
