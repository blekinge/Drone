/*
  ESP32 Sender/Bridge til Arduino Pilot
  - Manuelle input: to potmetre (servo, motor) + 3 knapper (LED-bitmasker)
  - PC-kommandoer via USB-seriel (mode=..., servo=..., motor=..., leds=...)
  - ESP-NOW: sender CommandPacket til pilot, modtager TelemetryPacket og forwarder til PC
 
  Recommended pins for potentiometer (ADC1)
  - GPIO32 (ADC1_CH4)
  - GPIO33 (ADC1_CH5)
  - GPIO34 (ADC1_CH6, input-only)
  - GPIO35 (ADC1_CH7, input-only)
  - GPIO36 / VP (ADC1_CH0, input-only)
  - GPIO39 / VN (ADC1_CH3, input-only)

  Avoid (or only use if Wi-Fi is OFF)

  ADC2 pins: GPIO0, 2, 4, 12, 13, 14, 15, 25, 26, 27
  ADC2 conflicts with the Wi-Fi driver, so readings can be unstable/zero during ESP-NOW.
*/

#include <WiFi.h>
#include <esp_now.h>

//Cross-version compatible shim (works on 2.x and 3.x)
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  #define ESPNOW_RECV_CB_ARGS  const esp_now_recv_info* info, const uint8_t* incomingData, int len
  #define ESPNOW_SENDER_MAC    (info->src_addr)
#else
  #define ESPNOW_RECV_CB_ARGS  const uint8_t* mac, const uint8_t* incomingData, int len
  #define ESPNOW_SENDER_MAC    (mac)
#endif

// ---------- Pins ----------
#define POT_SERVO_PIN 32   // ADC (input-only OK)
#define POT_MOTOR_PIN 33   // ADC
#define Joystick1_sel 14

#define POT_1 36   // ADC
#define POT_2 39   // ADC
#define Joystick1_V 34
#define Joystick1_H 35
#define Joystick1_Sel 19


#define BTN_LED1_PIN  2   // knap til LED bit0 (aktiv lav)
#define BTN_LED2_PIN  12   // knap til LED bit1
#define BTN_LED3_PIN  13   // knap til LED bit2
#define BTN_LED3_PIN  15   // knap til LED bit2

// ---------- LEDC resolution for motor on pilot (reference) ----------
const int MOTOR_PWM_RES_BITS = 10; // 0..1023

// ---------- ESP-NOW broadcast ----------
uint8_t broadcastAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ---------- Packets (skal matche pilot) ----------
typedef struct __attribute__((packed)) {
  uint16_t servo_us;   // 1000..2000
  uint16_t motor_duty; // 0..1023
  uint8_t  leds;       // bit0..bit2
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

// ---------- State ----------
enum ControlMode { MODE_MANUAL, MODE_PC };
ControlMode mode = MODE_MANUAL;

CommandPacket cmd = {1500, 0, 0};
unsigned long lastCmdSendMs = 0;
unsigned long lastTelemetryForwardMs = 0;
unsigned long lastPcCmdMs = 0;

TelemetryPacket lastTelemetry = {};
volatile bool haveTelemetry = false;

// ---------- ESP-NOW Callbacks ----------
void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  // Optional debug
}

void onDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  const uint8_t* mac = info->src_addr;   // if you need the sender MAC
  if (len == sizeof(TelemetryPacket)) {
    memcpy((void*)&lastTelemetry, incomingData, sizeof(TelemetryPacket));
    haveTelemetry = true;
  }
}

// ---------- Helpers ----------
uint16_t mapServoFromADC(int adc) {
  // adc: 0..4095 → 1000..2000 µs
  long us = map(adc, 0, 4095, 1000, 2000);
  if (us < 500) us = 500;
  if (us > 2500) us = 2500;
  return (uint16_t)us;
}

uint16_t mapMotorFromADC(int adc) {
  // adc: 0..4095 → 0..1023
  long duty = map(adc, 0, 4095, 0, (1<<MOTOR_PWM_RES_BITS)-1);
  if (duty < 0) duty = 0;
  if (duty > (1<<MOTOR_PWM_RES_BITS)-1) duty = (1<<MOTOR_PWM_RES_BITS)-1;
  return (uint16_t)duty;
}

uint8_t readLedMask() {
  uint8_t m = 0;
  if (digitalRead(BTN_LED1_PIN) == LOW) m |= 0x01;
  if (digitalRead(BTN_LED2_PIN) == LOW) m |= 0x02;
  if (digitalRead(BTN_LED3_PIN) == LOW) m |= 0x04;
  return m;
}

void sendCommand() {
  esp_now_send(broadcastAddress, (uint8_t*)&cmd, sizeof(cmd));
}

void forwardTelemetryIfDue() {
  unsigned long now = millis();
  if (now - lastTelemetryForwardMs >= 200) {
    lastTelemetryForwardMs = now;
    if (haveTelemetry) {
      haveTelemetry = false;
      // NDJSON til PC
      Serial.print("{\"type\":\"telemetry\",");
      Serial.print("\"busV\":"); Serial.print(lastTelemetry.busV, 3); Serial.print(",");
      Serial.print("\"current_mA\":"); Serial.print(lastTelemetry.current_mA, 1); Serial.print(",");
      Serial.print("\"power_mW\":"); Serial.print(lastTelemetry.power_mW, 1); Serial.print(",");
      Serial.print("\"lat\":"); Serial.print(lastTelemetry.lat, 7); Serial.print(",");
      Serial.print("\"lon\":"); Serial.print(lastTelemetry.lon, 7); Serial.print(",");
      Serial.print("\"speed_kmh\":"); Serial.print(lastTelemetry.speed_kmh, 2); Serial.print(",");
      Serial.print("\"sats\":"); Serial.print(lastTelemetry.sats); Serial.print(",");
      Serial.print("\"fixAge_ms\":"); Serial.print(lastTelemetry.fixAge_ms);
      Serial.print("\"Servo_us\":"); Serial.print(cmd.servo_us);
      Serial.print("\"motor_duty\":"); Serial.print(cmd.motor_duty);
      Serial.println("}");
    }
  }
}

void applyManualInputs() {
  int adcServo = analogRead(POT_SERVO_PIN);
  int adcMotor = analogRead(POT_MOTOR_PIN);
  //Serial.print("Read input Servo and motor "); Serial.print(adcServo); Serial.print(" "); Serial.println(adcMotor);
  cmd.servo_us  = mapServoFromADC(adcServo);
  cmd.motor_duty= mapMotorFromADC(adcMotor);
  cmd.leds      = readLedMask();
}

void printAck(const char* what) {
  Serial.print("{\"type\":\"ack\",\"");
  Serial.print(what);
  Serial.println("\":true}");
}

void printHelp() {
  Serial.println("# Kommandoer:");
  Serial.println("#   mode=manual            // skifter til manuel (pot/knapper)");
  Serial.println("#   mode=pc                // skifter til PC-styring");
  Serial.println("#   servo=1500             // µs (kun i mode=pc)");
  Serial.println("#   motor=512              // duty 0..1023 (kun i mode=pc)");
  Serial.println("#   leds=3                 // bitmask 0..7 (kun i mode=pc)");
}

// meget simpel key=value parser pr. linje
void handleSerialLine(String line) {
  line.trim();
  if (!line.length()) return;

  // del op i tokens adskilt af mellemrum
  const int MAXTOK = 10;
  String tok[MAXTOK];
  int n=0;
  int start=0;
  for (int i=0; i<=line.length() && n<MAXTOK; ++i) {
    if (i==line.length() || line[i]==' ') {
      if (i>start) tok[n++] = line.substring(start, i);
      start = i+1;
    }
  }

  for (int i=0;i<n;i++) {
    int eq = tok[i].indexOf('=');
    if (eq<0) continue;
    String k = tok[i].substring(0,eq);
    String v = tok[i].substring(eq+1);
    k.toLowerCase();
    v.trim();

    if (k=="mode") {
      if (v=="manual") { mode = MODE_MANUAL; printAck("mode"); }
      else if (v=="pc") { mode = MODE_PC; printAck("mode"); }
    } else if (k=="servo" && mode==MODE_PC) {
      int us = v.toInt();
      if (us<500) us=500; if (us>2500) us=2500;
      cmd.servo_us = (uint16_t)us;
      printAck("servo");
    } else if (k=="motor" && mode==MODE_PC) {
      int duty = v.toInt();
      if (duty<0) duty=0; if (duty>(1<<MOTOR_PWM_RES_BITS)-1) duty=(1<<MOTOR_PWM_RES_BITS)-1;
      cmd.motor_duty = (uint16_t)duty;
      printAck("motor");
    } else if (k=="leds" && mode==MODE_PC) {
      int m = v.toInt();
      if (m<0) m=0; if (m>7) m=7;
      cmd.leds = (uint8_t)m;
      printAck("leds");
    }
  }
  lastPcCmdMs = millis();
}

void setup() {
  Serial.begin(115200);
  delay(200);
  printHelp();

  pinMode(BTN_LED1_PIN, INPUT_PULLUP);
  pinMode(BTN_LED2_PIN, INPUT_PULLUP);
  pinMode(BTN_LED3_PIN, INPUT_PULLUP);

  // WiFi / ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println("! ESP-NOW init failed");
    while (1) delay(1000);
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("! Failed to add broadcast peer");
  }
}

void loop() {
  // Seriel indkommende linjer
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c=='\n' || c=='\r') {
      if (line.length()) {
        handleSerialLine(line);
        line = "";
      }
    } else {
      line += c;
      if (line.length() > 200) line = ""; // guard
    }
  }

  // Failsafe for PC-mode
  if (mode==MODE_PC && (millis() - lastPcCmdMs > 1500)) {
    mode = MODE_MANUAL;
    Serial.println("{\"type\":\"info\",\"msg\":\"PC-timeout → MANUAL\"}");
  }

  // Opdater kommandoer
  if (mode == MODE_MANUAL) {
    applyManualInputs();
  }

  // Send kommandoer periodisk
  unsigned long now = millis();
  if (now - lastCmdSendMs >= 50) {
    lastCmdSendMs = now;
    sendCommand();
  }

  // Forward telemetri til PC periodisk
  forwardTelemetryIfDue();
}
