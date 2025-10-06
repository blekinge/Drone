#include <Arduino.h>
#include "driver/ledc.h"

#if defined(ESP8266)
  #error "Wrong core: select ESP32 Wrover Module."
#elif !defined(ARDUINO_ARCH_ESP32)
  #error "Not ESP32: install 'esp32 by Espressif Systems' + select ESP32 Wrover Module."
#endif

// Use HIGH_SPEED on classic ESP32
#define SERVO_MODE  LEDC_HIGH_SPEED_MODE
#define MOTOR_MODE  LEDC_HIGH_SPEED_MODE

// --- Pins (GPIO numbers) ---
static const int SERVO_PIN = 14;   // proven OK
static const int MOTOR_PIN = 25;   // try 26/27 if 25 is busy

// --- Channels & timers (different timers!) ---
static const ledc_channel_t SERVO_CH = LEDC_CHANNEL_0;
static const ledc_channel_t MOTOR_CH = LEDC_CHANNEL_1;
static const ledc_timer_t   SERVO_TMR = LEDC_TIMER_0;
static const ledc_timer_t   MOTOR_TMR = LEDC_TIMER_1;

// --- Frequencies & resolutions ---
static const uint32_t SERVO_FREQ_HZ = 50;
static const ledc_timer_bit_t SERVO_BITS = LEDC_TIMER_16_BIT; // 0..65535

// Start safe: 10 kHz @ 12-bit. You can try 20 kHz later.
static const uint32_t MOTOR_FREQ_HZ = 10000;
static const ledc_timer_bit_t MOTOR_BITS = LEDC_TIMER_12_BIT; // 0..4095

static inline void servoWriteUS(int us) {
  const float period_us = 1000000.0f / (float)SERVO_FREQ_HZ;
  const uint32_t dutyMax = (1u << SERVO_BITS) - 1u;
  uint32_t duty = (uint32_t)((us / period_us) * dutyMax);
  ledc_set_duty(SERVO_MODE, SERVO_CH, duty);
  ledc_update_duty(SERVO_MODE, SERVO_CH);
}

static inline void motorWritePercent(float pct) {
  if (pct < 0) pct = 0; if (pct > 100) pct = 100;
  const uint32_t dutyMax = (1u << MOTOR_BITS) - 1u;
  uint32_t duty = (uint32_t)(dutyMax * (pct / 100.0f));
  ledc_set_duty(MOTOR_MODE, MOTOR_CH, duty);
  ledc_update_duty(MOTOR_MODE, MOTOR_CH);
}

static void cfgTimer(ledc_mode_t mode, ledc_timer_t tmr, ledc_timer_bit_t bits, uint32_t hz, const char* name) {
  ledc_timer_config_t t = {
    .speed_mode      = mode,
    .duty_resolution = bits,
    .timer_num       = tmr,
    .freq_hz         = hz,
    .clk_cfg         = LEDC_AUTO_CLK
  };
  esp_err_t e = ledc_timer_config(&t);
  Serial.printf("%s timer cfg -> err=%d, target=%u Hz, actual=%u Hz\n",
                name, (int)e, hz, (unsigned)ledc_get_freq(mode, tmr));
}

static void cfgChannel(ledc_mode_t mode, ledc_channel_t ch, ledc_timer_t tmr, int gpio, const char* name) {
  ledc_channel_config_t c = {
    .gpio_num   = gpio,
    .speed_mode = mode,
    .channel    = ch,
    .intr_type  = LEDC_INTR_DISABLE,
    .timer_sel  = tmr,
    .duty       = 0,
    .hpoint     = 0
  };
  esp_err_t e = ledc_channel_config(&c);
  Serial.printf("%s channel cfg -> err=%d, gpio=%d, ch=%d, timer=%d\n",
                name, (int)e, gpio, (int)ch, (int)tmr);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nLEDC dual-frequency diag");

  // Configure timers (different ones)
  cfgTimer(SERVO_MODE, SERVO_TMR, SERVO_BITS, SERVO_FREQ_HZ, "SERVO");
  cfgTimer(MOTOR_MODE, MOTOR_TMR, MOTOR_BITS, MOTOR_FREQ_HZ, "MOTOR");

  // Configure channels
  cfgChannel(SERVO_MODE, SERVO_CH, SERVO_TMR, SERVO_PIN, "SERVO");
  cfgChannel(MOTOR_MODE, MOTOR_CH, MOTOR_TMR, MOTOR_PIN, "MOTOR");

  // Arm / center
  servoWriteUS(1000);
  delay(1500);
  servoWriteUS(1500);

  // Set motor to 50% steady so you can probe with a meter/scope
  motorWritePercent(50);
  Serial.println("Motor set to 50% @ 10 kHz. If no output, try GPIO26 or 27.");
}

void loop() {
  // Keep servo alive; motor stays at 50%
  servoWriteUS(1500); delay(800);
  servoWriteUS(1000); delay(800);
  servoWriteUS(2000); delay(800);
}


