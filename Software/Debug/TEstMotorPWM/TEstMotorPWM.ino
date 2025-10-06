#include <Arduino.h>

#if !defined(ARDUINO_ARCH_ESP32)
  #error "Vælg 'ESP32 Wrover Module' under Tools → Board."
#endif

const int MOTOR_PWM_PIN       = 18;   // gode valg: 18, 19, 23, 25, 26, 27
const int motorPwmChannel     = 0;
const int MOTOR_PWM_FREQ_HZ   = 20000;   // 50 Hz til servo/ESC
const int MOTOR_PWM_RES_BITS  = 12;   // 0..65535


static inline void writePulseUS(int pulse_us) {
  const float period_us = 1000000.0f / MOTOR_PWM_FREQ_HZ;         // 20000 µs ved 50 Hz
  uint32_t duty = (uint32_t)((pulse_us / period_us) * ((1UL << MOTOR_PWM_RES_BITS) - 1));
  ledcWrite(motorPwmChannel, duty);
}

void setup() {
  // Motor PWM via LEDC
  int motorPwmChannel = ledcAttach(MOTOR_PWM_PIN, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);
  if (motorPwmChannel < 0) {
    Serial.println("Motor attach failed");
  }

  ledcWrite(motorPwmChannel, 0);

  // typisk ESC/servo init
  writePulseUS(1000);   // "min" i 2 sek for at arme ESC
  delay(2000);
}

void loop() {
  
  writePulseUS(1500); delay(1000);  // midt
  writePulseUS(1000); delay(1000);  // min
  writePulseUS(2000); delay(1000);  // max
  
}



