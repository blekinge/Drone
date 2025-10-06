#include <Arduino.h>

#if !defined(ARDUINO_ARCH_ESP32)
  #error "Vælg 'ESP32 Wrover Module' under Tools → Board."
#endif

const int MOTOR_PWM_PIN       = 18;   // gode valg: 18, 19, 23, 25, 26, 27
const int motorPwmChannel     = 0;

const int MOTOR_PWM_FREQ_HZ  = 20000; // 20 kHz
const int MOTOR_PWM_RES_BITS = 12;    // 0..4095 (bedre match til 20 kHz)

void setup() {
  ledcSetup(motorPwmChannel, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);
  ledcAttachPin(MOTOR_PWM_PIN, motorPwmChannel);
  ledcWrite(motorPwmChannel, 0);
}

void loop() {
  int maxDuty = (1 << MOTOR_PWM_RES_BITS) - 1;
  ledcWrite(motorPwmChannel, maxDuty * 0.5); // ~50% duty
  delay(2000);
  ledcWrite(motorPwmChannel, maxDuty * 0.0); // stop
  delay(2000);
}
