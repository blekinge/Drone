#include <Arduino.h>

#if defined(ESP8266)
  #error "Du bygger som ESP8266! Skift til ESP32 Wrover Module under Tools → Board."
#elif !defined(ARDUINO_ARCH_ESP32)
  #error "Ikke ESP32-arkitektur. Vælg ESP32 Arduino → ESP32 Wrover Module."
#endif

const int motorPin        = 18;
const int motorPwmChannel = 0;
const int pwmFreq         = 50;   // 50 Hz (servo/ESC). Brug fx 20000 for DC-motor.
const int pwmRes          = 16;   // 0..65535

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nBoard OK: ESP32-arkitektur registreret.");
  
  bool ok1 = ledcSetup(motorPwmChannel, pwmFreq, pwmRes);  // returnerer float/bool afh. core
  ledcAttachPin(motorPin, motorPwmChannel);
  ledcWrite(motorPwmChannel, 0);

  Serial.print("ledcSetup/Attach/Write kaldt. Kanal="); Serial.println(motorPwmChannel);
  Serial.println("Hvis dette kompilerer/loader, er LEDC tilgængelig.");
}

void loop() {}
