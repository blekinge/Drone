// the number of the LED pin
const int ledPin = 18;  // 16 corresponds to GPIO 18
const int ledFreq = 20000;

void setup() {
  // set the LED as an output
  pinMode(ledPin, OUTPUT);
  analogWriteFrequency(ledPin,ledFreq);
}

void loop(){
  // increase the LED brightness
  for(int dutyCycle = 0; dutyCycle <= 255; dutyCycle++){   
    // changing the LED brightness with PWM
    analogWrite(ledPin, dutyCycle);
    delay(15);
  }

  // decrease the LED brightness
  for(int dutyCycle = 255; dutyCycle >= 0; dutyCycle--){
    // changing the LED brightness with PWM
    analogWrite(ledPin, dutyCycle);
    delay(15);
  }
}