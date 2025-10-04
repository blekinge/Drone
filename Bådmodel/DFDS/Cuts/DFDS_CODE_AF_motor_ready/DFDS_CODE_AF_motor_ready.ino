#include <Servo.h>

#include <AFMotor.h>
#include <IBusBM.h>
IBusBM ibus;

AF_DCMotor motor1(1, MOTOR12_64KHZ);
AF_DCMotor motor2(2, MOTOR12_64KHZ);
AF_DCMotor motor3(3, MOTOR12_64KHZ);
AF_DCMotor motor4(4, MOTOR12_64KHZ);
Servo servo1;
Servo servo2;
void setup() {
  Serial.begin(115200);
  ibus.begin(Serial);
   
 servo1.attach (9);
 servo2.attach (10);
}

void loop() {

  double ch1Value = readChannel(0, -100, 100, 0);
  double ch2Value = readChannel(1, -100, 100, 0);
  double ch3Value = readChannel(2, 0, 255, 0);
  double ch4Value = readChannel(3, -100, 100, 0);
  double ch6Value = readChannel(5, -100, 100, 0);
  
  if ((ch2Value == 0) && (ch4Value == 0))
  {
   
    motor1.run(RELEASE);//Rear left
    motor2.run(RELEASE);//Rear right
    motor3.run(RELEASE);//Front left
     motor1.setSpeed(ch3Value);
   motor2.setSpeed(ch3Value);
   motor3.setSpeed(255);
   servo1.write(ch1Value);
  servo2.write(ch1Value);
  }
  if ((ch2Value > 5)&& (ch6Value > 5) )
  {
  servo1.write(ch1Value);
  servo2.write(ch1Value);
    motor1.run(FORWARD);
  
    motor2.run(FORWARD);

     motor1.setSpeed(ch3Value);
   motor2.setSpeed(ch3Value);
   motor3.setSpeed(255);
  }
  
  else if ((ch2Value < -5) && (ch6Value > 5) )
  {

    motor1.run(BACKWARD);
   
    motor2.run(BACKWARD);
     motor1.setSpeed(ch3Value);
   motor2.setSpeed(ch3Value);
   motor3.setSpeed(255);
   servo1.write(ch1Value);
  servo2.write(ch1Value);
  }
else if ((ch2Value > 5) && (ch6Value < -5) )
  {

    motor1.run(FORWARD);
   
    motor2.run(BACKWARD);
     motor1.setSpeed(ch3Value);
   motor2.setSpeed(ch3Value);
   motor3.setSpeed(255);
   servo1.write(ch1Value);
  servo2.write(ch1Value);
  }
 else if ((ch2Value < -5) && (ch6Value <- 5))
  {

    motor1.run(BACKWARD);
   
    motor2.run(FORWARD);
     motor1.setSpeed(ch3Value);
   motor2.setSpeed(ch3Value);
   motor3.setSpeed(255);
   servo1.write(ch1Value);
  servo2.write(ch1Value);
  }
    else if ((ch4Value > 50) )
  {

    motor3.run(FORWARD);

   motor1.setSpeed(ch3Value);
   motor2.setSpeed(ch3Value);
   motor3.setSpeed(255);
   servo1.write(ch1Value);
  servo2.write(ch1Value);
  }
  else if ((ch4Value < -50) )
  {

    motor3.run(BACKWARD);
    motor1.setSpeed(ch3Value);
   motor2.setSpeed(ch3Value);
   motor3.setSpeed(255);
   servo1.write(ch1Value);
  servo2.write(ch1Value);
    
  }
  
else
  {
    motor3.run(RELEASE);
    motor1.run(RELEASE);
   
    motor2.run(RELEASE);
     motor1.setSpeed(ch3Value);
   motor2.setSpeed(ch3Value);
   motor3.setSpeed(255);
   servo1.write(ch1Value);
  servo2.write(ch1Value);
    
  }
  delay(200);
}

int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = ibus.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}
