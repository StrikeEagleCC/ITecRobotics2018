#include <XYZrobotServo.h>

#define servoSerial Serial2

XYZrobotServo servo1(servoSerial,1);
XYZrobotServo servo2(servoSerial,2);
XYZrobotServo servo3(servoSerial,3);
XYZrobotServo servo4(servoSerial,4);


void setup() {
  pinMode(17, INPUT_PULLUP);
  Serial.begin(115200);
  servoSerial.begin(115200);
  
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(servo1.readPosition());
  Serial.print('\t');
  Serial.print(servo2.readPosition());
  Serial.print('\t');
  Serial.print(servo3.readPosition());
  Serial.print('\t');
  Serial.print(servo4.readPosition());
  Serial.print("\n\n\n");
}
