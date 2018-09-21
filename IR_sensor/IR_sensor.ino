#include <QTRSensors.h>

/* These two lines make it simpler to concatenante serial prints. For example, istead of:
 *  Serial.print("Voltage: ");
 *  Serial.print(vbus_voltage);
 *  Serial.println("V");
 *  
 *  you can instead use:
 *  Serial << "Voltage: " << vbus_voltage << "V\n";
 */
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

#define NUM_SENSORS     3      // number of sensors in each array
#define TIMEOUT         2500   // wait for 2500 microsecodns for sensor outputs to go low
#define SIGNAL_PIN      A0

int minValue = 500;  //sets minimum threshold for brightness
int maxValue = 2000; //sets max threshold for blackness

// Create arrays
QTRSensorsRC ltFront((unsigned char[]) {2,  3,4}, NUM_SENSORS, TIMEOUT);
QTRSensorsRC rtFront((unsigned char[]) {5, 6, 7}, NUM_SENSORS, TIMEOUT);
//QTRSensorsRC ltRear((unsigned char[]) {8, 9, 10}, NUM_SENSORS, TIMEOUT);
//QTRSensorsRC rtRear((unsigned char[]) {11, 12 ,13}, NUM_SENSORS, TIMEOUT);

unsigned int sensorValues [NUM_SENSORS];

void setup() {
  pinMode(SIGNAL_PIN, OUTPUT);
  digitalWrite(SIGNAL_PIN, LOW);
  Serial.begin(115200);
}

void loop() {
  
  ltFront.read(sensorValues);
  contrastCheck();
  Serial << sensorValues[0] << '\t' << sensorValues[1] << '\t' << sensorValues[3] << '\n';
  
  rtFront.read(sensorValues);
  contrastCheck();
  Serial << sensorValues[0] << '\t' << sensorValues[1] << '\t' << sensorValues[3] << '\n';
  
//  ltRear.read(sensorValues);
//  contrastCheck();
//  Serial << sensorValues[0] << '\t' << sensorValues[1] << '\t' << sensorValues[3] << '\n';
//  
//  ltFront.read(sensorValues);
//  contrastCheck();
//  Serial << sensorValues[0] << '\t' << sensorValues[1] << '\t' << sensorValues[3] << "\n\n\n";
  
}

void contrastCheck() {
  boolean belowMin = false;
  boolean aboveMax = false;
  for(unsigned char i = 0; i < NUM_SENSORS; i++) {
    if(sensorValues[i] < minValue) {
      belowMin = true;
    }else if(sensorValues[i] > maxValue) {
      aboveMax = true;
    }
  }
  if(belowMin && aboveMax) {
    digitalWrite(SIGNAL_PIN, HIGH);
    delay(10);
  }else digitalWrite(SIGNAL_PIN, LOW);
}
