#include <QTRSensors.h>

#define NUM_SENSORS     3      // number of sensors in each array
#define TIMEOUT         2500   // wait for 2500 microsecodns for sensor outputs to go low
#define NUM_SAMPLES     10    // how many samples over which to spread the running average.
#define MIN_VALUE       200        //sets minimum threshold for brightness
#define MAX_VALUE       2300       //sets max threshold for blackness
#define ltLED           8
#define rtLED           9

//create sensor array objects
QTRSensorsRC ltSensor((unsigned char[]) {2, 3, 4}, NUM_SENSORS, TIMEOUT);
QTRSensorsRC rtSensor((unsigned char[]) {5, 6, 7}, NUM_SENSORS, TIMEOUT);

// Create arrays
unsigned int ltValues[NUM_SENSORS];
unsigned int rtValues[NUM_SENSORS];

/* History arrays to strore sensor values for running average. Array will          e.g:             Sensor #0   Sensor#1   Sensor#2
 * have number of rows equal to the number of samples defined above, and                 Sample#0        2500       2500        211
 * the number ofcolumns eqal to the number of sensors in each array                      Sample#1        2500       2500        186
 * defined above.                                                                        Sample#2        2500       2500        234
*/
unsigned int ltValuesHist[NUM_SAMPLES][NUM_SENSORS];
unsigned int rtValuesHist[NUM_SAMPLES][NUM_SENSORS];

unsigned long ltValuesSum[NUM_SENSORS];
unsigned long rtValuesSum[NUM_SENSORS];

unsigned int ltValuesAvg[NUM_SENSORS];
unsigned int rtValuesAvg[NUM_SENSORS];

int sampleCount = 0;

void setup() {
  Serial.begin(115200);
  
  pinMode(ltLED, OUTPUT);
  pinMode(rtLED, OUTPUT);
  digitalWrite(ltLED, LOW);
  digitalWrite(ltLED, LOW);

  //reset arrays to middle of the road
  for (int i = 0; i < NUM_SENSORS; i++) {
    for (int j = 0; j < NUM_SAMPLES; j++) {
      ltValuesHist[j][i] = TIMEOUT / 2;
      rtValuesHist[j][i] = TIMEOUT / 2;
    }

    ltValuesSum[i] = (long) TIMEOUT * NUM_SAMPLES / 2;
    rtValuesSum[i] = (long) TIMEOUT * NUM_SAMPLES / 2;
  }
}

void loop() {
  //read sensor values
  ltSensor.read(ltValues);
  rtSensor.read(rtValues);

  for (int i = 0; i < NUM_SENSORS; i++) {
    //update sums
    ltValuesSum[i] = ltValuesSum[i] - ltValuesHist[sampleCount][i] + ltValues[i];
    rtValuesSum[i] = rtValuesSum[i] - rtValuesHist[sampleCount][i] + rtValues[i];
    
    //update history
    ltValuesHist[sampleCount][i] = ltValues[i];
    rtValuesHist[sampleCount][i] = rtValues[i];
    
    //calculate average
    ltValuesAvg[i] = ltValuesSum[i] / NUM_SAMPLES;
    rtValuesAvg[i] = rtValuesSum[i] / NUM_SAMPLES;
  }
  
  boolean ltbelowMin = false;
  boolean ltaboveMax = false;
  boolean rtbelowMin = false;
  boolean rtaboveMax = false;
  static unsigned long ltDelay = 0;
  static unsigned long rtDelay = 0;

  //check average values against thresholds defined above
  for(unsigned char i = 0; i < NUM_SENSORS; i++) {
    //check left sensor averages
    if(ltValuesAvg[i] < MIN_VALUE) {
      ltbelowMin = true;
    }else if(ltValuesAvg[i] > MAX_VALUE) {
      ltaboveMax = true;
    }

    //check right sensor averages
    if(rtValuesAvg[i] < MIN_VALUE) {
      rtbelowMin = true;
    }else if(rtValuesAvg[i] > MAX_VALUE) {
      rtaboveMax = true;
    }
  }

  //control indicator pins
  if(ltbelowMin && ltaboveMax) {
    digitalWrite(ltLED, HIGH);
    ltDelay = millis();
  }else if (millis() - ltDelay > 100) digitalWrite(ltLED, LOW);
  if(rtbelowMin && rtaboveMax) {
    digitalWrite(rtLED, HIGH);
    rtDelay = millis();
  }else if (millis() - rtDelay > 100) digitalWrite(rtLED, LOW);


  if (sampleCount < NUM_SAMPLES -1) {
    sampleCount ++;
  } else sampleCount = 0;

//    //debugging
//  Serial.print("Sample Count: ");
//  Serial.println(sampleCount);
//  Serial.println("Sensor Values");
//  for (int i = 0; i < NUM_SENSORS; i++) {
//    Serial.print(ltValues[i]);
//    Serial.print('\t');
//  }
//  for (int i = 0; i < NUM_SENSORS; i++) {
//    Serial.print(rtValues[i]);
//    Serial.print('\t');
//  }
//  
//  Serial.println("\nSensor History");
//  for (int j = 0; j < NUM_SAMPLES; j++) {
//    for (int i = 0; i < NUM_SENSORS; i++) {
//      Serial.print(ltValuesHist[j][i]);
//      Serial.print('\t');
//    }
//    for (int i = 0; i < NUM_SENSORS; i++) {
//      Serial.print(rtValuesHist[j][i]);
//      Serial.print('\t');
//    }
//    Serial.println();
//  }
//
//  Serial.println("Sensor Sums");
//  for (int i = 0; i < NUM_SENSORS; i++) {
//    Serial.print(ltValuesSum[i]);
//    Serial.print('\t');
//  }
//  for (int i = 0; i < NUM_SENSORS; i++) {
//    Serial.print(rtValuesSum[i]);
//    Serial.print('\t');
//  }
//  Serial.println();
//  
//  Serial.println("Sensor Averages");
//  for (int i = 0; i < NUM_SENSORS; i++) {
//    Serial.print(ltValuesAvg[i]);
//    Serial.print('\t');
//  }
//  for (int i = 0; i < NUM_SENSORS; i++) {
//    Serial.print(rtValuesAvg[i]);
//    Serial.print('\t');
//  }
//  Serial.println('\n');
//  delay(1000);
}
