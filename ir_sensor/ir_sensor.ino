/*
 * This sketch monitors two IR reflectance sensor arrays consisting of 
 * pololu QTR-RC sensors. The number of sensors in each array can be changed in 
 * the definitions.
 * 
 * The sketch continuously monitors the array, and averages the previous several 
 * measurements taken for each sensor. The number of sampeles averaged can be
 * configured in the definitions.
 * 
 * Calibration is accomplished automatically for the first several seconds after
 * boot. The length of time to continue calibration is configured in the definitions.
 * 
 * A target is considered detected when for a given array, the average value of at
 * least one sensor is below a specific threshold AND the average value of at least 
 * one other is above a specific threshold. These thresholds can be set in the
 * definitions. When setting these thresholds, remember that a value of 1000 represents
 * the least reflective object the sensor was calibrated to, and 0 represents the most reflective.
 * 
 * 
 */

#include <QTRSensors.h>

#define NUM_SENSORS     5      // number of sensors in each array
#define TIMEOUT         2500   // wait for 2500 microsecodns for sensor outputs to go low
#define NUM_SAMPLES     10    // how many samples over which to spread the running average.
#define CAL_TIME        30    // how long to continue calibration after boot
#define ltDetectSignal  8     // left signal pin
#define rtDetectSignal  9     //right signal pin


#define MIN_VALUE       100        //sets minimum threshold for brightness (from calibrated values, out of 1000)
#define MAX_VALUE       900        //sets max threshold for blackness (from calibrated values, out of 1000)

//create sensor array objects
QTRSensorsRC ltSensor((unsigned char[]) {2, 3, 4, 5, 6}, NUM_SENSORS, TIMEOUT);
QTRSensorsRC rtSensor((unsigned char[]) {A0, A1, A2, A3, A4}, NUM_SENSORS, TIMEOUT);

// Create arrays
unsigned int ltValues[NUM_SENSORS];
unsigned int rtValues[NUM_SENSORS];

/* History arrays to strore sensor values for running average. Array will          e.g:             Sensor #0   Sensor#1   Sensor#2
 * have number of rows equal to the number of samples defined above, and                 Sample#0         500        237        211
 * the number ofcolumns eqal to the number of sensors in each array                      Sample#1         346        123        186
 * defined above.                                                                        Sample#2         397        945        234
*/
unsigned int ltValuesHist[NUM_SAMPLES][NUM_SENSORS];
unsigned int rtValuesHist[NUM_SAMPLES][NUM_SENSORS];

unsigned long ltValuesSum[NUM_SENSORS];
unsigned long rtValuesSum[NUM_SENSORS];

unsigned int ltValuesAvg[NUM_SENSORS];
unsigned int rtValuesAvg[NUM_SENSORS];

unsigned int ltCalMin[NUM_SENSORS];
unsigned int ltCalMax[NUM_SENSORS];

unsigned int rtCalMin[NUM_SENSORS];
unsigned int rtCalMax[NUM_SENSORS];


int sampleCount = 0;

void setup() {
  Serial.begin(115200);
  
  pinMode(ltDetectSignal, OUTPUT);
  pinMode(rtDetectSignal, OUTPUT);
  digitalWrite(ltDetectSignal, LOW);
  digitalWrite(ltDetectSignal, LOW);

  pinMode(13, OUTPUT);
  

  // Initialize array values to center (half of timeout)

  for (int i = 0; i < NUM_SENSORS; i++) {
    // Set Calibration Values
    unsigned int centerVal = TIMEOUT / 2;
    ltCalMin[i] = centerVal;
    ltCalMax[i] = centerVal;
    rtCalMin[i] = centerVal;
    rtCalMax[i] = centerVal;

    // set history array values
    for (int j = 0; j < NUM_SAMPLES; j++) {
      ltValuesHist[j][i] = 500;
      rtValuesHist[j][i] = 500;
    }

    // Set sum values
    ltValuesSum[i] = (long) 500 * NUM_SAMPLES;
    rtValuesSum[i] = (long) 500 * NUM_SAMPLES;

    
  }
  
}

void loop() {
  //read sensor values
  ltSensor.read(ltValues);
  rtSensor.read(rtValues);

  for (int i = 0; i < NUM_SENSORS; i++) {
    
    // Update calibration values for the first minute of operation
    static boolean calibrated = false;
    if (!calibrated) {
      if (millis() < CAL_TIME * 1000) {
        if (ltValues[i] < ltCalMin[i]) {
          ltCalMin[i] = ltValues[i];
        }else if (ltValues[i] > ltCalMax[i]) {
          ltCalMax[i] = ltValues[i];
        }
        if (rtValues[i] < rtCalMin[i]) {
          rtCalMin[i] = rtValues[i];
        }else if (rtValues[i] > rtCalMax[i]) {
          rtCalMax[i] = rtValues[i];
        }
      }else calibrated = true;
    }

    // Keep values within calibrated range (important for post calibration operation)
    ltValues[i] = constrain(ltValues[i], ltCalMin[i], ltCalMax[i]);
    rtValues[i] = constrain(rtValues[i], rtCalMin[i], rtCalMax[i]);
    
    // Map sensor values to calibrated range
    ltValues[i] = map(ltValues[i], ltCalMin[i], ltCalMax[i], 0, 1000);
    rtValues[i] = map(rtValues[i], rtCalMin[i], rtCalMax[i], 0, 1000);
    
    
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
    digitalWrite(ltDetectSignal, HIGH);
    ltDelay = millis();
  }else if (millis() - ltDelay > 100) digitalWrite(ltDetectSignal, LOW);
  if(rtbelowMin && rtaboveMax) {
    digitalWrite(rtDetectSignal, HIGH);
    rtDelay = millis();
  }else if (millis() - rtDelay > 100) digitalWrite(rtDetectSignal, LOW);


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
//  delay(100);
}
