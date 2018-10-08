#include <ODriveArduino.h>
#include <PS3BT.h>
#include <XYZrobotServo.h>
#include <usbhub.h> //in case the bluetooth dongle has a hub
#include <math.h>  //for trig functions

/* These two lines make it simpler to concatenante serial prints. For example, istead of:
 *  Serial.print("Voltage: ");
 *  Serial.print(vbus_voltage);
 *  Serial.println("V");
 *  
 *  you can instead use:
 *  Serial << "Voltage: " << vbus_voltage << "V\n";
 */
// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }


/*~~~~~~~~~~~~~~~~~~ USB SHIELD AND DONGLE SETUP ~~~~~~~~~~~~~~~~~~~~*/
USB Usb;
BTD Btd(&Usb); // create the Bluetooth Dongle instance
PS3BT PS3(&Btd, 0x00, 0x1A, 0x7D, 0xDA, 0x71, 0x13); // This is the dongles adress. It will change with different dongles.


/*~~~~~~~~~~~~~~~~~~~~~~~~~~ ODRIVE SETUP ~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
#define odrive_serial Serial3

// ODrive object
ODriveArduino odrive(odrive_serial);


/*~~~~~~~~~~~~~~~~~~~~~~~~~~ SERVO SETUP ~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
#define servo_serial Serial2

// Servo objects
XYZrobotServo armBase(servo_serial, 1);
XYZrobotServo armMid(servo_serial, 2);
XYZrobotServo wrist(servo_serial, 3);
XYZrobotServo gripper(servo_serial, 4);


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~ ROS SETUP ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
#define ROS_serial Serial1

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ PINOUT ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
const byte ODriveReset = 52;
const byte battLED = 50;
const byte BTconnectLED = 51;
const byte autoModeLED = 49;
const byte shutdownPin = 8;

//interrupts
const byte ltTargetDetect = 20;
const byte rtTargetDetect = 21;

/*~~~~~~~~~~~~~~~~~~~~~~ CONTROLLER SETTINGS ~~~~~~~~~~~~~~~~~~~~~~~~*/
int ltAnalogXDeadZone = 10;  //set deazone ranges each axis on each stick. Values will probably be less than 10
int ltAnalogYDeadZone = 10;
int rtAnalogXDeadZone = 10;
int rtAnalogYDeadZone = 10;
int analogL2DeadZone = 0;
int analogR2DeadZone = 0;
float ltAnalogXScaler = 1;  //scales down the input from the axes. Values range from 0-1
float ltAnalogYScaler = 1;
float rtAnalogXScaler = 1;
float rtAnalogYScaler = 1;
float analogL2Scaler = 1;
float analogR2Scaler = 1;


/*~~~~~~~~~~~~~~~~~~~~~~~~ DRIVING SETTINGS ~~~~~~~~~~~~~~~~~~~~~~~~~*/
int steeringTrim      = 0.0;   // In radians, negative for left bias, positive for right bias
unsigned int accelerationTime  = 1000;  //milliseconds to transition from full reverse to full forward. Higher value means slower acceleration (and deceleration)


/*~~~~~~~~~~~~~~~~~~~~~~~~~ SERVO SETTINGS ~~~~~~~~~~~~~~~~~~~~~~~~~~*/
boolean armBaseInvert  = true;
boolean armMidInvert   = false;
boolean wristInvert    = false;
boolean gripperInvert  = false;

// Servo preset angles. Angles range from -16500 to 16500. Units are 100th's of a degree, to avoid floating point math later.
long armBaseHomeAngle  = 0;
long armMidHomeAngle   = 0;
long wristHomeAngle    = 0;
long gripperHomeAngle  = 0;

long armBaseReadyAngle  = 0;
long armMidReadyAngle   = 0;
long wristReadyAngle    = 0;
long gripperReadyAngle  = 0;

long armBaseAngleLimit[2]  = {-165000,165000};
long armMidAngleLimit[2]   = {-165000, 165000};
long wristAngleLimit[2]    = {-165000, 165000};
long gripperAngleLimit[2]  = {-165000, 165000};

unsigned int servoUpdateSpeed  = 2; //in 10's of milliseconds (2 = 20ms)


/*~~~~~~~~~~~~~~~~~~~~~~~~ BATTERY SETTINGS ~~~~~~~~~~~~~~~~~~~~~~~~~*/

const int battWarn1          =  853;       //level for first warning, based on a mapping of float value from ODrive mapped 0,25.2,0,1023
const int battWarn2          =  804;       //level for second warning
const int battCritical       =  731;       //critical level
const int battNumSamples     =  10;        //number of samples to average
const int battSampleInterval =  1000;      //how often to take samples (in milliseconds)
const int battFlashInterval1 =  1500;      //how often to flash indicator at first warning (milliseconds)
const int battFlashInterval2 =  250;       //how often to flash indicator at second warning (milliseconds)


/*~~~~~~~~~~~~ VARIABLE DECLARATIONS AND INITIALIZATIONS ~~~~~~~~~~~~*/
// Controller inputs
int ltAnalogX      = 127;
int ltAnalogY      = 127;
int rtAnalogX      = 127;
int rtAnalogY      = 127;
int analogL2btn    = 0;
int analogR2btn    = 0;
boolean l1         = false;
boolean r1         = false;
boolean l3         = false;
boolean r3         = false;
boolean tri        = false;    
boolean sqr        = false;    
boolean circ       = false;    
boolean cross      = false;   
boolean dPadUp     = false;
boolean dPadDown   = false;
boolean dPadLeft   = false;
boolean dPadRight  = false;
boolean select     = false;
boolean start      = false;
boolean psBtn      = false;

boolean armToHome  = false;
boolean armHome    = true;
boolean armReady   = false;
boolean autoMode   = false;

long armBaseAngle  = 0;
long armMidAngle   = 0;
long gripperAngle  = 0;
long wristAngle    = 0;
int armBasePos     = 0;
int armMidPos      = 0;
int wristPos       = 0;
int gripperPos     = 0;

//autoMode drive command variables
float steeringThetaAuto = 0;
float driveRAuto        = 0;

long motorPos[2] = {0, 0};  // motor positions, lt and rt respectively

unsigned long currentMillis  = 0;
volatile byte targetDetected = 0;  // 0 = no target, 1 = left side, 2 = right side

void setup() {
  Serial.begin(115200);  
  
  if (Usb.Init() == -1) {  //presumably, this if statement stops the boot if something doesnt start.
    Serial.print(F("\r\nOSC did not start"));  //what is the "F" in the argument here? and what is \nOSC?
    while (1); //halt  //is the purpose of this just to hang the board if the usb device isn't initialized?
  }
  Serial.println(F("\r\nPS3 Bluetooth Library Started"));

  // Start Serial ports
  odrive_serial.begin(115200);
  servo_serial.begin(115200);
  ROS_serial.begin(115200);

  // Set pinmodes
  pinMode(ltTargetDetect, INPUT);
  pinMode(rtTargetDetect, INPUT);
  pinMode(ODriveReset, OUTPUT);
  pinMode(battLED, OUTPUT);
  pinMode(BTconnectLED, OUTPUT);
  pinMode(shutdownPin, OUTPUT);
  digitalWrite(shutdownPin, LOW);

  // Configure interrupts
  attachInterrupt(digitalPinToInterrupt(ltTargetDetect), ltTarget, RISING);
  attachInterrupt(digitalPinToInterrupt(rtTargetDetect), rtTarget, RISING);

  // Reset the ODrive
  resetOdrive();

  //connect PS3 controller
  while (!controllerConnected()) {
    delay(10);
  }
  
  // Get and set servo positions/angles
  armBasePos = armBase.readPosition();
  armMidPos =  armMid.readPosition();
  wristPos = wrist.readPosition();
  gripperPos = gripper.readPosition();

  armBaseAngle = map(armBasePos, 0, 1023, -165000, 165000);
  armMidAngle = map(armMidPos, 0, 1023, -165000, 165000);
  wristAngle = map(wristPos, 0, 1023, -165000, 165000);
  gripperAngle = map(gripperPos, 0, 1023, -165000, 165000);
}

void loop() {
  battCheck();
  getCtlInputs();
  
  // Monitor for shutdown signal
  if (dPadUp) {
    digitalWrite(shutdownPin, HIGH);
    Serial.print(F("Emergency shutdown signal recieved."));
    while(1); //halt
  }
  
  inputCtlMod();
  
  // Toggle auto mode
  if (start && armHome) {
    autoMode = !autoMode;
    noInterrupts();        //disable interrupts while modifying a volitile variable
    targetDetected = 0;
    interrupts();
    if (autoMode) {
      //tell ROS we're in autoMode
    } else {
      //tellROS we're no longer in autoMode
    }
  }

  // Enable control of arm only if not in auto mode
  if (!autoMode) armCtl();

  ROScomm();
  
  driveCtl();
}

void driveCtl() {
/*
 * This function implements drive control. To enable two wheel steering with a single analog stick,
 * the X and Y inputs are imagined as cartesian converted to polar coordinates. The resulting 
 * vector is rotated 45 degrees and converted back to cartesian coordintes, the X values representing
 * right wheel speed, and the Y values representing left wheel speed. Scaling of the velocity vector
 * is possible, as well as implementation of adaptive steering. Adaptive steering dampens turning input
 * as forward velocity increases.
 */
 
//Convert lt analog stick X and Y coordinates to polar coordinates
  float steeringTheta;
  float driveR;
  if (!autoMode) { 
    steeringTheta = atan2(ltAnalogX, ltAnalogY); //determine angle of velocity vector
    steeringTheta = steeringTheta + steeringTrim; //apply steering trim
    driveR = sqrt(square(ltAnalogX)+square(ltAnalogY)); //determine the magnitude of the velocity vector
    driveR = constrain(driveR, 0.0, 128.0);
  }else {
    steeringTheta = steeringThetaAuto;
    driveR = driveRAuto;
  }
  
  unsigned int driveRscaled = driveR * 511; //scale up for application of acceleration limits. effectively constrained to 65,408
      
  //Serial << "Steering Angle: \t" << (steeringTheta * 57.2957) << "\tVelocity Magnituded: \t" << driveR << "\n\n";
  
  steeringTheta = steeringTheta - .7854;  // rotate vector 45 degrees (pi/4)
  
  long LWS = 0;
  long RWS = 0;
  static long lastLWS = 0;
  static long lastRWS = 0;
  
//convert to cartesian and store as wheel speed
  LWS = driveRscaled * sin(steeringTheta);
  RWS = driveRscaled * cos(steeringTheta);
  
  //apply acceleration limits
  static unsigned long accMillis = millis();
  currentMillis = millis();
  unsigned long elapsed = currentMillis - accMillis;
  accMillis = currentMillis;
  
  if (elapsed < accelerationTime) {  //only apply acceleration limits if code is running fast enough
    // Determine acceleration increment
    float accelerationInc;
    accelerationInc = (elapsed)/ ((float) accelerationTime) * 65408; //65,408: the effective limit on wheel speed
  
    // Apply limits to left wheel wheel speed
    long LWSdiff = LWS - lastLWS;
    if (abs(LWSdiff) > accelerationInc) {
      if (LWSdiff > 0) { //accelerating in forward or decelerating in reverse
        LWS = lastLWS + accelerationInc;
      }else if (LWSdiff < 0) { //accelerating in reverse or decelerating forward
        LWS = lastLWS - accelerationInc;
      }
    }
    // Apply limits to right wheel speed
    long RWSdiff = RWS - lastRWS;
    if (abs(RWSdiff) > accelerationInc) {
      if (RWSdiff > 0) { //accelerating in forward or decelerating in reverse
        RWS = lastRWS + accelerationInc;
      }else if (RWSdiff < 0) { //accelerating in reverse or decelerating forward
        RWS = lastRWS - accelerationInc;
      }
    }
  }
  lastLWS = LWS;
  lastRWS = RWS;
  
  // Map speeds back to values easier to visualize
  LWS = map(LWS, -65408, 65408, -128, 128);
  RWS = map(RWS, -65408, 65408, -128, 128);

  static boolean speedMode = false;
  if(tri == true) speedMode = !speedMode;

  if(speedMode) {
    // SpeedMode maximizes forward velocity at the expense of control resolution and turning radius at speed
    LWS = constrain(LWS, -90, 90);
    RWS = constrain(RWS, -90, 90);

//    Serial <<"Wheel Speeds prior to mapping: LT: " << LWS << "\tRT: " << RWS << '\n';

    LWS = map(LWS, -90, 90, -150000, 150000);
    RWS = map(RWS, -90, 90, -150000, 150000);
    
//    Serial << "Wheel after mapping: \tLT: " << LWS << "\tRT: " << RWS << '\n';
    
  }else {
//    Serial <<"Wheel Speeds prior to mapping: LT: " << LWS << "\tRT: " << RWS << '\n';

    LWS = map(LWS, -128, 128, -60000, 60000);
    RWS = map(RWS, -128, 128, -60000, 60000);

//    Serial <<"Wheel after mapping:\t LT: " << LWS << "\tRT: " << RWS << '\n';
  }
  
  odrive.SetVelocity(0, LWS);
  odrive.SetVelocity(1, RWS);

}

void armCtl() {
/*Controls the arm. Angles are represented as multiples of 100.
 */
  static unsigned long armPrevMillis = 0;
  if(select){
    armToHome = !armToHome; //toggle arm home state if button is pressed
  }
  if(armToHome){
    deactivateArm();
  }else if (!armToHome && !armReady){  //test to see if the arm was just activated
    activateArm();
  }else if (armReady && millis() - armPrevMillis > servoUpdateSpeed * 10){  //If the arm is active, and ready, enable control. The timer helps keep input speed constant regardless of loop speed
    if (rtAnalogY != 0) {
      armBaseAngle = armBaseAngle + rtAnalogY / 2;
      armBaseAngle  = constrain(armBaseAngle, armBaseAngleLimit[0], armBaseAngleLimit[1]);
      armBasePos = map(armBaseAngle, -165000, 165000, 0, 1023);
      armBase.setPosition(armBasePos, servoUpdateSpeed);
    }
    if (analogL2btn != 0) {
      armMidAngle = armMidAngle + (analogL2btn - analogR2btn) / 4;
      armMidAngle = constrain(armMidAngle, armMidAngleLimit[0], armMidAngleLimit[1]);
      armMidPos = map(armMidAngle, -165000, 165000, 0, 1023);
      armMid.setPosition(armMidPos, servoUpdateSpeed);
    }
    if (rtAnalogX != 0) {
      wristAngle = wristAngle + rtAnalogX/2;
      wristAngle = constrain(wristAngle, wristAngleLimit[0], wristAngleLimit[1]);
      wristPos = map(wristAngle,  -165000, 165000, 0, 1023);
      wrist.setPosition(wristPos, servoUpdateSpeed);
    }
    if (l1 == true || r1 == true) {
      if (l1 == true && r1 == false) {
        gripperAngle = gripperAngle + 100;
      }else if (r1 == true && l1 == false) {
        gripperAngle = gripperAngle - 100;
      }
      gripperAngle = constrain(gripperAngle, gripperAngleLimit[0], gripperAngleLimit[1]);
      gripperPos = map(gripperAngle, -165000, 165000, 0, 1023);
      gripper.setPosition(gripperPos, servoUpdateSpeed); 
  
      armPrevMillis = millis();
    }
  }
}

void getCtlInputs() {
/*This function gets all the control inputs from the PS3 controller. It would probably
 * be faster to only request each input as it is needed, but this is clean and simple.
 * In the meantime, comment out any inputs that arent needed by the robot.
 */
  Usb.Task();
  if (controllerConnected()) {
    ltAnalogX = PS3.getAnalogHat(LeftHatX);  // get right stick X and Y positions and center on zero
    ltAnalogY = PS3.getAnalogHat(LeftHatY);
//    rtAnalogX = PS3.getAnalogHat(RightHatX);
    rtAnalogY = PS3.getAnalogHat(RightHatY);
    l1 = PS3.getButtonPress(L1);
    r1 = PS3.getButtonPress(R1);
    analogL2btn = PS3.getAnalogButton(L2);
    analogR2btn = PS3. getAnalogButton(R2);
//    l3 = PS3.getButtonClick(L3);
//    r3 = PS3.getButtonClick(R3);
    tri = PS3.getButtonClick(TRIANGLE);
    circ = PS3.getButtonClick(CIRCLE);
    cross = PS3.getButtonClick(CROSS);
    sqr = PS3.getButtonClick(SQUARE);
    dPadUp = PS3.getButtonPress(UP);
    dPadDown = PS3.getButtonPress(DOWN);
//    dPadLeft = PS3.getButtonClick(LEFT);
//    dPadRight = PS3.getButtonClick(RIGHT);
    start = PS3.getButtonClick(START);
    select = PS3.getButtonClick(SELECT);
    psBtn = PS3.getButtonClick(PS);

  //turn on controller LEDs for found target
    if (autoMode) {
      if (targetDetected == 1) PS3.setLedOn(LED3);
      if (targetDetected == 2) PS3.setLedOn(LED4);
    }else {
      PS3.setLedOff(LED3);
      PS3.setLedOff(LED4);
    }
    
    digitalWrite(BTconnectLED, HIGH);
  }else{
    ltAnalogX = 127;  //reset buttons and axes if the controller drops out
    ltAnalogY = 127;
    rtAnalogX = 127;
    rtAnalogY = 127;
    analogL2btn = 0;
    analogR2btn = 0;
    l1 = false;
    r1 = false;
    l3 = false;
    r3 = false;
    tri = false;
    circ = false;
    cross = false;
    sqr = false;
    dPadUp = false;
    dPadDown = false;
    dPadLeft = false;
    dPadRight = false;    
    start = false;
    select = false;
    psBtn = false;
  }
}

void inputCtlMod () {
  /*This function centers analog stick inputs on zero, flips the necessary axes,
   * applies deadzones, and scales inputs.
   */
  ltAnalogX = ltAnalogX - 127; //center on zero
  ltAnalogY = ltAnalogY - 127; //flip input direction and center on zero
  rtAnalogY = map(rtAnalogY, 0, 255, 255, 0) - 127;

//  Serial << "Axis values before deadzone: X: " << ltAnalogX << "\tY: " << ltAnalogY << '\n';
  
  if(ltAnalogX >=0){
    ltAnalogX = constrain(ltAnalogX,ltAnalogXDeadZone,128);  //apply deadzones
    ltAnalogX = map(ltAnalogX,ltAnalogXDeadZone,128,0,128 * ltAnalogXScaler); //scale and expand
  }else{
    ltAnalogX = constrain(ltAnalogX,-128,-ltAnalogXDeadZone);
    ltAnalogX = map(ltAnalogX,-128,-ltAnalogXDeadZone,-128 * ltAnalogXScaler,0);
  }
  if(ltAnalogY >=0){
    ltAnalogY = constrain(ltAnalogY,ltAnalogYDeadZone,128);
    ltAnalogY = map(ltAnalogY,ltAnalogYDeadZone,128,0,128 * ltAnalogYScaler);
  }else{
    ltAnalogY = constrain(ltAnalogY,-128,-ltAnalogYDeadZone);
    ltAnalogY = map(ltAnalogY,-128,-ltAnalogYDeadZone,-128 * ltAnalogYScaler,0);
  }  
  if(rtAnalogY >=0){
    rtAnalogY = constrain(rtAnalogY,rtAnalogYDeadZone,128);
    rtAnalogY = map(rtAnalogY,rtAnalogYDeadZone,128,0,128 * ltAnalogYScaler);
  }else{
    rtAnalogY = constrain(rtAnalogY,-128,-rtAnalogYDeadZone);
    rtAnalogY = map(rtAnalogY,-128,-rtAnalogYDeadZone,-128 * ltAnalogYScaler,0);
  }

//  Serial << "Axis values after deadzone: X: " << ltAnalogX << "\tY: " << ltAnalogY << '\n';
}

void deactivateArm() {
/*This function sends the arm to the home (collapsed) position, and sets servo speeds to zero
 */
  static unsigned long deactivateMillis = 0;
  
  if(select){  //run first bit only when arm is first deactivated
    armReady = false;
    armHome = false;    

    //map ome angles to home positions
    armBasePos = map(armBaseHomeAngle, -165000, 165000, 0, 1023);
    armMidPos = map(armMidHomeAngle, -165000, 165000, 0, 1023);
    wristPos = map(wristHomeAngle, -165000, 165000, 0, 1023);
    gripperPos = map(gripperHomeAngle, -165000, 165000, 0, 1023);

    //send servos to home position
    armBase.setPosition(armBasePos, 255);
    armMid.setPosition(armMidPos, 255);
    wrist.setPosition(wristPos,200);
    gripper.setPosition(gripperPos, 100);
  }
  if (!armHome && millis() - deactivateMillis > 1000) { //run the following check once per second
    //check if servos are at home position
    boolean servosHome[4] = {false, false, false, false};
    int servoPos[4];
    servoPos[0] = armBase.readPosition();
    servoPos[1] = armMid.readPosition();
    servoPos[2] = wrist.readPosition();
    servoPos[3] = gripper.readPosition();
    
    if (servoPos[0] > (armBasePos - 2) && servoPos[0] < (armBasePos + 2)) {
      servosHome[0] = true;
      armBase.setSpeed(0);
    }
    if (servoPos[1] > (armMidPos - 2) && servoPos[1] < (armMidPos + 2)) {
      servosHome[1] = true;
      armMid.setSpeed(0);
    }
    if (servoPos[2] > (wristPos - 2) && servoPos[2] < (wristPos + 2)) {
      servosHome[2] = true;
      wrist.setSpeed(0);
    }
    if (servoPos[3] > (gripperPos - 2) && servoPos[3] < (gripperPos + 2)) servosHome[3] = true;
  
    //if servos are in position, call it ready
    if (servosHome[0] && servosHome[1] && servosHome[2] && servosHome[3]) armHome = true;
  }
  deactivateMillis = millis();
}

void activateArm() {
/*This function turns on power to the servos, and moves the arm to a ready position*/
static unsigned long activateMillis = 0;

  if (select) {  //run  first bit only when arm is first activated
    armHome = false;
    armReady = false;

    //map ready angles to ready positions
    armBasePos = map(armBaseReadyAngle, -165000, 165000, 0, 1023);
    armMidPos = map(armMidReadyAngle, -165000, 165000, 0, 1023);
    wristPos = map(wristReadyAngle, -165000, 165000, 0, 1023);
    gripperPos = map(gripperReadyAngle, -165000, 165000, 0, 1023);

    //send servos to ready position
    armBase.setPosition(armBasePos, 255);
    armMid.setPosition(armMidPos, 255);
    wrist.setPosition(wristPos,200);
    gripper.setPosition(gripperPos, 100);
  }
  
  if (millis() - activateMillis > 1000) { //run the following check once per second
    //check if servos are at ready position
    boolean servosReady[4] = {false, false, false, false};
    int servoPos[4];
    servoPos[0] = armBase.readPosition();
    servoPos[1] = armMid.readPosition();
    servoPos[2] = wrist.readPosition();
    servoPos[3] = gripper.readPosition();
    
    if (servoPos[0] > (armBasePos - 2) && servoPos[0] < (armBasePos + 2)) servosReady[0] = true;
    if (servoPos[1] > (armMidPos - 2) && servoPos[1] < (armMidPos + 2)) servosReady[1] = true;
    if (servoPos[2] > (wristPos - 2) && servoPos[2] < (wristPos + 2)) servosReady[2] = true;
    if (servoPos[3] > (gripperPos - 2) && servoPos[3] < (gripperPos + 2)) servosReady[3] = true;
  
    //if servos are in position, call it ready
    if (servosReady[0] && servosReady[1] && servosReady[2] && servosReady[3]) armReady = true;
  }
  activateMillis = millis();
}

void ROScomm() {
  // Get motor/encoder positions
  for (int i = 0; i < 2; i++) {
    odrive_serial << "w axis" << i << ".encoder.count_in_cpr\n";
    delay(10);
    motorPos[i] = odrive.readInt();
  }
  // Pass to ROS
  
  // Get steering angle and velocity
}
void battCheck(){
/* This function samples the battery voltage at intervals
 *  and averages a number of samples before assigning 
 *  actions based on defined warning levels. Settings are
 *  defined at the top of the sketch.
 */
  //create array to store voltage samples
  static int sampleArray[battNumSamples];

  static int sampleCount = 0;

  static int sampleCurrent;
  static long sampleSum;
  static int sampleAvg;
  static byte battCode = 2;  //condition code for battery level. 0 = good, 1 = low, 2 = very low, 3 = critical

  static unsigned long samplePrevMillis = 0;
  static unsigned long flashPrevMillis = 0;
  static int battLEDState = LOW;
  
  // pseudo setup(), so we can innitialize the array and sum without having to make 
  // all these variables global
  static boolean firstRun = true;
  if (firstRun) {
    for (int i = 0; i < battNumSamples; i++) {
      sampleArray[i] = battWarn1;
    }

    sampleSum = (long) battWarn1 * battNumSamples;

    digitalWrite(battLED, battLEDState);
    
    firstRun = false;
  }

  //sample battery voltage and assign condition code once every interval
  if (millis() - samplePrevMillis >= battSampleInterval) {
    samplePrevMillis = millis();

    //sample battery voltage
    odrive_serial << "r vbus_voltage\n";
    sampleCurrent = map(odrive.readFloat(),0,26,0,1023);

    //update sum
    sampleSum = sampleSum - sampleArray[sampleCount] + sampleCurrent;

    //update sample array
    sampleArray[sampleCount] = sampleCurrent;

    //update average
    sampleAvg = sampleSum / battNumSamples;

    //update condition code
    if (sampleAvg < battCritical) {
      battCode = 3;
    }else if (sampleAvg < battWarn2) {
      battCode = 2;
    }else if (sampleAvg < battWarn1) {
      battCode = 1;
    }else battCode = 0;

    //increment sample count
    if (sampleCount < battNumSamples - 1) {
      sampleCount++;
    }else sampleCount = 0;

//    //debugging
//    Serial.print("Sample value: ");
//    Serial.println(sampleCurrent);
//    Serial.print("Sample history: ");
//    for (int i = 0; i < battNumSamples; i++) {
//      Serial.print(sampleArray[i]);
//      Serial.print('\t');
//    }
//    Serial.println();
//    Serial.print("\t Sample sum: ");
//    Serial.print(sampleSum);
//    Serial.print("\t\tSample average: ");
//    Serial.println(sampleAvg);
//    Serial.println();
  }

  //take action based on condition code
  switch (battCode) {
    case 3:
//      Serial.println("Battery critically low. Shutting down . . .");
      digitalWrite(shutdownPin, HIGH);
      digitalWrite(battLED, HIGH);
      while(1); //halt
      break;
    case 2:
      if (millis() - flashPrevMillis >= battFlashInterval2) {
//        Serial.println("Battery very low.");
        battLEDState = !battLEDState;
        digitalWrite(battLED, battLEDState);
        flashPrevMillis = millis();
      }
      break;
    case 1:
      if (millis() - flashPrevMillis >= battFlashInterval1) {
//        Serial.println("Battery low.");
        battLEDState = !battLEDState;
        digitalWrite(battLED, battLEDState);
        flashPrevMillis = millis();
      }
      break;
    case 0:
      if (battLEDState == HIGH) {
        battLEDState = LOW;
        digitalWrite(battLED, battLEDState);
      }
      break;
  }
}

boolean odriveCheck(){
  //use this function to check the axis, motors, and encoders for errors, as well as for the correct control modes
  return true;
}

boolean controllerConnected() {
  // Wait for PS3 controller to connect
  static int BTconnectLEDState = LOW;
  if (!PS3.PS3Connected){
    Serial.println(F("PS3 controller disconnected . . ."));
    unsigned long connectPrevMillis = 0;
    battCheck();
    Usb.Task();
    if (millis() - connectPrevMillis > 250) {
      connectPrevMillis = millis();
      BTconnectLEDState = !BTconnectLEDState;
      digitalWrite(BTconnectLED, BTconnectLEDState);
    } 
    return false;
  }else {
    Serial.println(F("Controller connected"));
    BTconnectLEDState = HIGH;
    digitalWrite(BTconnectLED, BTconnectLEDState);
    return true;
  }
}

void resetOdrive() {
  Serial.println("Resetting ODrive . . .");
  digitalWrite(ODriveReset, LOW);
  delay(100);
  digitalWrite(ODriveReset, HIGH);
  delay(100);

  //wait for ODrive to come online
  odrive_serial << "r vbus_voltage\n";
  while(odrive.readFloat() == 0.00) {
    delay(100);
    odrive_serial << "r vbus_voltage\n";
  }

  // Once the ODrive is online, battCheck will work
  battCheck();

  //run calibrations
  Serial.println("Calibrating motors . . .");
  int requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
  odrive.run_state(0, requested_state, false);
  odrive.run_state(1, requested_state, true);

  Serial.println("Calibrating Encoders . . .");
  requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
  odrive.run_state(0, requested_state, false);
  odrive.run_state(1, requested_state, true);

  Serial.println("Setting velocity control mode . . .");
  odrive_serial << "w axis0.controller.config.control_mode 2\n";
  odrive_serial << "w axis1.controller.config.control_mode 2\n";
  delay(10);

  Serial.println("Setting closed loop control . . .");
  requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  odrive.run_state(0, requested_state, false);
  odrive.run_state(1, requested_state, false);
  
  //check both motors are in closed loop control mode
  int timeout_ctr = 100;
  boolean stateCheck[2] = {false, false};
  do {
    delay(100);
    for (byte i = 0; i < 2; i++){
      odrive_serial << "r axis" << i <<".current_state\n";
      delay(10);
      if(odrive.readInt() == requested_state) stateCheck[i] = true; 
    }
    if (stateCheck[1] && stateCheck[2]) {
      Serial.println("Motors are good to go!");
      return;
    }
  } while (timeout_ctr-- > 0);
  
  Serial.println ("Could not set closed loop control");

}
/*~~~~~~~~~~~~~~~~ INTERRUPT SERVICE ROUTINES (ISR) ~~~~~~~~~~~~~~~~~*/
void ltTarget() {
  if (autoMode) targetDetected = 1;
}
void rtTarget() {
  if (autoMode) targetDetected = 2;
}
// End of ISRs
