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
/* See the PS3BT example sketch from the USB Host Sheild library for more 
 *  information on this setup.
 */
USB Usb;
BTD Btd(&Usb); // create the Bluetooth Dongle instance
PS3BT PS3(&Btd, 0x00, 0x1A, 0x7D, 0xDA, 0x71, 0x13); // This is the dongles adress. It will change with different dongles.


/*~~~~~~~~~~~~~~~~~~~~~~~~~~ ODRIVE SETUP ~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* #define statements are like copy/paste instructions. Before compiling,
 *  the Arduino IDE will, in this case, replace "odrive_serial" with
 *  "Serial3", and then compile. This improves human readability, and 
 *  makes it easier to change the serial port if we want to.
 */
#define odrive_serial Serial3

// Create ODrive object
ODriveArduino odrive(odrive_serial);


/*~~~~~~~~~~~~~~~~~~~~~~~~~~ SERVO SETUP ~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
#define servo_serial Serial2

// Create servo objects
XYZrobotServo armBase(servo_serial, 4);
XYZrobotServo armMid(servo_serial, 3);
XYZrobotServo wrist(servo_serial, 2);
XYZrobotServo gripper(servo_serial, 1);


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~ ROS SETUP ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
#define ROS_serial Serial1

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ PINOUT ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* When using the USB Host Shiled (or any shield that uses the ICSP header),
 *  pins 50-53 should not be used, since they are connected to ICSP.
 */
// Indicators
const byte battLED0        = 49;
const byte BTconnectLED    = 13;
const byte autoModeLED     = 46;
const byte buzzer          = 48;

const byte odriveReset     = 34;
const byte shutdownPin     = 44;

//interrupts
const byte ltTargetDetect  = 20;
const byte rtTargetDetect  = 21;

/*~~~~~~~~~~~~~~~~~~~~~~ CONTROLLER SETTINGS ~~~~~~~~~~~~~~~~~~~~~~~~*/
/*The deadzones are stored as arrays, where the first value is lower than
 * the lowest the applicable axis can achieve without touching the joystick.
 * The second value is the highest. You can find these numbers for any
 * controller by uploading the PS3BT example sketch and observing the serial
 * monitor while moving the sticks around.
 */
int ltAnalogXDeadZone[2] = {115, 147};
int ltAnalogYDeadZone[2] = {103, 142};
int rtAnalogXDeadZone[2] = {115, 147};
int rtAnalogYDeadZone[2] = {115, 150};
int analogL2DeadZone = 0; //the L2 sticks only need a single value, since they rest at one extreme of their physical travel.
int analogR2DeadZone = 0;

float ltAnalogXScaler = .3;  //scales down the input from the axes. Values range from 0-1
float ltAnalogYScaler = 1;
float rtAnalogXScaler = 1;
float rtAnalogYScaler = 1;
float analogL2Scaler = 1;
float analogR2Scaler = 1;


/*~~~~~~~~~~~~~~~~~~~~~~~~ DRIVING SETTINGS ~~~~~~~~~~~~~~~~~~~~~~~~~*/
int steeringTrim      = 0.0;   // In radians, negative for left bias, positive for right bias
unsigned int accelerationTime  = 1000;  //milliseconds to transition from full reverse to full forward. Higher value means slower acceleration (and deceleration)


/*~~~~~~~~~~~~~~~~~~~~~~~~~ SERVO SETTINGS ~~~~~~~~~~~~~~~~~~~~~~~~~~*/

// Servo preset positions
int armBaseHomePos  = 251;
int armMidHomePos   = 782;
int wristHomePos    = 511;
int gripperHomePos  = 553;

int armBaseReadyPos  = 312;
int armMidReadyPos   = 745;
int wristReadyPos    = 511;
int gripperReadyPos  = 553;

/* Position limits are stored as arrays. The first value is the lowest a
 *  servo can safely move given it's construction, and the second value
 *  is the highest. These values can be determined by running the "Servo Position" 
 *  sketch, and watching the serial monitor while manipulating the servos manually.
 */
int armBasePosLimit[2]  = {251,934};  // low and high limits, respectively
int armMidPosLimit[2]   = {20, 780};
int wristPosLimit[2]    = {244, 641};
int gripperPosLimit[2]  = {84, 553};

/* This variable helps to keep servo response speed consistent regardless
 *  of how fast loop() is running.
 */
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

boolean armInitialized = false;
boolean armToHome      = false;
boolean armHome        = false;
boolean armReady       = false;
boolean autoMode   = false;

long armBaseAngle  = 0;
long armMidAngle   = 0;
long gripperAngle  = 0;
long wristAngle    = 0;
int armBasePos     = 0;
int armMidPos      = 0;
int wristPos       = 0;
int gripperPos     = 0;
long armBaseAngleLimit[2]  = {0,0};  // We'll calculate these from the PosLimits above
long armMidAngleLimit[2]   = {0,0};
long wristAngleLimit[2]    = {0,0};
long gripperAngleLimit[2]  = {0,0};

//autoMode drive command variables
float steeringThetaAuto = 0;
float driveRAuto        = 0;

long motorPos[2] = {0, 0};  // motor positions, lt and rt respectively

unsigned long currentMillis  = 0;
volatile byte targetSide = 0;  // 0 = no target, 1 = left side, 2 = right side
boolean targetDetected = false;

void setup() {
  /* The following three lines are necessary to get XYZrobot A1-16 servos 
   *  to power up and communicate correctly. Pulling the Tx pin high was 
   *  determined experimentally to be necessary. Without doing this, the 
   *  servos would not reliably power up simultaniously with the arduino.
   *  They still don't always power up reliably, but it's better.
   *  Pulling the Rx pin high is requred to recieve data. This is in the 
   *  servo documentation.
   */
  pinMode(17, INPUT_PULLUP); //necessary to recieve data from the servos. pin 17 is rx pin for Serial2
  pinMode(16, OUTPUT);
  digitalWrite(16, HIGH); //necessary to ensure servos power up correctly. pin 16 is tx pin for Serial2.
  /* End of weird servo setup requirements.*/

  // Set pinmodes
  pinMode(battLED0, OUTPUT);
  pinMode(BTconnectLED, OUTPUT);
  pinMode(autoModeLED, OUTPUT);
  
  pinMode(ltTargetDetect, INPUT);
  pinMode(rtTargetDetect, INPUT);
  
  pinMode(odriveReset, INPUT); //leave it as an floating input until needed (odrive logic is 3.3V)
  pinMode(shutdownPin, OUTPUT);
  
  digitalWrite(shutdownPin, LOW);
  
  digitalWrite(battLED0, LOW);
  digitalWrite(BTconnectLED, LOW);
  digitalWrite(autoModeLED, LOW);
  digitalWrite(buzzer, LOW);

  // Start Serial ports 
  Serial.begin(115200);
  odrive_serial.begin(115200);
  servo_serial.begin(115200);
  ROS_serial.begin(115200); 

  /* Turn off servo motors
   *  This is done so that if the arduino is reset without resetting
   *  the servos, they will relax and allow manipulation of the arm and
   *  can be safely re-initialized.
   */
  armBase.torqueOff();
  armMid.torqueOff();
  wrist.torqueOff();
  gripper.torqueOff();
  
  // Configure interrupts
  attachInterrupt(digitalPinToInterrupt(ltTargetDetect), ltTarget, RISING);
  attachInterrupt(digitalPinToInterrupt(rtTargetDetect), rtTarget, RISING);
  
  if (Usb.Init() == -1) {  // Halt boot if USB shield does not initialize
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.println(F("\r\nPS3 Bluetooth Library Started"));

  // Reset the ODrive
  //TODO: timeout?
  resetOdrive();

  //connect PS3 controller
  while (!controllerConnected()) {
    Usb.Task();
    delay(10);
  }

  /* Calculate limit angles from the limit positions. Done here in setup()
   *  so that it doesn't have to be don every time the servo positions are 
   *  calculated.
   */
  for (int i = 0; i < 2; i++) {
    armBaseAngleLimit[i] = map(armBasePosLimit[i], 0, 1023, -16500, 16500);
    armMidAngleLimit[i]  = map(armMidPosLimit[i],  0, 1023, -16500, 16500);
    wristAngleLimit[i]   = map(wristPosLimit[i],   0, 1023, -16500, 16500);
    gripperAngleLimit[i] = map(gripperPosLimit[i], 0, 1023, -16500, 16500);
  }

}

void loop() {
  
  battCheck();
  
  getCtlInputs();
  
  // Monitor for shutdown signal
  if (dPadUp) killPower();
  
  inputCtlMod();
  
  // Toggle auto mode
  if (start) autoModeSwitch();

  // Enable control of arm only if not in auto mode
  if (!autoMode) armCtl();

  // If in auto mode, do automatic stuff
  if(autoMode) autoModeCtl();
  
  driveCtl();

//  static unsigned long loopcounter = 0;
//  if (loopcounter % 1000 == 0) Serial.println(loopcounter);
//  loopcounter++;
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
  
  if (!autoMode) { //only consider stick input when not in auto mode
    steeringTheta = atan2(ltAnalogX, ltAnalogY); //determine angle of velocity vector
    driveR = sqrt(square(ltAnalogX)+square(ltAnalogY)); //determine the magnitude of the velocity vector
    driveR = constrain(driveR, 0.0, 128.0);

//    Serial << "X: " << ltAnalogX << "\tY: " << ltAnalogY << "\tTheta: " << steeringTheta << '\n';
    
  }else if (!targetDetected) { //only consider auto inputs if the target hasn't been detected
    steeringTheta = steeringThetaAuto;
    driveR = driveRAuto;
  }else { //if in auto mode, and a target has been detected, stop.
    steeringTheta = 0;
    driveR = 0;
  }

  // Apply steering trim
  steeringTheta = steeringTheta + steeringTrim;

  /* Scale up the velocity magnitude in order to more precisely apply
   *  acceleration limits while avoiding floating point math. Since the 
   *  magnitude has previously been constrained to 128, 511 is the largest 
   *  number we can scale it by and still fit the resulting value
   *  into an int datatype.
   */  
  unsigned int driveRscaled = driveR * 511.0; //scale up for application of acceleration limits. effectively constrained to 65,408
      
//  Serial << "Steering Angle: \t" << (steeringTheta * 57.2957) << "\tVelocity Magnituded: \t" << driveR << "\n\n";
  
  steeringTheta = steeringTheta - (PI / 4);  // rotate vector 45 degrees (pi/4)
  
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
  //remember these values for the next loop.
  lastLWS = LWS;
  lastRWS = RWS;

  // Map speeds back to values easier to visualize  
  LWS = map(LWS, -65408, 65408, -128, 128); 
  RWS = map(RWS, -65408, 65408, -128, 128);
  
  static boolean speedMode = false;
  // Change mode only if the robot is stopped
  if(tri == true && LWS == 0 && RWS ==0) {
    speedMode = !speedMode;
    if(speedMode) {
      tone(buzzer, 1500, 250);
    } else {
      tone(buzzer, 1000, 250);
    }
  }

  if(speedMode) {
    // SpeedMode maximizes forward velocity at the expense of control resolution and turning radius at speed
    LWS = constrain(LWS, -90, 90);
    RWS = constrain(RWS, -90, 90);

    //Serial <<"Wheel Speeds prior to mapping: LT: " << LWS << "\tRT: " << RWS << '\n';

    LWS = map(LWS, -90, 90, -150000, 150000);
    RWS = map(RWS, -90, 90, -150000, 150000);
    
    //Serial << "Wheel after mapping: \tLT: " << LWS << "\tRT: " << RWS << '\n';
    
  }else {
    Serial <<"Wheel Speeds prior to mapping: LT: " << LWS << "\tRT: " << RWS << '\n';

    LWS = map(LWS, -128, 128, -50000, 50000);
    RWS = map(RWS, -128, 128, -50000, 50000);

//    Serial <<"Wheel after mapping:\t LT: " << LWS << "\tRT: " << RWS << '\n';
  }
  
  odrive.SetVelocity(0, LWS);
  odrive.SetVelocity(1, RWS);

}

void autoModeCtl() {
  if (targetSide != 0 && !targetDetected) {
    Serial.println("TargetFound!");
    //TODO: tell ROS we found the target
    targetDetected = true;
  }
static unsigned long buzzerMillis = 0;
  if (targetDetected && millis() - buzzerMillis > 500) {
    buzzerMillis = millis();
    tone(buzzer, 2600, 250);
  }

  //get directions from ROS and convert to int
  ROScomm();
  
}

void autoModeSwitch() {
  if (armHome && !autoMode) { //initialize automode
    autoMode = true;
    targetDetected = false;
    noInterrupts();        //disable interrupts while modifying a volitile variable
    targetSide = 0;
    interrupts();
    digitalWrite(autoModeLED, HIGH);
    Serial.println("Automode!");
    ROS_serial.flush();
//    steeringThetaAuto = 0;
//    driveRAuto = 0;
    //TODO tell ROS we're in automode

  }else if ((start && autoMode) || !controllerConnected()){ //exit automode
    autoMode = false;
    targetDetected = false;
    digitalWrite(autoModeLED, LOW);
    Serial.println("No Automode!");
    //TODO: tellROS we're no longer in autoMode
  }
}

void armCtl() {
/*Controls the arm. Angles are represented as multiples of 100.
 */
  static unsigned long armPrevMillis = 0;
  if(select){
    armToHome = !armToHome; //toggle arm home state if button is pressed
    if (!armInitialized) armInitialized = true;
  }
  if(armToHome){
    deactivateArm();
  }else if (!armToHome && !armReady){  //test to see if the arm was just activated
    activateArm();
  }else if (armReady && (millis() - armPrevMillis) > (servoUpdateSpeed * 10)){  //If the arm is active, and ready, enable control. The timer helps keep input speed constant regardless of loop speed
    if (rtAnalogY != 0) {
      armBaseAngle = armBaseAngle + rtAnalogY / 2;
//      armMidAngle = armMidAngle - rtAnalogY / 2;
      armBaseAngle  = constrain(armBaseAngle, armBaseAngleLimit[0], armBaseAngleLimit[1]);
      armBasePos = map(armBaseAngle, -16500, 16500, 0, 1023);
      armBase.setPosition(armBasePos, servoUpdateSpeed);
    }
    if (analogL2btn != 0 ||analogR2btn !=0 || rtAnalogY != 0) {
      armMidAngle = armMidAngle + (analogR2btn - analogL2btn) / 4;
      armMidAngle = constrain(armMidAngle, armMidAngleLimit[0], armMidAngleLimit[1]);
      armMidPos = map(armMidAngle, -16500, 16500, 0, 1023);
      armMid.setPosition(armMidPos, servoUpdateSpeed);
    }
    if (rtAnalogX != 0) {
      wristAngle = wristAngle + rtAnalogX;
      wristAngle = constrain(wristAngle, wristAngleLimit[0], wristAngleLimit[1]);
      wristPos = map(wristAngle,  -16500, 16500, 0, 1023);
      wrist.setPosition(wristPos, servoUpdateSpeed);
    }
    if (l1 == true || r1 == true) {
      if (l1 == true && r1 == false) {
        gripperAngle = gripperAngle + 1000;
      }else if (r1 == true && l1 == false) {
        gripperAngle = gripperAngle - 1000;
      }
      gripperAngle = constrain(gripperAngle, gripperAngleLimit[0], gripperAngleLimit[1]);
      gripperPos = map(gripperAngle, -16500, 16500, 0, 1023); 
      gripper.setPosition(gripperPos, servoUpdateSpeed);
  
      
    }
  armPrevMillis = millis();
  }
}

void getCtlInputs() {
/*This function gets all the control inputs from the PS3 controller. It would probably
 * be faster to only request each input as it is needed, but this is clean and simple.
 * In the meantime, comment out any inputs that arent needed by the robot.
 */
  Usb.Task();
  if (controllerConnected()) {
    ltAnalogX = PS3.getAnalogHat(LeftHatX);
    ltAnalogY = PS3.getAnalogHat(LeftHatY);
    rtAnalogX = PS3.getAnalogHat(RightHatX);
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

  }else{
    //reset buttons and axes if the controller drops out, so the robot doesn't drive off uncontrollably
    ltAnalogX = 127;  
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

    autoModeSwitch();
  }
}

void inputCtlMod () {
  /*This function centers analog stick inputs on zero, flips the necessary axes,
   * applies deadzones, and scales inputs.
   */
  // Apply Deazones
  if (ltAnalogY < ltAnalogYDeadZone[0]) {
    ltAnalogY = map(ltAnalogY, 0, ltAnalogYDeadZone[0], 0, 127);
  }else if (ltAnalogY > ltAnalogYDeadZone[1]) {
    ltAnalogY = map(ltAnalogY, ltAnalogYDeadZone[1], 255, 128, 255);
  }else ltAnalogY = 127;

  if (ltAnalogX < ltAnalogXDeadZone[0]) {
    ltAnalogX = map(ltAnalogX, 0, ltAnalogXDeadZone[0], 0, 127);
  }else if (ltAnalogX > ltAnalogXDeadZone[1]) {
    ltAnalogX = map(ltAnalogX, ltAnalogXDeadZone[1], 255, 128, 255);
  }else ltAnalogX = 127;


  if (rtAnalogY < rtAnalogYDeadZone[0]) {
    rtAnalogY = map(rtAnalogY, 0, rtAnalogYDeadZone[0], 0, 127);
  }else if (rtAnalogY > rtAnalogYDeadZone[1]) {
    rtAnalogY = map(rtAnalogY, rtAnalogYDeadZone[1], 255, 128, 255);
  }else rtAnalogY = 127;

  if (rtAnalogX < rtAnalogXDeadZone[0]) {
    rtAnalogX = map(rtAnalogX, 0, rtAnalogXDeadZone[0], 0, 127);
  }else if (rtAnalogX > rtAnalogXDeadZone[1]) {
    rtAnalogX = map(rtAnalogX, rtAnalogXDeadZone[1], 255, 128, 255);
  }else rtAnalogX = 127;
  
  ltAnalogX = ltAnalogX - 127; //center on zero
  ltAnalogY = ltAnalogY - 127;
  rtAnalogX = rtAnalogX - 127;
  rtAnalogY = map(rtAnalogY, 0, 255, 255, 0) - 127;  //flip input direction and center on zero


//  Serial << "Axis values after deadzone: X: " << ltAnalogX << "\tY: " << ltAnalogY << '\n';
}

void deactivateArm() {
/*This function sends the arm to the home (collapsed) position, and sets servo speeds to zero
 */
  static unsigned long deactivateMillis = 0;
  
  if(select){  //run first bit only when arm is first deactivated
    armReady = false;
    armHome = false;    
    
    //send servos to home position
    armBase.setPosition(armBaseHomePos, 255);
    armMid.setPosition(armMidHomePos, 255);
    wrist.setPosition(wristHomePos,200);
    gripper.setPosition(gripperHomePos, 100);
    
    deactivateMillis = millis();
  }
  if (!armHome && millis() - deactivateMillis > 2550) { //run the following check once per second
    armBase.torqueOff();
    armMid.torqueOff();
    wrist.torqueOff();
    gripper.torqueOff();
    armHome = true;
  }
}

void activateArm() {
/*This function turns on power to the servos, and moves the arm to a ready position*/
if (!armInitialized) return;  // skip activation if the arm hasn't been initialized
static unsigned long activateMillis = 0;

  if (select) {  //run  first bit only when arm is first activated
    armHome = false;
    armReady = false;

    //send servos to ready position
    armBase.setPosition(armBaseReadyPos, 255);
    armMid.setPosition(armMidReadyPos, 255);
    wrist.setPosition(wristReadyPos,200);
    gripper.setPosition(gripperReadyPos, 100);
    
    activateMillis = millis();
  }
  
  if (millis() - activateMillis > 2550) { //run the following check once per second
    armBaseAngle = map(armBaseReadyPos, 0, 1023, -16500, 16500);
    armMidAngle  = map(armMidReadyPos,  0, 1023, -16500, 16500);
    wristAngle   = map(wristReadyPos,   0, 1023, -16500, 16500);
    gripperAngle = map(gripperReadyPos, 0, 1023, -16500, 16500);
    armReady = true;
  }
}

void ROScomm() {
  String str1 = "";
  String str2 = "";
  for (;;) {
    if (!ROS_serial.available()) {
      return;
    }
    char c = ROS_serial.read();
    if (c == ',') break;
    
    str1 += c;
  }
  for (;;) {
    char c = ROS_serial.read();
    if (c == '\n') break;

    str2 += c;
  }
  
  Serial << "Strings:" << str1 << '\t' << str2 << '\n';
  
  //convert second string to float angle in radians
  int angle = str2.toInt();
  Serial << "Angle in Degrees: \t" << angle << '\n';
  /* 0 is straight, neg is left, pos is right. For steeringThetaAuto,
   *  pi is straight, and units are radians. So add 180 to correct the direction
   *  and divide by 180 to convert to radian.
   */

   //correct offset
   angle = (angle - 180) * -1;
   steeringThetaAuto = angle * PI / 180.0;

   Serial << "Angle In Radians: \t" << steeringThetaAuto << "\n\n";

   //convert first string to velocity magnitude
   driveRAuto = str1.toFloat();
  return;  
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

    digitalWrite(battLED0, battLEDState);
    
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
      digitalWrite(battLED0, HIGH);
      armBase.torqueOff();
      armMid.torqueOff();
      wrist.torqueOff();
      gripper.torqueOff();
      while(1); //halt
      break;
    case 2:
      if (millis() - flashPrevMillis >= battFlashInterval2) {
//        Serial.println("Battery very low.");
        battLEDState = !battLEDState;
        digitalWrite(battLED0, battLEDState);
        flashPrevMillis = millis();
      }
      break;
    case 1:
      if (millis() - flashPrevMillis >= battFlashInterval1) {
//        Serial.println("Battery low.");
        battLEDState = !battLEDState;
        digitalWrite(battLED0, battLEDState);
        flashPrevMillis = millis();
      }
      break;
    case 0:
      if (battLEDState == HIGH) {
        battLEDState = LOW;
        digitalWrite(battLED0, battLEDState);
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
  static bool isConnected = false;
  static bool prevConnected = true;
  isConnected = PS3.PS3Connected;
  if (!isConnected){
    if (isConnected != prevConnected) {
      Serial.println(F("PS3 controller disconnected . . ."));
      prevConnected = isConnected;
    }
    static unsigned long connectPrevMillis = 0;
    battCheck();
    if (millis() - connectPrevMillis > 250) {
      connectPrevMillis = millis();
      BTconnectLEDState = !BTconnectLEDState;
      digitalWrite(BTconnectLED, BTconnectLEDState);
    } 
    return false;
  }else {
    if (isConnected != prevConnected) {
      Serial.println(F("PS3 controller connected!"));
      BTconnectLEDState = HIGH;
      digitalWrite(BTconnectLED, BTconnectLEDState);
      prevConnected = isConnected;
    }
    return true;
  }
}

void resetOdrive() {
  Serial.println("Resetting ODrive . . .");
  pinMode(odriveReset, OUTPUT);
  digitalWrite(odriveReset, LOW);
  delay(100);
  pinMode(odriveReset, INPUT); //back to a floating input
  delay(100);

  //wait for ODrive to come online
  odrive_serial << "r vbus_voltage\n";
  int retryCount = 1;
  while(odrive.readFloat() == 0.00 && retryCount != 11) {
    retryCount++;
    if (retryCount == 11) {
      Serial.println("\nNo Response from odrive. Canceling reset.");
      return;
    }
    delay(1000);
    odrive_serial << "r vbus_voltage\n";
    Serial.print("Retry #");
    Serial.println(retryCount);
  }

  // Once the ODrive is online, battCheck will work
  // battCheck();

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

void killPower() {
  /* When called, this function will turn off the servos, send a shutdown
   *  signal to the kill relay, and halt the loop.
   */
  //TODO: send shutdown signal to IR array?
  digitalWrite(shutdownPin, HIGH);
  armBase.torqueOff();
  armMid.torqueOff();
  wrist.torqueOff();
  gripper.torqueOff();
  Serial.print(F("Emergency shutdown signal recieved."));
  while(1); //halt
}

/*~~~~~~~~~~~~~~~~ INTERRUPT SERVICE ROUTINES (ISR) ~~~~~~~~~~~~~~~~~*/
void ltTarget() {
  if (autoMode) targetSide = 1;
}
void rtTarget() {
  if (autoMode) targetSide = 2;
}
// End of ISRs
