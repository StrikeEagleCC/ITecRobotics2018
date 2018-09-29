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
int const ODriveReset = 52;
int const battLED = 6;
int const BTconnectLED = 13;
int const shutdownPin = 8;
int const detectorEnablePin = 999;
int const DetectorTriggerPin = 999;
int const detectorAlertPin = 999;

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
int steeringTrim = 0.0; // In radians, negative for left bias, positive for right bias
int accelerationTime = 1000; //milliseconds to transition from full reverse to full forward. Higher value means slower acceleration (and deceleration)


/*~~~~~~~~~~~~~~~~~~~~~~~~~ SERVO SETTINGS ~~~~~~~~~~~~~~~~~~~~~~~~~~*/
boolean armBaseInvert = true;
boolean armMidInvert = false;
boolean armWristInvert = false;
boolean gripperInvert = false;


/*~~~~~~~~~~~~~~~~~~~~~~~~ BATTERY SETTINGS ~~~~~~~~~~~~~~~~~~~~~~~~~*/
int battWarn1 = 853;  //level for first warning, based on a mapping of float value from ODrive mapped 0,25.2,0,1023
int battWarn2 = 804;  //level for second warning
int battCritical = 731; 


/*~~~~~~~~~~~~ VARIABLE DECLARATIONS AND INITIALIZATIONS ~~~~~~~~~~~~*/
int ltAnalogX = 127;
int ltAnalogY = 127;
int rtAnalogX = 127;
int rtAnalogY = 127;
int analogL2btn = 0;
int analogR2btn = 0;
boolean l1 = false;
boolean r1 = false;
boolean l3 = false;
boolean r3 = false;
boolean tri = false;    
boolean sqr = false;    
boolean circ= false;    
boolean cross= false;   
boolean dPadUp = false;
boolean dPadDown = false;
boolean dPadLeft = false;
boolean dPadRight = false;
boolean select = false;
boolean start = false;
boolean psBtn = false;

boolean armToHome = false;
boolean armHome = true;
boolean armReady = false;
boolean autoMode = false;

int armBaseAngle = 0;
int armMidAngle = 0;
int gripperAngle = 0;
int armBasePos = 0;
int armMidPos = 0;
int gripperPos = 0;

unsigned long currentMillis = 0;


void setup() {
  Serial.begin(115200);
  #if !defined(__MIPSEL__) 
    while(!Serial); //Wait for serial port to connect - used on Leonardo, Teensy, and otehr boards with built in USB CDC serial connection
    //^^^is this necessary for arduino then?
  #endif
  if (Usb.Init() == -1) {  //presumably, this if statement stops the boot if something doesnt start.
    Serial.print(F("\r\nOSC did not start"));  //what is the "F" in the argument here? and what is \nOSC?
    while (1); //halt  //is the purpose of this just to hang the board if the usb device isn't initialized?
  }
  Serial.println(F("\r\nPS3 Bluetooth Library Started"));

  odrive_serial.begin(115200);
  servo_serial.begin(115200);
  ROS_serial.begin(115200);

  pinMode(ODriveReset, OUTPUT);
  pinMode(battLED, OUTPUT);
  pinMode(BTconnectLED, OUTPUT);
  pinMode(shutdownPin, OUTPUT);
  digitalWrite(shutdownPin, LOW);

  // Reset the ODrive
  Serial.println("Resetting ODrive . . .");
  digitalWrite(ODriveReset, LOW);
  delay(100);
  digitalWrite(ODriveReset, HIGH);
  delay(100);

  odrive_serial << "r vbus_voltage\n";
  while(odrive.readFloat() == 0.00) {
    delay(100);
    odrive_serial << "r vbus_voltage\n";
  }

  // Once the ODrive is online, battCheck will work
  battCheck();

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
  int stateCheck = 0;
  do {
    delay(100);
    odrive_serial << "r axis0.current_state\n";
    if(odrive.readInt() != requested_state) stateCheck = ++stateCheck;
    odrive_serial << "r axis1.current_state\n";
    if(odrive.readInt() != requested_state) stateCheck = ++stateCheck;
    if(timeout_ctr = 1) Serial.println ("Could not set closed loop control");
  } while (stateCheck != 0 || --timeout_ctr > 0);
  
  if(timeout_ctr > 0) Serial.println("Motors are good to go!");
  
  // Wait for PS3 controller to connect
  if (!PS3.PS3Connected){
    Serial.println("Waiting for PS3 controller...");
    unsigned int connectPrevMillis = 0;
    currentMillis = millis();
    static boolean BTconnectLEDState = true;
    digitalWrite(BTconnectLED, HIGH);
    while (!PS3.PS3Connected){
      battCheck();
      Usb.Task();
      if(currentMillis - connectPrevMillis > 250) {
        BTconnectLEDState = !BTconnectLEDState;
      if(BTconnectLEDState){
      }
        if(BTconnectLEDState == true){
          digitalWrite(BTconnectLED, HIGH);
        }else{
          digitalWrite(BTconnectLED, LOW);
        }
        connectPrevMillis = currentMillis;
      }
      //wait for controller to connect
    }
    Serial.println("Controller connected");
    digitalWrite(BTconnectLED, HIGH); 
  }  
}

  
void loop() {
  Usb.Task();
  battCheck();
  getCtlInputs();
  inputCtlMod();


  if (start == true && !armHome) {  //engage auto mode!
    autoMode = !autoMode;
  }
  if (autoMode) {
    autoModeCtl();
  }

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
 
//Convert to polar coordinates  
  float steeringTheta = atan2(ltAnalogX, ltAnalogY); //determine angle of velocity vector
  steeringTheta = steeringTheta + steeringTrim; //apply steering trim
  float driveR = sqrt(square(ltAnalogX)+square(ltAnalogY)); //determine the magnitude of the velocity vector
  driveR = constrain(driveR, 0.0, 128.0);
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

    Serial <<"Wheel Speeds prior to mapping: LT: " << LWS << "\tRT: " << RWS << '\n';

    LWS = map(LWS, -90, 90, -150000, 150000);
    RWS = map(RWS, -90, 90, -150000, 150000);
    
    Serial << "Wheel after mapping: \tLT: " << LWS << "\tRT: " << RWS << '\n';
    
  }else {
    Serial <<"Wheel Speeds prior to mapping: LT: " << LWS << "\tRT: " << RWS << '\n';

    LWS = map(LWS, -128, 128, -60000, 60000);
    RWS = map(RWS, -128, 128, -60000, 60000);

    Serial << "Wheel after mapping:\t LT: " << LWS << "\tRT: " << RWS << '\n';
  }
  
  odrive.SetVelocity(0, LWS);
  odrive.SetVelocity(1, RWS);

}

//void armCtl() {
///*Controls the arm. Angles are represented as multiples of 100, so 10 degrees = 1000.
// */
//  static unsigned long armPrevMillis = 0;
//  currentMillis = millis();
//  if(select){
//    armToHome = !armToHome; //toggle arm home state if button is pressed
//  }
//  if(armToHome){
//    deactivateArm();
//  }else if (!armToHome && !armReady){  //test to see if the arm was just activated
//    activateArm();
//  }else if (armReady && currentMillis - armPrevMillis > 20){  //If the arm is active, enable control. The timer is necessary to keep input speed constant regardless of loop speed
//    armPrevMillis = currentMillis;
//    if (rtAnalogY > 0){
//      armBaseAngle = armBaseAngle + rtAnalogY / 2;
//    }
//    if (rtAnalogY < 0) {
//      armBaseAngle = armBaseAngle + rtAnalogY /2;
//    }
//    armBaseAngle = constrain(armBaseAngle,2000,22088);  //constrain angle to valid number, and keep it from crashing into itself
//    
//    if (analogL2btn > 0){
//      armMidAngle = armMidAngle + (analogL2btn /  4);
//    }else if (analogR2btn > 0){
//      armMidAngle = armMidAngle - (analogR2btn / 4);
//    }
//    armMidAngle = constrain(armMidAngle,2000,27000);  //constrain angle to valid number, and keep it from crashing into itself
//    
//    if (l1 == true){
//      gripperAngle = gripperAngle + 100;
//    }
//    if (r1 == true){
//      gripperAngle = gripperAngle - 100;
//    }
//   gripperAngle = constrain(gripperAngle,700,10000);
//   
//   moveServos(5000);
//  }
////  Serial.print(armBaseAngle);
//}


void autoModeCtl() {
  
}

  



void getCtlInputs () {
/*This function gets all the control inputs from the PS3 controller. It would probably
 * be faster to only request each input as it is needed, but this is clean and simple.
 * In the meantime, comment out any inputs that arent needed by the robot.
 */
  if (PS3.PS3Connected) {
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
    Serial.println("Waiting for PS3 controller...");
    unsigned int connectPrevMillis = 0;
    currentMillis = millis();
    boolean BTconnectLEDon = true;
    digitalWrite(BTconnectLED, HIGH);
    while (!PS3.PS3Connected){
      battCheck();
      Usb.Task();
      if(currentMillis - connectPrevMillis > 250) {
        BTconnectLEDon = !BTconnectLEDon;
        if(BTconnectLEDon){
          digitalWrite(BTconnectLED, HIGH);
        }else{
          digitalWrite(BTconnectLED, LOW);
        }
        connectPrevMillis = currentMillis;
      }
      //wait for controller to connect
    }
    Serial.println("Controller connected");
    digitalWrite(BTconnectLED, HIGH);
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

////void moveServos(int inc) {
///*This function moves the servos towards the target position determined by other functions.
// * "inc" is the number of increments to move during each interval.
// * It maps the target position to the length of the servo input pulses, and slows movement
// * as the arm approaches a zero position to reduce the likelyhood of part interferance and
// * damage. In order to avoid time consuming floating math while still enabling higher control
// * resolution, angles are represented in multiples of 100, so 10 degrees is input as 1000, 
// * and 270 degrees is input as 27000.
// */
//  static int armBasePos = 0;
//  static int armBase0Pos = 0;
//  static int armBase1Pos = 0;
//  static int armMidPos = 0;
//  static int gripperPos = 0;
//  static unsigned long servoPrevMillis = 0;
//  currentMillis = millis();
//  if (currentMillis - servoPrevMillis > 20) { //the default Servo.h library won't refresh a servo any faster than every 20ms by default. To take advantage of a refresh rate this fast, the library requires editing. Our servos are not good enough to warrant this effort
//
//    if (armBase0.attached() && armBase1.attached()) {  //ensure servo isn't killed before trying to move it. This also prevents position from changing while the servo is detached, which can result in position jumps when it is enabled again.
//      if (armBaseAngle > armBasePos) {
//        armBasePos = armBasePos + inc; 
//        armBasePos = constrain(armBasePos, 0, armBaseAngle); //prevent position from overshooting angle
//      }else if (armBaseAngle < armBasePos) {
//        if (armBasePos > 1000){
//          armBasePos = armBasePos - inc;
//          armBasePos = constrain(armBasePos, armBaseAngle, 27000);
//        }else{
//          armBasePos = armBasePos - 20;
//          armBasePos = constrain(armBasePos, armBaseAngle, 27000);
//        }
//      }
//    if(armBase0Invert == true){
//      armBase0Pos = map(armBasePos, 0, 27000, 27000, 0); //swap direction of servo movement
//    }else{
//      armBase0Pos = armBasePos;
//    }
//    if(armBase1Invert == true){
//      armBase1Pos = map(armBasePos,  0, 27000, 27000, 0); //swap direction of servo movement
//    }else {
//      armBase1Pos = armBasePos;
//    }
//    armBase0.writeMicroseconds(map(armBase0Pos + armBase0Trim, 0, 27000, servoMinus, servoMaxus));
//    armBase1.writeMicroseconds(map(armBase1Pos + armBase1Trim, 0, 27000, servoMinus, servoMaxus));
//    }else{
//      armBaseAngle = armBasePos;
//    }
//
//    if (armMid.attached()) {  //ensure servo isn't killed before trying to move it. This also prevents position from changing while the servo is detached, which can result in position jumps when it is enabled again.
//      if (armMidAngle > armMidPos) {
//        armMidPos = armMidPos + inc; 
//        armMidPos = constrain(armMidPos, 0, armMidAngle); //prevent position from overshooting angle
//      }
//      else if (armMidAngle < armMidPos) {
//        if (armMidPos > 1000){
//          armMidPos = armMidPos - inc;
//          if(armMidPos < armMidAngle){  //using the constrain here like I did everywhere else prevented it from uploading . . . hence the if statement
//            armMidPos = armMidAngle;
//          }
//        }
//        else{
//          armMidPos = armMidPos - 20;
//          armMidPos = constrain(armMidPos, armMidAngle, 27000);
//        }
//      }
//    if(armMidInvert == true){
//      armMidPos = map(armMidPos,  0, 27000, 27000, 0); //swap direction of servo movement
//    }
//    armMid.writeMicroseconds(map(armMidPos, 0 ,27000, servoMinus, servoMaxus));
//    }else{
//      armMidAngle = armMidPos;
//    }
//    
//    if(gripper.attached()) {
//      if (gripperAngle > gripperPos) {
//        gripperPos = gripperPos + inc;
//        gripperPos = constrain(gripperPos, 0, gripperAngle);
//      }else if (gripperAngle < gripperPos){
//        gripperPos = gripperPos - inc;
//        gripperPos = constrain(gripperPos, gripperAngle, 27000);
//      }
//    if(gripperInvert == true){
//      gripperPos = map(gripperPos,  0, 27000, 27000, 0); //swap direction of servo movement
//    }
//    gripper.writeMicroseconds(map(gripperPos, 0, 27000, servoMinus, servoMaxus));
//    }else{
//      gripperAngle = gripperPos;
//    }
//    servoPrevMillis = millis();
//    if (armToHome && armBasePos == 0 && armMidPos == 0 && gripperPos == gripperAngle){
//      armHome = true;
//    }
//    if(!armToHome && armBasePos == armBaseAngle && armMidPos == armMidAngle){
//      armReady = true;
//    }
//  }
//}

//void deactivateArm() {
///*This function sends the arm to the home (collapsed) position, turns off servo
// * position input, and kills power to the servos.
// */
//  if(select){  //arm was just deactivated
//    armReady = false;
//    armHome = false;    
//  }
//  
//  armBaseAngle = 0;
//  armMidAngle = 0;
//
//  if(armHome && armBase0.attached()) {
//   digitalWrite(armBase0Enable, LOW);
//   digitalWrite(armBase1Enable, LOW);
//   digitalWrite(armMidEnable, LOW);
//   digitalWrite(gripperEnable, LOW);
//   armBase0.detach();
//   armBase1.detach();
//   armMid.detach();
//   gripper.detach();
//  }else if(!armHome){
//    moveServos(50);
//  }
////  Serial.print(digitalRead(limSwA));
////  Serial.println(digitalRead(limSwB));
////  Serial.println();
//}

//void activateArm() {
///*This function turns on power to the servos, and moves the arm to a ready position*/
//  if (armHome && select) {  //Was the arm fully home when it was activated?
//    armHome = false;
//    armReady = false;
//    digitalWrite(armBase0Enable, HIGH);  //connect power and attach servo
//    digitalWrite(armBase1Enable, HIGH);
//    digitalWrite(armMidEnable, HIGH);
//    digitalWrite(gripperEnable, HIGH);
//    armBase0.attach(armBase0Pin);
//    armBase1.attach(armBase1Pin);
//    armMid.attach(armMidPin);
//    gripper.attach(gripperPin);
//    }    
//    if (armBasePos < 3000) {
//      armBaseAngle = 3000;
//    }
//    if (armMidPos < 3000) {
//      armMidAngle = 3000;
//    }
//  moveServos(50);
//}

void battCheck(){
/* This function averages the battery voltage with 10 samples over a 2.5 second period.
 * An LED flashes at a frequency dependent on the level of battery depletion.
 * Below a critical voltage, the mechanics of the robot are shut down and the program is halted.
 */
  static unsigned long sensePrevMillis = 0;
  static unsigned long battPrevMillis = 0;
  static int battLEDState = LOW;
  static int battLevel = 0;
  static int battLevel0 = battWarn1;
  static int battLevel1 = battWarn1;
  static int battLevel2 = battWarn1;
  static int battLevel3 = battWarn1;
  static int battLevel4 = battWarn1;
  static int battLevel5 = battWarn1;
  static int battLevel6 = battWarn1;
  static int battLevel7 = battWarn1;
  static int battLevel8 = battWarn1;
  static int battLevel9 = battWarn1;
  static int i = 0;

  odrive_serial << "r vbus_voltage\n";
  battLevel = map(odrive.readFloat(),0,26,0,1023);
  
  currentMillis = millis();  
  if (currentMillis - sensePrevMillis >= 250){  //Check battery voltage 4 times per second
    switch (i){  //store one reading per iteration, and average. there's probably a better way to do this with a for loop and an array or something, but this works.
      case 0:
        battLevel0 = battLevel;
        break;
      case 1:
        battLevel1 = battLevel;
        break;
      case 2:
        battLevel2 = battLevel;
        break;
      case 3:
        battLevel3 = battLevel;
        break;
      case 4:
        battLevel4 = battLevel;
        break;
      case 5:
        battLevel5 = battLevel;
        break;
      case 6:
        battLevel6 = battLevel;
        break;
      case 7:
        battLevel7 = battLevel;
        break;
      case 8:
        battLevel8 = battLevel;
        break;
      case 9:
        battLevel9 = battLevel;
        break;
    }

    //average all the values over the last 2.5 seconds
    battLevel = (battLevel0 + battLevel1 + battLevel2 + battLevel3 + battLevel4 + battLevel5 + battLevel6 + battLevel7 + battLevel8 + battLevel9) / 10;
//    Serial << "Battery Voltage: " << battLevel << '\n';
    
    if (i <= 9){
      i = ++i;
    }else{
      i=0;
    }
    sensePrevMillis = currentMillis;  
  }

  //take actions based on battery level
  if (battLevel < battCritical){
    Serial << "Battery critically low. Shutting down . . . \n";
    digitalWrite(shutdownPin, HIGH);
    while(1); //halt
  }else if((battLevel < battWarn2) && (currentMillis - battPrevMillis >= 250)){  //flash fast
    Serial << "Battery very low.\n";
    battLEDState = !battLEDState;
    digitalWrite(battLED, battLEDState);
    battPrevMillis = currentMillis;
  }else if((battLevel < battWarn1) && (currentMillis - battPrevMillis >= 1500)){  //flash slow
    Serial << "Battery low.\n";
    battLEDState = !battLEDState;
    digitalWrite(battLED, battLEDState);
    battPrevMillis = currentMillis;
  }else if ((battLevel > battWarn1) && (battLEDState == HIGH)){  //if the battery is good, turn the LED off.
    battLEDState = LOW;
    digitalWrite(battLED, battLEDState);
  }
}

boolean odriveCheck (){
  //use this function to check the axis, motors, and encoders for errors, as well as for the correct control modes
}
