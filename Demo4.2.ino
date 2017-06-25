//ME EN 1010 Arduino Sketch Template

/****************************************************************
Author Name: Eric Bellan and Jacob Kolb
Date: 4/14/16
Sketch Name: Demo 4
Sketch Description: Cannon basic
Pin Usage     Pin type  Number Hardware
IR LED        Digital   13
Reloader      Digital   10
Buttons       Analog    A7
DC motor on   Analog    5
DC motor dir. Digital   4
Right Switch  Digital   12
Left Switch   Ditiatl   11
Solenoid on   Analog    6
Solenoid Pow  Digital   7
Launcher Ser, Analog    9   
‐‐‐‐‐‐‐‐‐‐‐‐‐‐‐‐     ‐‐‐‐‐‐‐‐‐‐‐‐‐‐‐‐
****************************************************************/
/****************************
** #defines and #includes **
****************************/
#include <Servo.h>
/***********************
** Global Variables ***
***********************/
// *** Declare & Initialize Pins ***
const int reloaderPin = 10;
const int irLED = 13;
const int buttonPinVar = A7;
const int IRpin = A5;
const int dcPOW = 5;
const int dcDIR = 4;
const int rightContactSwitch = 12;
const int leftContactSwitch=  11;
const int solPOW = 6;
const int solDIR = 7;
const int servoPin = 9;

// *** Create Servo Objects ***
Servo servo;
Servo servoLoader;

// *** Declare & Initialize Program Variables ***
double cutoffD = .9;
int highPOW = 243;
int lowPOW  = 236;
int maxStripe = 38;
double launchD;
byte encoderPos;
byte xTargetHB;
byte xTargetLB;
byte driveTo[6];
byte target = 0;
int writeToServo[6];    
int thetaServo[6];
int origin = 0;
int servoReloadAngle = 52;
int reloaderServoAngle1 = 13;
int reloaderServoAngle2 = 38;
int dReload = 500;
int dReloadL = 1000;
int irDelay = 1000;
int desiredPosition = -100;
int set = 0;
int servoSmallInc = 1;
int servoLargeInc = 5;
int launcherServoAngle = 90;
int leftSwitchReading = 0;
int rightSwitchReading = 0;
int solenoidPower = 240;
int solenoidActivationTime = 500; // milliseconds
int counts = 0;
int motorPower = 255;
int buttonReading = 0;
int buttonPressed = 0;
int irSensorReading = 0;
double thetaLaunchDub[6];
double thetaServoDub[6];
double xTargetMilli;
double xTargetMeter;
double d[3] = {0.041, 0.190, 0.067};
double v0 = 3.0502;
double xTarget[6];
double H[4] = {0.1313, 0.0960, 0.0880, 0.0475} ;
double thetaS0 = 16.97;
double thetaL0 = 13.96;
unsigned long lastTime;
unsigned long currentTime = 0;
boolean previousEncoderBoolean;
boolean lastEncoderBoolean;
boolean dispBool = 0;
boolean currentEncoderBoolean = 0;
boolean motorOn = 0;//knows if motor is running or is off
boolean motorLeft =1; // keeps track of motor direction 1 = left/0 = right
/********************
** Setup Function **
********************/
void setup(void){
// PUT YOUR SETUP CODE HERE, TO RUN ONCE:

// *** Configure Digital Pins & Attach Servos ***
  pinMode(irLED,OUTPUT);
  pinMode(solPOW,OUTPUT);
  pinMode(solDIR,OUTPUT);
  pinMode(rightContactSwitch,INPUT_PULLUP);
  pinMode(leftContactSwitch,INPUT_PULLUP);
  pinMode(buttonPinVar,INPUT);  
  pinMode(dcPOW,OUTPUT);
  pinMode(dcDIR,OUTPUT);
  pinMode(IRpin, INPUT);
  servo.attach(servoPin);
  servoLoader.attach(reloaderPin);

// *** Initialize Serial Communication ***
  Serial.begin(9600);
  Serial.write(97);
//  while(Serial.available() <=0) {
//    }

// *** Take Initial Readings ***
  lastEncoderBoolean = GetEncoderBoolean();
  lastTime = millis();  
  leftSwitchReading = digitalRead(leftContactSwitch);
  rightSwitchReading = digitalRead(rightContactSwitch);

// *** Move Hardware to Desired Initial Positions ***
  servo.write(servoReloadAngle);
  servoLoader.write(reloaderServoAngle2);
  motorLeft = 1;
  desiredPosition = -100;
  MoveLauncher();
  lastEncoderBoolean = GetEncoderBoolean();

// *** Get Data from MATLAB ***
  for(int c = 0; c<6;c++){
      while(Serial.available() < 3) {
      }
      encoderPos = Serial.read();
      xTargetHB = Serial.read();
      xTargetLB = Serial.read();
      
  // *** Declare and Initialize Variables Needed for Targeting Computations ***
      int HB = xTargetHB;
      int LB = xTargetLB;
      xTargetMilli = (256*HB)+LB;
      xTargetMeter = xTargetMilli/1000;
      driveTo[c]=encoderPos;
      xTarget[c]=xTargetMeter;    
      Serial.print("Target ");
      Serial.print(c+1);
      Serial.print(", drive to stripe = ");
      Serial.print(driveTo[c]);
      Serial.print(" and aim for = ");
      Serial.print(xTarget[c]);
      Serial.println(" m.");
    }

// *** Start the Competition Timer ***
  Serial.println("Starting Competition Timer");
  digitalWrite(irLED,HIGH);
  delay(1000);
  digitalWrite(irLED,LOW); 

// *** Perform Targeting Computations ***
  TargetServoAngles(xTarget);
  for(int k = 0; k<6;k++){
    Serial.print("Target dist = ");
    Serial.print(xTarget[k]);
    Serial.print("m --> Servo angle = ");
    Serial.print(thetaServoDub[k]);
    Serial.println(" deg");
  }
} // end setup() function

/*******************
** Loop Function **
*******************/
void loop(void){
//PUT YOUR MAIN CODE HERE, TO RUN REPEATEDLY
  for(int i = 0; i<6;i++){
    launcherServoAngle = thetaServoDub[i];
    desiredPosition = driveTo[i];
    launchD = xTarget[i];
    if(i < 5 ){
      Serial.print("Target ");
      Serial.println(i+1);
      Serial.print("Moving cannon to ");
      Serial.print(desiredPosition);
      Serial.print(" and Servo to ");
      Serial.print(launcherServoAngle);
      Serial.println(" degrees.");
      MoveLauncher();
      BreakMotor();     
      Serial.print("Servo at ");
      Serial.print(launcherServoAngle);
      Serial.println(" degrees.");
      Serial.print("Cannon at position ");
      Serial.println(desiredPosition);
      Serial.print("Distance is ");
      Serial.print(launchD);
      Serial.println(" m.");
      servo.write(launcherServoAngle);
      delay(550);
      FireSolenoid();
      Reload();
    }
    //if target is last target
    else{
      Serial.print("Moving cannon to ");
      Serial.print(desiredPosition);
      Serial.print(" and Servo to  ");
      Serial.print(launcherServoAngle);
      Serial.println(" degrees.");
      MoveLauncher();
      BreakMotor();     
      Serial.print("Servo at ");
      Serial.print(launcherServoAngle);
      Serial.println(" degrees.");
      Serial.print("Cannon at position ");
      Serial.println(desiredPosition);
      Serial.print("Distance is ");
      Serial.print(launchD);
      Serial.println(" m.");
      servo.write(launcherServoAngle);
      delay(550);
      FireSolenoid();
      delay(150);
      servo.write(servoReloadAngle);
      Serial.println("Going to Home");
      desiredPosition = -100;
      MoveLauncher();
      digitalWrite(irLED,HIGH);
      delay(1000);
      digitalWrite(irLED,LOW);
      Serial.println("Timer has stopped");
      while(true){ 
        Serial.println("");
      }
    }
  }
} // end loop() function
/****************************
** User‐Defined Functions **
****************************/
int MoveLauncher(){
  if(desiredPosition > counts){
    motorLeft = 0;
  }
  else{
    motorLeft = 1;
  }
  TurnMotorOn();
  leftSwitchReading = digitalRead(leftContactSwitch);
  rightSwitchReading = digitalRead(rightContactSwitch);
  while(!(desiredPosition == counts || motorLeft == 0 && rightSwitchReading == 1 || motorLeft == 1 && leftSwitchReading == 1)){
    CountStripes();
    leftSwitchReading = digitalRead(leftContactSwitch);
    rightSwitchReading = digitalRead(rightContactSwitch);
  }
  BreakMotor();
  lastEncoderBoolean = GetEncoderBoolean();
  if(!(digitalRead(leftContactSwitch) == 0)){
    counts = 0;
    desiredPosition = 0;
    delay(5);
  }
  if(!(digitalRead(rightContactSwitch) == 0)){
    counts = maxStripe;
    desiredPosition = maxStripe;
    delay(5);
  }
}
// moves to reload position and reloads ping pong ball
int Reload(){
  //move cannon servo to reload angle
  Serial.println("****Reloading****");
  servo.write(servoReloadAngle);
  desiredPosition = 100;
  MoveLauncher();
  delay(150);
  //lift reloader arm
  servoLoader.write(reloaderServoAngle1);
  delay(500);
  //lower reloader arm
  servoLoader.write(reloaderServoAngle2);
  delay(50);
}
int FireSolenoid(){
  if(launchD > cutoffD){
    solenoidPower = highPOW;
    digitalWrite(solDIR,HIGH);
    analogWrite(solPOW,solenoidPower);
    delay(solenoidActivationTime);
    analogWrite(solPOW,0);
    Serial.print("Firing at power: ");
    Serial.println(solenoidPower);
  }
  else{
    solenoidPower = lowPOW;
    digitalWrite(solDIR,HIGH);
    analogWrite(solPOW,solenoidPower);
    delay(solenoidActivationTime);
    analogWrite(solPOW,0);
    Serial.print("Firing at power: ");
    Serial.println(solenoidPower);
  }
    Serial.println("FIRE");
    delay(150);
}
int CountStripes(){
  delay(10);
  previousEncoderBoolean = currentEncoderBoolean;
  currentEncoderBoolean = GetEncoderBoolean();
  currentTime = millis();
  unsigned long elapsedTime = currentTime - lastTime;
  if(elapsedTime > 10 && currentEncoderBoolean != lastEncoderBoolean && currentEncoderBoolean == previousEncoderBoolean){
    if(motorLeft){
      counts--;
    }
    else{
      counts++;
    }
  lastEncoderBoolean = currentEncoderBoolean;
  lastTime = currentTime;
  }
  delay(1);
}
int TurnMotorOn(){
  digitalWrite(dcDIR,motorLeft);
  analogWrite(dcPOW,motorPower);
  motorOn = 1;
}
int BreakMotor(){
  analogWrite(dcPOW,0);
  delay(10);
  motorLeft = !motorLeft;
  digitalWrite(dcDIR,motorLeft);
  analogWrite(dcPOW,motorPower);
  delay(35);
  analogWrite(dcPOW,0);
  motorOn = 0; 
}
boolean GetEncoderBoolean(){
    irSensorReading = analogRead(IRpin);
    if(irSensorReading >= 450){
      return currentEncoderBoolean=0;
    }
    else{
     return currentEncoderBoolean = 1;
    }
}
int ReadButton(){
  buttonReading = analogRead(buttonPinVar);
  if(buttonReading <= 5){
    //Serial.println("UP button pressed");
    return buttonPressed =1;
  }
  else if(buttonReading <= 154 && buttonReading >= 134){
    //Serial.println("LEFT button pressed");
    return buttonPressed=2;
  }
  else if(buttonReading >= 323 && buttonReading <= 343){
    //Serial.println("DOWN button pressed");
    return buttonPressed=3;
  }
   else if(buttonReading >= 498 && buttonReading <= 518){
    //Serial.println("RIGHT button pressed");
    return buttonPressed=4;
  } 
  else if(buttonReading >= 735 && buttonReading <= 755){
    //Serial.println("SELECT button pressed");
    return buttonPressed=5;
  }  
  else{
    //Serial.println("NO buttons pressed");
    return buttonPressed=0;
  }  
}

double Deg2Rad(double Deg){
  double Rad = ((Deg*3.1415) / 180);
  return Rad;
}

// radians to degrees
double Rad2Deg(double Rad){
  double Deg = ((Rad*180) / 3.1415);
  return Deg;
}

// quadratic
double Quadratic(double a, double b, double c, double plusOrMinus){
  double quad = ((-b)+ (plusOrMinus * sqrt(((b*b)-(4*a*c) ))))/(2*a);
  return quad;
}
// landing distance
double LandingDistance(double d[], double v0, double theta){
  double d1 = d[0];
  double d2 = d[1];
  double d3 = d[2];
  double g = 9.81;\
  double plusOrMinus = -1;
  double thetaRad = Deg2Rad(theta);
  double v0x = v0*cos(thetaRad); 
  double v0y = v0*sin(thetaRad);
  double x0 = (d2*cos(thetaRad))-(d3*sin(thetaRad));
  double y0 = d1 + (d2*sin(thetaRad)) + (d3*cos(thetaRad));
  double a  = -.5*g;
  double b = v0y;
  double c = y0;
  double tLand = Quadratic(a, b, c, plusOrMinus);
  double xLand = x0 + (v0x * tLand);
  return xLand;
}

// Range angle
double RangeAngle(double d[], double v0){
  double thetaDeg;
  double rangeAngle;
  double range;
  double xLand;
  for( thetaDeg = 25, rangeAngle = 0, range = 0; thetaDeg <= 85; thetaDeg =  thetaDeg +.01 ){
    xLand =  LandingDistance( d, v0,  thetaDeg);   
        
        if(xLand > range){
          rangeAngle = thetaDeg;
          range =  xLand;
        }
        else{
          break;
        }
      
      }
return rangeAngle;
}

// Launch angle
double LaunchAngle(double d[], double v0, double xTarget){
  double rangeAngle = RangeAngle (d, v0);
  double range  =  LandingDistance( d, v0,  rangeAngle);
  double thetaL = rangeAngle - .01;
  double xLand = range;
    while(xLand > xTarget ) {
    thetaL = thetaL + .01;
    xLand =  LandingDistance( d, v0,  thetaL);
  }
return thetaL;
}

// Servo Angle
double ServoAngle(double H[], double thetaL, double thetaS0, double thetaL0){
  double H1 = H[0];
  double H2 = H[1];
  double H3 = H[2];
  double H4 = H[3];
  double thetaLRad = Deg2Rad(thetaL);
  double thetaS0Rad = Deg2Rad(thetaS0);
  double thetaL0Rad = Deg2Rad(thetaL0);
  double K1 = H1/H2;
  double K2 = H1/H4;
  double K3 = (((H1*H1)+(H2*H2)-(H3*H3)+(H4*H4))/(2*H2*H4));
  double theta2 = thetaL - thetaL0;
  double theta2Rad = Deg2Rad(theta2);
  double A = (cos(theta2Rad)) - (K1) - (K2*cos(theta2Rad)) +(K3);\
  double B = (-2*sin(theta2Rad));
  double C = (K1) - ((K2 + 1)*cos(theta2Rad)) + (K3);
  double plusOrMinus = -1;
  double root = Quadratic(A, B, C, plusOrMinus);
  double theta4 = 2*atan(root);
  double thetaSRad = theta4 + thetaS0Rad;
  double thetaSDeg = Rad2Deg(thetaSRad);
  return thetaSDeg;
}

// Target Servo Angles
int TargetServoAngles(double xTarget[]){
    for(int b = 0 ;b < 6; b++ ){
      thetaLaunchDub[b] = LaunchAngle(d, v0, xTarget[b]);
      thetaServoDub[b] =  round(ServoAngle( H,  thetaLaunchDub[b], thetaS0, thetaL0));
//      Serial.print("Target dist = ");
//      Serial.print(xTarget[k]);
//      Serial.print("m --> Servo angle = ");
//      Serial.print(thetaServoDub[k]);
//      Serial.println(" deg");
      thetaServo[b] = (int)round(thetaLaunchDub[b]);
    }
}

// create custom headings as necessary to clearly organize your sketch
// e.g., Button functions, DC Motor functions, Servo functions, etc.
