//ultrasonic sensor libraries
#include "NewPing.h"

//servo motors libraries
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h> 


//set servo motor constraints
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); //create new PWM servo class
#define SERVOMIN  125 // minmum pulse length count of 4096 (of the servo motor driver)
#define SERVOMAX  575 // maximum pulse length count of 4096 (of the servo motor driver)

//define ultrasonic sensors
#define R_triggerpin 45 
#define R_echopin 44
#define M_triggerpin 49 
#define M_echopin 48
#define L_triggerpin 47
#define L_echopin 46
#define Arm_triggerpin 39
#define Arm_echopin 38
#define Long_triggerpin 26
#define Long_echopin 27
#define MaxDistance 400 

//set DC drive motor constraints
#define motor1pin1 8
#define motor1pin2 9
#define motor2pin1 10
#define motor2pin2 11
#define pwm_motorA 2
#define pwm_motorB 3

int counter = 0;

int motorturnspeed = HIGH;
int drivespeed = HIGH;

NewPing sonarR(R_triggerpin, R_echopin, MaxDistance);
NewPing sonarM(M_triggerpin, M_echopin, MaxDistance);
NewPing sonarL(L_triggerpin, L_echopin, MaxDistance);
NewPing sonarArm(Arm_triggerpin, Arm_echopin, MaxDistance);

float durationR, durationM, durationL, durationArm; //timing place holder
float distanceR, distanceM, distanceL, distanceArm; //distance place holder
int iterations = 2; //set number of times the ultrasonic sensors read the input


  // Calculate the desired distance from the wall (1cm)
  float desiredDistance = 4.8;

  // Calculate the allowed range for the sensors
  float minDistance = desiredDistance - 0.5;  // 0.5cm below desired
  float maxDistance = desiredDistance + 0.5;  // 0.5cm above desired


  //bool specialfunction = true;
    //bool specialfunction = true;
  bool actionperformed = false;
  bool grabbedobject = false;
  bool timetoopenarm = false;
  bool ReleasePackage = false;
  bool navigationfastsequence = false;


unsigned long previousMillis = 0;
const unsigned long intervaltopmaze = 20000;
const unsigned long intervalbottommaze = intervaltopmaze+20000; //670000 time to release
unsigned long currentMillis;


double wantvalue = 5.5;
double Kp = 4.65;
double Ki = 0.7;
double errorL;
double errorR;
double PcontrollerL, PcontrollerR, LKiTotal, RKiTotal;
double IcontrollerL, IcontrollerR;
float PIresultL, PIresultR;
float MAX_INTEGRAL = 100;


void setup() {
  Serial.begin(9600);
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(pwm_motorA, OUTPUT);
  pinMode(pwm_motorB, OUTPUT);

  analogWrite(pwm_motorB, 120);
  analogWrite(pwm_motorA, 120);

//initialize all robot arm servos
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  pwm.setPWM(0, 0, angleToPulse(110)); //big arm up to 110
  pwm.setPWM(1, 0, angleToPulse(45)); // wrist up and over
  pwm.setPWM(3, 0, angleToPulse(80)); //gripper closed
  delay(4000);
 }




// if wantvalue = 6 and the distanceL = 8 (too far), errorL = 6-8 = -2 (negative)
// if wantvalue = 6 and the distanceL = 4 (too close), errorL = 6-4 = 2 (positive)
//therefore, when negative, increase right motor speed, and reduce left motor speed
//when positive, increase left motor speed, and decrease right motor speed
//opposite for right sensor


void loop(){
ultrasonic ();
forward();
currentMillis = millis();



errorL = wantvalue - distanceL;
errorR = wantvalue - distanceR;
//only P controller
PcontrollerL = Kp*errorL;
PcontrollerR = Kp*errorR;

LKiTotal += errorL;
RKiTotal += errorR;
IcontrollerL = Ki*LKiTotal;
IcontrollerR = Ki*RKiTotal;
//both P and PI controller
PIresultL = PcontrollerL + IcontrollerL;
PIresultR = PcontrollerR + IcontrollerR;
//anti windup to prevent over shoot like crazy
  if (LKiTotal > MAX_INTEGRAL) {
        LKiTotal = MAX_INTEGRAL;
    } else if (LKiTotal < -MAX_INTEGRAL) {
        LKiTotal = -MAX_INTEGRAL;
    }

  if (RKiTotal > MAX_INTEGRAL) {
        RKiTotal = MAX_INTEGRAL;
    } else if (RKiTotal < -MAX_INTEGRAL) {
        RKiTotal = -MAX_INTEGRAL;
    }

 

  if (!navigationfastsequence && currentMillis < intervaltopmaze) {
    PIcontrollerFAST();
    currentMillis = millis();
     //Serial.println("nav fast");
  } else if (!navigationfastsequence && currentMillis >= intervaltopmaze) {
    navigationfastsequence = true;  // Switch to navigationslow() after 30 seconds of navigationfast()
    currentMillis = millis();
    analogWrite(pwm_motorB, 110);
    analogWrite(pwm_motorA, 110);
    //Serial.println("nav switching");
  } else if (navigationfastsequence && currentMillis < intervalbottommaze) {
    PIcontroller();
    //Serial.println("nav slow");
  } else if (!ReleasePackage) {
    // Stop the car or perform any other desired action
    releasepackage();
    ReleasePackage=true;
  } 

PIcontroller();

if (ReleasePackage && !grabbedobject && distanceArm < 4.0 && distanceM > 6.0 ){
      brakes();
      delay(1000);
      backward();
      delay(270);
      brakes();
      delay(800);
      armopen();
      delay(500);
      armclose_slow();
      backward();
      delay(500);
      grabbedobject = true;
      //return;
    }

}







//ULTRASONIC QUICK FUNCTION
void ultrasonic () {
durationR = sonarR.ping_median(iterations); //the number of times it pings (5)
durationM = sonarM.ping_median(iterations); //ping by the iteration (5)
durationL = sonarL.ping_median(iterations); //ping by the iteration (5)
durationArm = sonarArm.ping_median(iterations); //ping by the iteration (5)


distanceR = (durationR/2)*0.0343; //to calculate distance, divide duration by 2 and multiply by medium of sound (0.0343)
distanceM = (durationM/2)*0.0343; //to calculate distance, divide duration by 2 and multiply by medium of sound (0.0343)
distanceL = (durationL/2)*0.0343; //to calculate distance, divide duration by 2 and multiply by medium of sound (0.0343)
distanceArm = (durationArm/2)*0.0343; //to calculate distance, divide duration by 2 and multiply by medium of sound (0.0343)


 Serial.print("Right "); 
    Serial.print(distanceR);
    Serial.println(" cm");
    delayMicroseconds(2);

 Serial.print("Middle ");
    Serial.print(distanceM);
    Serial.println(" cm");
    delayMicroseconds(2);

 Serial.print("Left ");
    Serial.print(distanceL);
    Serial.println(" cm");
    delayMicroseconds(2);

  Serial.print("Arm ");
    Serial.print(distanceArm);
    Serial.println(" cm");
    delayMicroseconds(2);



}


//ALL DC MOTOR QUICK FUNCTIONS
void left() {
   analogWrite(pwm_motorB, 130);
  analogWrite(pwm_motorA, 130);
 digitalWrite (motor1pin1, LOW); //right wheel spin CW
 digitalWrite (motor1pin2, HIGH); //NOTHING BY ITSELF: right wheel CCW
 digitalWrite (motor2pin1, HIGH); //left wheel  CW
 digitalWrite (motor2pin2, LOW); //NOTHING BY ITSELF: left wheel CCW
}
void right() {
   analogWrite(pwm_motorB, 130);
  analogWrite(pwm_motorA, 130);
 digitalWrite (motor1pin1, HIGH); //right wheel spin CW
 digitalWrite (motor1pin2, LOW); //NOTHING BY ITSELF: right wheel CCW
 digitalWrite (motor2pin1, LOW); //left wheel  CW
 digitalWrite (motor2pin2, HIGH); //NOTHING BY ITSELF: left wheel CCW
}
void forward(){
 digitalWrite (motor1pin1, LOW); //right wheel spin CW
 digitalWrite (motor1pin2, HIGH); //NOTHING BY ITSELF: right wheel CCW
 digitalWrite (motor2pin1, LOW); //left wheel  CW
 digitalWrite (motor2pin2, HIGH); //NOTHING BY ITSELF: left wheel CCW
}
void backward(){
digitalWrite (motor1pin1, HIGH); //right wheel spin CW
 digitalWrite (motor1pin2, LOW); //NOTHING BY ITSELF: right wheel CCW
 digitalWrite (motor2pin1, HIGH); //left wheel  CW
 digitalWrite (motor2pin2, LOW); //NOTHING BY ITSELF: left wheel CCW
}
void brakes(){
digitalWrite (motor1pin1, LOW); //right wheel spin CW
 digitalWrite (motor1pin2, LOW); //NOTHING BY ITSELF: right wheel CCW
 digitalWrite (motor2pin1, LOW); //left wheel  CW
 digitalWrite (motor2pin2, LOW); //NOTHING BY ITSELF: left wheel CCW
}

//ALL SERVO MOTOR QUICK FUNCTIONS
int angleToPulse(int ang){
   int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);// sets the angle of servo motor 0 to 180 for servo min and max
   //Serial.print("angle: ");Serial.print(ang);
   //Serial.print(" pulse: ");Serial.println(pulse);
   return pulse;
}

void armclose (){
 pwm.setPWM (0,0,angleToPulse(110)); //0 is the big motor
  delay(1000);
  pwm.setPWM (3,0,angleToPulse(85)); //2 is the gripper motor //0deg (close) - 35 deg (open)
  delay (1000);
 pwm.setPWM (1,0,angleToPulse(45)); //1 is the wrist motor //70deg (down) - 20 deg (up)
  delay(1000);
}
void armopen () {
  pwm.setPWM (3,0,angleToPulse(0)); //2 is the gripper motor //0deg (close) - 35 deg (open)
  delay(1000);
  pwm.setPWM (0,0,angleToPulse(0)); //0 is the big motor
  delay(1000);
  pwm.setPWM (1,0,angleToPulse(110)); //1 is the wrist motor //90deg (down) - 20 deg (up)
  delay(1000);
}
void armclose_slow () {
for( int angle=0; angle<= 80 ; angle +=1 ){
      delay(20);
        pwm.setPWM(3, 0, angleToPulse(angle) );
    }
delay (20);
for(int angle=0; angle<=110; angle +=2 ){
      delay(20);
        pwm.setPWM(0, 0, angleToPulse(angle) );
    }
delay (20);
for( int angle=110; angle>=45; angle -=2 ){
      delay(20);
        pwm.setPWM(1, 0, angleToPulse(angle) );
    }
delay (20);
}
void armopen_slow () {
for( int angle=110; angle>= 0 ; angle -=2 ){
      delay(20);
        pwm.setPWM(0, 0, angleToPulse(angle) );
    }
delay (20);
for(int angle=45; angle<=110; angle +=2 ){
      delay(20);
        pwm.setPWM(1, 0, angleToPulse(angle) );
    }
delay (20);
for( int angle=80; angle>=0; angle -=1 ){
      delay(20);
        pwm.setPWM(3, 0, angleToPulse(angle) );
    }
delay (20);
}

void navigationfast(){

  analogWrite(pwm_motorB, 255);
  analogWrite(pwm_motorA, 255);

  if (distanceL < desiredDistance && distanceR > desiredDistance)
{
    Serial.println("RIGHT");
    right();
} else if (distanceR < desiredDistance && distanceL > desiredDistance)
{
    Serial.println("LEFT");
    left();
}

  // Check the middle sensor and take action if it's within 10cm
  if (distanceM < 6.0) {
    backward();
    // Determine which way to turn based on which side is closer to the wall
    if (distanceL < distanceR) {
      // Turn right
      Serial.println("RIGHT");
      right();
    } else {
      // Turn left
      Serial.println("LEFT");
      left();
    }
    // Wait until the middle sensor is back to a safe distance
    while (distanceM < 8.0) {
      ultrasonic();
    }
    backward();
     }

}

void navigationslow(){

  analogWrite(pwm_motorB, 130);
  analogWrite(pwm_motorA, 130);


  if (distanceL < desiredDistance && distanceR > desiredDistance)
{
    Serial.println("RIGHT");
    right();
} else if (distanceR < desiredDistance && distanceL > desiredDistance)
{
    Serial.println("LEFT");
    left();
}

  // Check the middle sensor and take action if it's within 10cm
  if (distanceM < 8.0) {
    backward();
    // Determine which way to turn based on which side is closer to the wall
    if (distanceL < distanceR) {
      // Turn right
      Serial.println("RIGHT");
      right();
    } else {
      // Turn left
      Serial.println("LEFT");
      left();
    }
    // Wait until the middle sensor is back to a safe distance
    while (distanceM < 10.0) {
      ultrasonic();
    }
    backward();
     }
}


void middlesensorNavigation(){
  if (distanceM < 8.0) {
    backward();
    // Determine which way to turn based on which side is closer to the wall
    if (distanceL < distanceR) {
      // Turn right
      Serial.println("RIGHT");
      right();
    } else {
      // Turn left
      Serial.println("LEFT");
      left();
    }
    // Wait until the middle sensor is back to a safe distance
    while (distanceM < 10.0) {
      ultrasonic();
    }
    backward();
     }
}

void PIcontroller(){
// if (errorL <= 0){ //youre far away from the wall and need to move wheels Left closer to wall
// analogWrite(pwm_motorB,115 - abs(PIresultL));
// analogWrite(pwm_motorA,115 + abs (PIresultR));
// }
// else if (errorL >= 0){//youre to close to the wall and need to move wheels Right away to wall
// analogWrite(pwm_motorB,115 + abs(PIresultL));
// analogWrite(pwm_motorA,115 - abs (PIresultR));
// }

if (errorR < 0){ //youre far away from the wall and need to move wheels Left closer to wall
analogWrite(pwm_motorB,110 + abs(PIresultR));
analogWrite(pwm_motorA,110 - abs (PIresultR));

}
else if (errorR > 0){//youre to close to the wall and need to move wheels Right away to wall
analogWrite(pwm_motorB,110 - abs(PIresultR));
analogWrite(pwm_motorA,110 + abs (PIresultR));
}
  if (distanceM < 7.0) {
    backward();
    // Determine which way to turn based on which side is closer to the wall
    if (distanceL < distanceR) {
      // Turn right
      Serial.println("RIGHT");
      right();
    } else {
      // Turn left
      Serial.println("LEFT");
      left();
    }
    // Wait until the middle sensor is back to a safe distance
    while (distanceM < 9.0) {
      ultrasonic();
    }
    backward();
     }
}


void PIcontrollerFAST(){
// if (errorL <= 0){ //youre far away from the wall and need to move wheels Left closer to wall
// analogWrite(pwm_motorB,115 - abs(PIresultL));
// analogWrite(pwm_motorA,115 + abs (PIresultR));
// }
// else if (errorL >= 0){//youre to close to the wall and need to move wheels Right away to wall
// analogWrite(pwm_motorB,115 + abs(PIresultL));
// analogWrite(pwm_motorA,115 - abs (PIresultR));
// }

if (errorR < 0){ //youre far away from the wall and need to move wheels Left closer to wall
analogWrite(pwm_motorB,140 + abs(PIresultR));
analogWrite(pwm_motorA,140 - abs (PIresultR));

}
else if (errorR > 0){//youre to close to the wall and need to move wheels Right away to wall
analogWrite(pwm_motorB,140 - abs(PIresultR));
analogWrite(pwm_motorA,140 + abs (PIresultR));
}
else {
  analogWrite(pwm_motorB, 255);
  analogWrite(pwm_motorA, 255);
  digitalWrite (motor1pin1, LOW); //right wheel spin CW
 digitalWrite (motor1pin2, HIGH); //NOTHING BY ITSELF: right wheel CCW
 digitalWrite (motor2pin1, LOW); //left wheel  CW
 digitalWrite (motor2pin2, HIGH); //NOTHING BY ITSELF: left wheel CCW
}
  if (distanceM < 7.0) {
    backward();
    // Determine which way to turn based on which side is closer to the wall
    if (distanceL < distanceR) {
      // Turn right
      Serial.println("RIGHT");
      right();
    } else {
      // Turn left
      Serial.println("LEFT");
      left();
    }
    // Wait until the middle sensor is back to a safe distance
    while (distanceM < 9.0) {
      ultrasonic();
    }
    backward();
     }
}

void Pcontroller(){
 if (errorR < 0){ //youre far away from the wall and need to move wheels Left closer to wall
analogWrite(pwm_motorB,120 + abs(PcontrollerR));
analogWrite(pwm_motorA,120 - abs(PcontrollerR));
}
else if (errorR > 0){//youre to close to the wall and need to move wheels Right away to wall
analogWrite(pwm_motorB,120 - abs(PcontrollerR));
analogWrite(pwm_motorA,120 + abs(PcontrollerR));
}
// else if (errorL <= 0){ //youre far away from the wall and need to move wheels Left closer to wall
// analogWrite(pwm_motorB,120 - abs(PcontrollerL));
// analogWrite(pwm_motorA,120 + abs(PcontrollerR));
// }
// else if (errorL >= 0){//youre to close to the wall and need to move wheels Right away to wall
// analogWrite(pwm_motorB,120 + abs(PcontrollerL));
// analogWrite(pwm_motorA,120 - abs(PcontrollerR));
// }
  if (distanceM < 6.0) {
    backward();
    // Determine which way to turn based on which side is closer to the wall
    if (distanceL < distanceR) {
      // Turn right
      Serial.println("RIGHT");
      right();
    } else {
      // Turn left
      Serial.println("LEFT");
      left();
    }
    // Wait until the middle sensor is back to a safe distance
    while (distanceM < 10.0) {
      ultrasonic();
    }
    backward();
     }
}

void grabpackage() {
      armopen();
      delay(500);
      armclose_slow();
      delay(500);
}

void releasepackage(){
  brakes();
  delay(500);
  backward();
  delay(500);
  brakes();
  armopen_slow();
  delay(500);
  brakes();
  backward();
  delay(300);
  brakes();
  armclose();
  left();
  delay(1500);
}