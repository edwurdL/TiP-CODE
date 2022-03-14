#include "vex.h"
#include "cmath"
using namespace vex;

competition Competition;

inertial inertial1 = inertial(PORT15);
motor CON = motor(PORT6, ratio18_1, false);
motor FR = motor(PORT20, ratio6_1, false);
motor BR = motor(PORT19, ratio6_1, false);
motor FL = motor(PORT16, ratio6_1, true);
motor BL = motor(PORT17, ratio6_1, true);
motor arm = motor(PORT2, ratio36_1, true);
motor mogo = motor(PORT11, ratio36_1, false);
motor mogo2 = motor(PORT1, ratio36_1, true);


controller Controller1 = vex::controller();

void pushinP(bool push){
  Pneu.set(push);
}
float armUWant = 0;
int armUp(){
  arm.setRotation(0,degrees);
  arm.spinFor(armUWant * 7,degrees);
  arm.stop(hold);
  return 1;
}
float armDWant = 0;
int armDown(){
  arm.setRotation(0,degrees);
  arm.spinFor(armDWant * -7,degrees);
  arm.stop(hold);
  return 1;
}


void resetInertial(){
  inertial1.setRotation(0,degrees);
}

void resetTrack (){
  TL.setPosition(0,degrees);
  TR.setPosition(0,degrees);
  vex::task::sleep(100);
}

double pi = atan(1)*4;

float mogoUWant = 1;
void mogoUp(){

  mogoUWant *= 5;
  mogo.setRotation(0, degrees);
  mogo2.setRotation(0, degrees);
  mogo.setVelocity(80, pct);
  mogo2.setVelocity(80, pct);
  mogo.rotateTo(mogoUWant, degrees, false);
  mogo2.rotateTo(mogoUWant, degrees);

}

float mogoDWant = 1;
int mogoDown(){

  mogoDWant *= 5;
  mogo.setRotation(0, degrees);
  mogo2.setRotation(0, degrees);
  mogo.setVelocity(80, pct);
  mogo2.setVelocity(80, pct);
  mogo.rotateTo(-mogoDWant, degrees, false);
  mogo2.rotateTo(-mogoDWant, degrees);
return 1;
}

int printTrackR(){
  while (1==1){
    Brain.Screen.newLine();
    Brain.Screen.print(TR.position(degrees));

  }

  return 1;
}

void leftBoth(){



  Brain.Screen.print(inertial1.rotation(degrees));
  Brain.Screen.print("finished");
}

void skills(){

}

void drawgrid(){
  Brain.Screen.setFillColor(yellow);
  Brain.Screen.drawRectangle(0,0,240,240);
  Brain.Screen.setPenColor(black);
  Brain.Screen.printAt(90, 120, true, "Skills");
  Brain.Screen.setFillColor(purple);
  Brain.Screen.drawRectangle(240,0,240,240);
  Brain.Screen.printAt(300, 120, true, "Competition");
}

void compgrid(){
  Brain.Screen.clearScreen();
  Brain.Screen.setFillColor(orange);
  Brain.Screen.drawRectangle(0, 0, 240, 120);
  Brain.Screen.printAt(80, 70, true, "LeftBoth");
  Brain.Screen.setFillColor(green);
  Brain.Screen.drawRectangle(240, 0, 240, 120);
  Brain.Screen.printAt(350, 70, true, "WP");
  Brain.Screen.setFillColor(cyan);
  Brain.Screen.drawRectangle(0, 120, 240, 120);
  Brain.Screen.printAt(80, 190, true, "LeftSide");
  Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(240,120, 240, 120);
  Brain.Screen.printAt(330, 190, true, "Center");
}

float Kp = 0.018;
float Ki = 0.000002;
float Kd = 0.095;//PID goes brrrrr

float desiredValue = 0.0;

float error;
float prevError = 0;
float derivative;
float totalError = 0;

double latMotorSpeed = 0.0;

int PID(){
  resetTrack();
  while(1){
  float TRPosition = TR.position(degrees);
  float TLPosition = TL.position(degrees);

  float averagePosition  = (TRPosition + TLPosition)/2;
  float conversion = (desiredValue * 360)/(2.75 * pi);

  error = conversion - averagePosition;
  derivative = error - prevError;
  totalError += error;

  latMotorSpeed = error * Kp + derivative * Kd + totalError * Ki;

  FR.spin(fwd, latMotorSpeed, volt);
  FL.spin(fwd, latMotorSpeed, volt);
  BR.spin(fwd, latMotorSpeed, volt);
  BL.spin(fwd, latMotorSpeed, volt);

  if((error < 200) && latMotorSpeed < 2.0){
    FR.stop(brake);
    BR.stop(brake);
    FL.stop(brake);
    BL.stop(brake);
    break;
  }

  prevError = error;
  vex::task::sleep(20);
  }
  return 1;
}

float rKp = 0.018;
float rKi = 0.000002;
float rKd = 0.095;//PID goes brrrrr

float rdesiredValue = 0.0;

float rerror;
float rprevError = 0;
float rderivative;
float rtotalError = 0;

double rlatMotorSpeed = 0.0;

int rPID(){
  resetInertial();
  resetTrack();
  while(1){
  float TRPosition = (TR.position(degrees));
  float TLPosition = (TL.position(degrees));

  float raveragePosition  = fabs(TRPosition + TLPosition)/2;
  float rconversion = (rdesiredValue * 360)/(2.75 * pi);

  rerror = rconversion - raveragePosition;
  rderivative = rerror - rprevError;
  rtotalError += rerror;

  rlatMotorSpeed = rerror * rKp + rderivative * rKd + rtotalError * rKi;

  FR.spin(reverse, rlatMotorSpeed, volt);
  FL.spin(reverse, rlatMotorSpeed, volt);
  BR.spin(reverse, rlatMotorSpeed, volt);
  BL.spin(reverse, rlatMotorSpeed, volt);

  if((rerror < 140) && rlatMotorSpeed < 1.5){
    FR.stop(brake);
    BR.stop(brake);
    FL.stop(brake);
    BL.stop(brake);
    break;
  }
  rprevError = rerror;
  vex::task::sleep(20);
  }
  return 1;
}

float turnLKp = 0.1525;
float turnLKi = 0.00000001;
float turnLKd = 0.09;//PID goes brrrrr

float turnLdesiredValue = 0.0;

float turnLerror;
float turnLprevError = 0;
float turnLderivative;
float turnLtotalError = 0;

double turnLSpeed = 0.0;

int tleftPID(){
  resetInertial();
  resetTrack();
  while(1){

  turnLerror = fabs(turnLdesiredValue) - fabs(inertial1.rotation(degrees));
  turnLderivative = turnLerror - turnLprevError;
  turnLtotalError += turnLerror;

  turnLSpeed = turnLerror * turnLKp + turnLderivative * turnLKd + turnLtotalError * turnLKi;
  Brain.Screen.print(inertial1.rotation(degrees));
  Brain.Screen.print(turnLSpeed);
  Brain.Screen.newLine();

  if(turnLSpeed > 10){
    turnLSpeed = 10;
  }

  FR.spin(fwd, turnLSpeed, volt);
  FL.spin(reverse, turnLSpeed, volt);
  BR.spin(fwd, turnLSpeed, volt);
  BL.spin(reverse, turnLSpeed, volt);

  if((turnLerror < 20) && turnLSpeed < 2.0){
    FR.stop(brake);
    BR.stop(brake);
    FL.stop(brake);
    BL.stop(brake);
    break;
  }
  turnLprevError = turnLerror;
  vex::task::sleep(20);
  }
  return 1;  
}

float turnRKp = 0.1525;
float turnRKi = 0.00000001;
float turnRKd = 0.09;//PID goes brrrrr

float turnRdesiredValue = 0.0;

float turnRerror;
float turnRprevError = 0;
float turnRderivative;
float turnRtotalError = 0;

double turnRSpeed = 0.0;

int trightPID(){
  resetInertial();
  resetTrack();
  while(1){

  turnRerror = fabs(turnRdesiredValue) - fabs(inertial1.rotation(degrees));
  turnRderivative = turnRerror - turnRprevError;
  turnRtotalError += turnRerror;

  turnRSpeed = turnRerror * turnRKp + turnRderivative * turnRKd + turnRtotalError * turnRKi;
  Brain.Screen.print(inertial1.rotation(degrees));
  Brain.Screen.print(turnRSpeed);
  Brain.Screen.newLine();

  if(turnRSpeed > 10){
    turnRSpeed = 10;
  }

  FR.spin(reverse, turnRSpeed, volt);
  FL.spin(fwd, turnRSpeed, volt);
  BR.spin(reverse, turnRSpeed, volt);
  BL.spin(fwd, turnRSpeed, volt);

  if((turnRerror < 15) && turnRSpeed < 2.0){
    FR.stop(brake);
    BR.stop(brake);
    FL.stop(brake);
    BL.stop(brake);
    break;
  }
  turnRprevError = turnRerror;
  vex::task::sleep(20);
  }
  return 1;  
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  //int screenstatus = 0;
  /*
  drawgrid();
  Brain.Screen.render(true, false);
  while(1){
    if(Brain.Screen.pressing()){
      int XPress = Brain.Screen.xPosition();
      int YPress = Brain.Screen.yPosition();
      if((XPress >= 0 && XPress <= 240) && (YPress >= 0 && YPress <= 240) && screenstatus == 0){
        wait(250, msec);
        screenstatus = 1;
        Brain.Screen.clearScreen();
        Brain.Screen.render();
        skills();
      }
      if((XPress >=240 && XPress <= 480) && (YPress >= 0 && YPress <= 240) && screenstatus == 0){
        wait(250, msec);
        screenstatus = 2;
        compgrid();
        Brain.Screen.render();
        }
        if(Brain.Screen.pressing()) {
        if((screenstatus == 2) && (XPress <= 240 && XPress >= 0) && (YPress <= 120 && YPress >= 0)){
          screenstatus = 3;
          Brain.Screen.clearScreen();
          Brain.Screen.render();
          leftBoth();
        } else if((screenstatus == 2) && (XPress <= 240 && XPress >= 0) && (YPress >= 120 && YPress <= 240)){
          screenstatus = 3;
          Brain.Screen.clearScreen();
          Brain.Screen.render();
          LeftSide();
        } else if ((screenstatus == 2) && (XPress >= 240 && XPress <= 480) && (YPress <= 120 && YPress >= 0)){
          screenstatus = 3;
          Brain.Screen.clearScreen();
          Brain.Screen.render();
          WP();
        } else if ((screenstatus ==2) && (XPress >= 240 && XPress <= 480) && (YPress >= 120 && YPress <= 240)){
          screenstatus = 3;
          Brain.Screen.clearScreen();
          Brain.Screen.render();
          center();
        }
      }
    }
  }
  */
}

void autonomous(void){
  /*desiredValue = 15;
  rPID();
  desiredValue = 15;
  PID();
  turnRdesiredValue = 90;
  trightPID();
  desiredValue = 18;
  PID();
  trightPID();*/
  mogoDWant = 85;
  mogoDown();
  turnLdesiredValue = 50;
  tleftPID();
  desiredValue = 26.7;
  PID();
  vex::task::sleep(10);
  turnRdesiredValue = 56.5;
  trightPID();
  mogoDWant = 29.5;
  task Thread1 = task(mogoDown);
  desiredValue = 84;
  PID();
  Thread1.stop();
  mogoUWant = 60;
  mogoUp();
  rdesiredValue = 20;
  rPID();
  CON.setVelocity(60, vex::velocityUnits::pct);
  CON.spin(reverse);

  /*task myThread = task(clawDown);
  moveRev(43, 10);
  wait(500,msec);
  moveRev(3,3);
  clawDown();
  moveFwd(20,10);
  clawOpen(300);
  moveFwd(14,8);
  turnLeft(25);
  moveRev(48, 10);
  wait(500,msec);
  moveRev(3,3);
  clawDown();
  moveFwd(28,10);
  clawOpen(300);
  moveFwd(3,3);
  task::sleep(100);

  //leftovers from troubleshooting
  Brain.Screen.print(inertial1.rotation(degrees));
  Brain.Screen.print("finished");

*/
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
  BL.spin(fwd, 0.8*0.12*(Controller1.Axis3.value()+Controller1.Axis1.value()) , volt);
  FL.spin(fwd, 0.8*0.12*(Controller1.Axis3.value()+Controller1.Axis1.value()) , volt);
  BR.spin(fwd, 0.7*0.12*(Controller1.Axis3.value()-Controller1.Axis1.value()) , volt);
  FR.spin(fwd, 0.7*0.12*(Controller1.Axis3.value()-Controller1.Axis1.value()) , volt);
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.


    if(Controller1.ButtonR1.pressing()) {
    mogo.spin(vex::directionType::fwd, 80, vex::velocityUnits::pct);
    mogo2.spin(vex::directionType::fwd, 80, vex::velocityUnits::pct);
    }
    else if(Controller1.ButtonR2.pressing())
    {
    mogo.spin(vex::directionType::rev, 80, vex::velocityUnits::pct);
    mogo2.spin(vex::directionType::rev, 80, vex::velocityUnits::pct);
    }
    else {
      mogo.stop(vex::brakeType::hold);
      mogo2.stop(vex::brakeType::hold);

        }
      
    if(Controller1.ButtonRight.pressing()) {
    CON.spin(vex::directionType::fwd, 80, vex::velocityUnits::pct);
    }
    else if(Controller1.ButtonDown.pressing())
    {
    CON.stop(coast);
    }
    else {
        CON.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);


        }

    if(Controller1.ButtonL1.pressing()) {
    arm.spin(vex::directionType::fwd, 80, vex::velocityUnits::pct);
    }
    else if(Controller1.ButtonL2.pressing())
    {
    arm.spin(vex::directionType::rev, 80, vex::velocityUnits::pct);
    }
    else {
        arm.stop(vex::brakeType::hold);


        }

    if(Controller1.ButtonY.pressing()) {
      pushinP(true);
    }
    else if(Controller1.ButtonB.pressing())
    {
      pushinP(false);
    }                 
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  vexcodeInit();
  inertial1.calibrate();
  // waits for the Inertial Sensor to calibrate
  while (inertial1.isCalibrating()) {
    wait(100, msec);
  }

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
