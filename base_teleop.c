#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTMotor)
#pragma config(Hubs,  S2, HTServo,  none,     none,     none)
#pragma config(Sensor, S3,     sonar,          sensorSONAR)
#pragma config(Sensor, S4,     gyro,           sensorI2CHiTechnicGyro)
#pragma config(Motor,  mtr_S1_C1_1,     driveFL,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     driveBL,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     lift1,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     lift2,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     acquirer,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     blankMotor,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_1,     driveFR,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_2,     driveBR,       tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S2_C1_1,    goalClamp,            tServoStandard)
#pragma config(Servo,  srvo_S2_C1_2,    liftBox,              tServoStandard)
#pragma config(Servo,  srvo_S2_C1_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_6,    servo6,               tServoNone)

// Copyright Stuy Fission 310
/**
 * Authored by Alvin Lin (alvin.lin@stuypulse.com)
 * http://omgimanerd.github.io
 * http://310fission.com
 * This file is the single person joystick teleop program for f(x) bot. All
 * parts of the robot can be controlled with one joystick in this program.
 */

#include "JoystickDriver.c"
#include "fx_header.h"

// Control constants.
const int CONTROL_MODE_SPEED = 25;
const int JOYSTICK_THRESHOLD = 25;

task main() {
  // x1, y1, x2, and y2 store the joystick values for the driver.
  int x1, y1, x2, y2;
  // liftDownLimiter stores whether or not the limiter is active on the lift
  // mechanism.
  bool liftDownLimiter = true;

  // last* variables are for toggle states
  bool controlDriveMode = false;
  int lastControlDriveMode = 0;
  bool acquirerActive = false;
  int lastAcquirerActive = 0;

  nMotorEncoder[lift1] = 0;
  nMotorEncoder[lift2] = 0;

  waitForStart();
  while (true) {
    // Update the values of the variables storing the joystick positions.
    getJoystickSettings(joystick);
    x1 = joystick.joy1_x1;
    y1 = joystick.joy1_y1;
    x2 = joystick.joy1_x2;
    y2 = joystick.joy1_y2;

    // If the joysticks are less than 25 in position, then they
    // are set to zero as a threshold.
    if (abs(y1) < JOYSTICK_THRESHOLD) {
      y1 = 0;
    }
    if (abs(y2) < JOYSTICK_THRESHOLD) {
      y2 = 0;
    }

    // Joystick button 1 toggles the drive mode between slow and fast.
    if (joy1Btn(1) && lastControlDriveMode == 0) {
      controlDriveMode = !controlDriveMode;
    }
    lastControlDriveMode = joy1Btn(1);

    // Joystick values are assigned to the motors.
    if (controlDriveMode) {
      motor[driveFL] = CONTROL_MODE_SPEED * sgn(y1);
      motor[driveBL] = CONTROL_MODE_SPEED * sgn(y1);
      motor[driveFR] = CONTROL_MODE_SPEED * -sgn(y2);
      motor[driveBR] = CONTROL_MODE_SPEED * -sgn(y2);
    } else {
      motor[driveFL] = y1;
      motor[driveBL] = y1;
      motor[driveFR] = -y2;
	    motor[driveBR] = -y2;
    }

    // Outputs Joystick 1 and drive state mode to the screen.
    eraseDisplay();
    nxtDisplayString(1, "X2: %i", x2);
    nxtDisplayString(2, "Y2: %i", y2);
    nxtDisplayString(3, "X1: %i", x1);
    nxtDisplayString(4, "Y1: %i", y1);
    nxtDisplayString(5, "lift1 %i", nMotorEncoder[lift1]);
    if (controlDriveMode) {
    	nxtDisplayString(6, "controlled mode");
    } else {
    	nxtDisplayString(6, "fast mode");
    }

    // Joystick button 2 toggles the acquirer on and off.
    if (joy1Btn(2) && lastAcquirerActive == 0) {
      acquirerActive = !acquirerActive;
      if (acquirerActive) {
        motor[acquirer] = -50;
      } else {
        motor[acquirer] = 0;
      }
    }
    lastAcquirerActive = joy1Btn(2);

    // Joystick button 3 will release the balls from the lifted box.
    if (joy1Btn(3)) {
      servo[liftBox] = 150;
    } else {
      servo[liftBox] = 0;
    }

    // Joystick button 4 will allow the manual override of the lift
    // lowering limiter.
    if (joy1Btn(4)) {
      liftDownLimiter = false;
    }

    // Joystick buttons 5 and 7 clamp and release the rolling goal.
    if (joy1Btn(6)) {
      servo[goalClamp] = 0;
    }
    if (joy1Btn(8)) {
      servo[goalClamp] = 200;
    }

    // Joystick buttons 7 and 8 lower the lift mechanism.
    // Uses a scaling deviation formula to keep them in sync.
    if (joy1Btn(5)) {
    	int deviation = normalizeDeviation(
	    		nMotorEncoder[lift1] - nMotorEncoder[lift2]);
	    motor[lift1] = normalizeSpeed(100 - deviation);
	    motor[lift2] = normalizeSpeed(100 + deviation);
    } else if (joy1Btn(7)) {
      if (liftDownLimiter &&
          nMotorEncoder[lift1] <= 0 &&
          nMotorEncoder[lift2] <= 0) {
        motor[lift1] = 0;
        motor[lift2] = 0;
      } else {
	    	int deviation = normalizeDeviation(
		    		nMotorEncoder[lift1] - nMotorEncoder[lift2]);
		    motor[lift1] = normalizeSpeed(-100 - deviation);
		    motor[lift2] = normalizeSpeed(-100 + deviation);
      }
    } else {
      motor[lift1] = 0;
      motor[lift2] = 0;
    }
  }
}
