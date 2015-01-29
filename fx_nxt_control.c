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
 * @author Alvin Lin (alvin.lin@stuypulse.com)
 * Creates a menu in which robot actions can be controlled by the
 * NXT buttons for the purpose of debugging.
 */

#include "fx_header.h"

/**
 * Sets all the motor encoders back to zero.
 */
void clearEncoders() {
  nMotorEncoder[driveFL] = 0;
  nMotorEncoder[driveBL] = 0;
  nMotorEncoder[driveFR] = 0;
  nMotorEncoder[driveBR] = 0;
  nMotorEncoder[lift1] = 0;
  nMotorEncoder[lift2] = 0;
}

/**
 * Sets the drivetrain to run at a certain speed without stopping.
 * Normalizes the speed passed in.
 * A positive speed moves forward and a negative speed moves backward.
 * @param leftSpeed The speed to run the left side of the drivetrain.
 * @param rightSpeed The speed to run the right side of the drivetrain.
 */
void driveMotors(int leftSpeed, int rightSpeed) {
  motor[driveFL] = normalizeSpeed(-leftSpeed);
  motor[driveBL] = normalizeSpeed(-leftSpeed);
  motor[driveFR] = normalizeSpeed(rightSpeed);
  motor[driveBR] = normalizeSpeed(rightSpeed);
}

const int DRIVE_STRAIGHT_FD_MODE = 0;
const int DRIVE_STRAIGHT_BK_MODE = 1;
const int DRIVE_L_MODE = 2;
const int DRIVE_R_MODE = 3;
const int LIFT_UP_MODE = 4;
const int LIFT_DOWN_MODE = 5;
const int ACQUIRER_MODE = 6;
const int GOAL_CLAMP_MODE = 7;
const int LIFT_BOX_MODE = 8;
const int SENSOR_OUTPUT = 9;
const int NUM_MODES = 10;

task main() {
  int mode = 0;
  bool leftPressed = false;
  bool rightPressed = false;
  bool centerPressed = false;
	string centerText1;
	string centerText2;

  clearEncoders();

  while (true) {
    // Switching actuation modes.
    // Creates a menu to choose from.
    if (nNxtButtonPressed == 2 && !leftPressed) {
      mode--;
      if (mode < 0) {
        mode += NUM_MODES;
      }
    }
    leftPressed = nNxtButtonPressed == 2;

    if (nNxtButtonPressed == 1 && !rightPressed) {
      mode = (mode + 1) % NUM_MODES;
    }
    rightPressed = nNxtButtonPressed == 1;

    centerPressed = nNxtButtonPressed == 3;

    eraseDisplay();

    // Sets center text.
    centerText1 = "Press center";
    centerText2 = "button to actuate";

    // Responses for each mode.
    switch (mode) {
	    case DRIVE_STRAIGHT_FD_MODE:
	      nxtDisplayString(1, "%i: Straight FD", mode);
	      if (centerPressed) {
			    int deviation = normalizeDeviation(
			    		averageMotors(driveFL, driveBL) - averageMotors(driveFR, driveBR));
			   	int leftSpeed = 100 - deviation;
			    int rightSpeed = 100 + deviation;
			    driveMotors(leftSpeed, rightSpeed);
	      } else {
	        driveMotors(0, 0);
	      }
	      break;

	    case DRIVE_STRAIGHT_BK_MODE:
	      nxtDisplayString(1, "%i: Straight BK", mode);
	      if (centerPressed) {
			    int deviation = normalizeDeviation(
			    		averageMotors(driveFL, driveBL) - averageMotors(driveFR, driveBR));
			   	int leftSpeed = -100 + deviation;
			    int rightSpeed = -100 - deviation;
			    driveMotors(leftSpeed, rightSpeed);
	      } else {
	        driveMotors(0, 0);
	      }

	      break;

	    case DRIVE_L_MODE:
	      nxtDisplayString(1, "%i: Left Drive", mode);
	      if (centerPressed) {
	        motor[driveFL] = 100;
	        motor[driveBL] = 100;
	      } else {
	        motor[driveFL] = 0;
	        motor[driveBL] = 0;
	      }
	      break;

	    case DRIVE_R_MODE:
	      nxtDisplayString(1, "%i: Right Drive", mode);
	      if (centerPressed) {
	        motor[driveFR] = -100;
	        motor[driveBR] = -100;
	      } else {
	        motor[driveFR] = 0;
	        motor[driveBR] = 0;
	      }
	      break;

	    case LIFT_UP_MODE:
	      nxtDisplayString(1, "%i: Lift Up", mode);
	      if (centerPressed) {
			    int deviation = normalizeDeviation(
			    		nMotorEncoder[lift1] - nMotorEncoder[lift2]);
			    motor[lift1] = normalizeSpeed(100 - deviation);
			    motor[lift2] = normalizeSpeed(100 + deviation);
	      } else {
	        motor[lift1] = 0;
	        motor[lift2] = 0;
	      }
	      break;

	    case LIFT_DOWN_MODE:
	      nxtDisplayString(1, "%i: Lift Down", mode);
	      if (centerPressed) {
			    int deviation = normalizeDeviation(
			    		nMotorEncoder[lift1] - nMotorEncoder[lift2]);
			    motor[lift1] = normalizeSpeed(-100 + deviation);
			    motor[lift2] = normalizeSpeed(-100 - deviation);
	      } else {
	        motor[lift1] = 0;
	        motor[lift2] = 0;
	      }
	      break;

	    case ACQUIRER_MODE:
	      nxtDisplayString(1, "%i: Acquirer", mode);
	      if (centerPressed) {
	        motor[acquirer] = -50;
	      } else {
	        motor[acquirer] = 0;
	      }
	      break;

	    case GOAL_CLAMP_MODE:
	      nxtDisplayString(1, "%i: Goal Clamp", mode);
	      if (centerPressed) {
	        servo[goalClamp] = 0;
	      } else {
	        servo[goalClamp] = 200;
	      }
	      break;

	    case LIFT_BOX_MODE:
	      nxtDisplayString(1, "%i: Lift Box", mode);
	      if (centerPressed) {
	        servo[liftBox] = 150;
	      } else {
	        servo[liftBox] = 0;
	      }
	      break;

    	case SENSOR_OUTPUT:
	      nxtDisplayString(1, "%i: Sensor Output", mode);
    		float sonarValue = SensorValue[sonar];
    		float gyroValue = SensorValue[gyro];
    		StringFormat(centerText1, "Sonar: %f", sonarValue);
    		StringFormat(centerText2, "Gyro: %f", gyroValue);
    		break;
    }

    // Outputs center text.
    nxtDisplayString(2, centerText1);
    nxtDisplayString(3, centerText2);

    // 10 millisecond buffer.
    wait1Msec(10);
  }
}
