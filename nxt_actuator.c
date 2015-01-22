#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTMotor)
#pragma config(Hubs,  S2, HTServo,  none,     none,     none)
#pragma config(Sensor, S3,     HTSPB,          sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     driveFL,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     driveBL,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     lift1,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     lift2,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     acquirer,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     blankMotor,         tmotorTetrix, openLoop)
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
 * Creates a menu in which actions can be controlled by the
 * NXT buttons for the purpose of debugging.
 */

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
 * @param tMotor The name of the front motor on a side.
 * @param tMotor The name of the back motor on the same side.
 * @return The average of the encoder values of the two specified motors.
 */
int averageMotors(tMotor frontMotor, tMotor backMotor) {
  return abs((nMotorEncoder[frontMotor] +	nMotorEncoder[backMotor]) / 2);
}

/**
 * Sets the drivetrain to run at a certain speed without stopping.
 * @param leftSpeed The speed to run the left side of the drivetrain.
 * @param rightSpeed The speed to run the right side of the drivetrain.
 */
void driveMotors(int leftSpeed, int rightSpeed) {
  motor[driveFL] = -leftSpeed;
  motor[driveBL] = -leftSpeed;
  motor[driveFR] = rightSpeed;
  motor[driveBR] = rightSpeed;
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
const int NUM_MODES = 9;

task main() {
	int mode = 0;
	bool leftPressed = false;
	bool rightPressed = false;
	bool centerPressed = false;

	clearEncoders();

  while (true) {

  	// Switching actuation modes
  	if (nNxtButtonPressed == 2 && !leftPressed) {
  		mode = (mode - 1) % NUM_MODES;
  	}
  	leftPressed = nNxtButtonPressed == 2;

  	if (nNxtButtonPressed == 1 && !rightPressed) {
  		mode = (mode + 1) % NUM_MODES;
  	}
  	rightPressed = nNxtButtonPressed == 1;

  	centerPressed = nNxtButtonPressed == 0;

    eraseDisplay();
    nxtDisplayString(2, "Press center to actuate");

    switch (mode) {
    	case DRIVE_STRAIGHT_FD_MODE:
    		nxtDisplayString(1, "%i: Straight Forward", mode);
    		if (centerPressed) {
    			if (averageMotors(driveFL, driveBL) > averageMotors(driveFR, driveBR)) {
    				driveMotors(75, 100);
    			} else {
    				driveMotors(100, 75);
    			}
    		} else {
    			driveMotors(0, 0);
    		}
    		break;
    	case DRIVE_STRAIGHT_BK_MODE:
    		nxtDisplayString(1, "%i: Straight Backward", mode);
    		if (centerPressed) {
    			if (averageMotors(driveFL, driveBL) > averageMotors(driveFR, driveBR)) {
    				driveMotors(-75, -100);
    			} else {
    				driveMotors(-100, -75);
    			}
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
		      if (nMotorEncoder[lift1] > nMotorEncoder[lift2]) {
		        motor[lift1] = 75;
		        motor[lift2] = 100;
		      } else {
		        motor[lift1] = 100;
		        motor[lift2] = 75;
		      }
    		} else {
    			motor[lift1] = 0;
    			motor[lift2] = 0;
    		}
    		break;
    	case LIFT_DOWN_MODE:
    		nxtDisplayString(1, "%i: Lift Down", mode);
    		if (centerPressed) {
		      if (nMotorEncoder[lift1] > nMotorEncoder[lift2]) {
		        motor[lift1] = -100;
		        motor[lift2] = -75;
		      } else {
		        motor[lift1] = -75;
		        motor[lift2] = -100;
		      }
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
    			servo[goalClamp] = 150;
    		} else {
    			servo[goalClamp] = 0;
    		}
    		break;
    }
    wait1Msec(10);
  }
}
