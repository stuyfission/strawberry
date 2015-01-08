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
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "JoystickDriver.c"

// Copyright Stuy Fission 310
/**
 * Authored by Alvin Lin (alvin.lin@stuypulse.com)
 * http://omgimanerd.github.io
 * http://310fission.com
 * This file is a basic teleop program for a tankdrive.
 * It also contains toggle code which will be recycled next year.
 */

// Control constants.
const int controlModeSpeed = 20;
const int joystickThreshold = 25;

task main() {
	int x1, y1, x2, y2;
	bool controlDriveMode = false;
	int lastControlDriveMode = 0;
	bool acquirerActive = false;
	int lastAcquirerActive = 0;
	bool teleopRunning = true;

	while (teleopRunning) {
		// Update the values of the variables storing the joystick positions.
		getJoystickSettings(joystick);
		x1 = joystick.joy1_x1;
	 	y1 = joystick.joy1_y1;
	 	x2 = joystick.joy1_x2;
	 	y2 = joystick.joy1_y2;

	 	// If the joysticks are less than 25 in position, then they
	 	// are set to zero as a threshold of movement.
    if (abs(y1) < joystickThreshold) {
      y1 = 0;
    }
    if (abs(y2) < joystickThreshold) {
      y2 = 0;
    }

		// Joystick button 1 toggles the drive mode on and off.
		if (joy1Btn(1) && lastControlDriveMode == 0) {
			controlDriveMode = !controlDriveMode;
		}
		lastControlDriveMode = joy1Btn(1);

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

	  if (controlDriveMode) {
			motor[driveL] = controlModeSpeed * (y1 / abs(y1));
			motor[driveR] = controlModeSpeed * (y2 / abs(y2));
		} else {
		 	motor[driveL] = y1;
		 	motor[driveR] = y2;
		}

	 	// Joystick button 4 will release the balls from the lifted box.
	 	if (joy1Btn(4)) {
	 		servo[liftBox] = 150;
	 	} else {
	 		servo[liftBox] = 0;
	 	}

    // Joystick buttons 5 and 7 raise and lower the lift mechanism.
    if (joy1Btn(5)) {
    	motor[liftL] = 100;
    	motor[liftR] = 100;
    } else if (joy1Btn(7)) {
    	motor[liftL] = -100;
    	motor[liftR] = -100;
    } else {
  		motor[liftL] = 0;
  		motor[liftR] = 0;
  	}

    // Joystick buttons 6 and 8 rotate the servo that clamps the
    // rolling goals on and off.
	 	if (joy1Btn(6)) {
	 		servo[goalClamp] = 0;
	 	}
	 	if (joy1Btn(8)) {
	 		servo[goalClamp] = 200;
	 	}

		// Shuts off the teleop program if joystick buttons 9 and 10
		// are pressed simultaneously.
		teleopRunning = !(joy1Btn(9) && joy1Btn(10));

		// Outputs the joystick states to the screen.
	 	eraseDisplay();
		nxtDisplayString(2, "X2: %i", x2);
		nxtDisplayString(3, "Y2: %i", y2);
		nxtDisplayString(4, "X1: %i", x1);
		nxtDisplayString(5, "Y1: %i", y1);
	}
	StopAllTasks();
}
