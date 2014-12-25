#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     driveL,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     driveR,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     liftL,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     liftR,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     acquirer,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     blank,         tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C4_1,    goalClamp,            tServoNone)
#pragma config(Servo,  srvo_S1_C4_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_6,    servo6,               tServoNone)
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

    // Joystick buttons 6 and 8 rotate the servo that clamps the
    // rolling goals on and off.
	 	if (joy1Btn(6)) {
	 		servo[goalClamp] = 255;
	 	}
	 	if (joy1Btn(8)) {
	 		servo[goalClamp] = 0;
	 	}

	 	// Joystick button 2 toggles the acquirer on and off.
		if (joy1Btn(2) && lastAcquirerActive == 0) {
			acquirerActive = !acquirerActive;
			if (acquirerActive) {
				motor[acquirer] = -100;
			} else {
				motor[acquirer] = 0;
			}
		}
		lastAcquirerActive = joy1Btn(2);

		// Joystick button 1 toggles the drive mode on and off.
		if (joy1Btn(1) && lastControlDriveMode == 0) {
			controlDriveMode = !controlDriveMode;
		}
		lastControlDriveMode = joy1Btn(1);

	  if (controlDriveMode) {
			motor[driveL] = controlModeSpeed * -(y1 / abs(y1));
			motor[driveR] = controlModeSpeed * -(y2 / abs(y2));
		} else {
		 	motor[driveL] = -y1;
		 	motor[driveR] = -y2;
		}

		// Shuts off the teleop program.
		teleopRunning = !(joy1Btn(9) && joy1Btn(10));

		// Outputs the joystick states to the screen.
	 	eraseDisplay();
		nxtDisplayString(2, "X2: %i", x2);
		nxtDisplayString(3, "Y2: %i", y2);
		nxtDisplayString(4, "X1: %i", x1);
		nxtDisplayString(5, "Y1: %i", y1);
	}
}
