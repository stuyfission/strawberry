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

#include "JoystickDriver.c"

// Copyright Stuy Fission 310
/**
 */

<<<<<<< HEAD
task outputEncoderValues() {
=======
// Control constants.
const int controlModeSpeed = 20;
const int joystickThreshold = 25;

void driveMotors (int encoderTicks, int speed) {
	encoderClear();
	while ((nMotorEncoder[left] < encoderTicks) && (nMotorEncoder[right] < encoderTicks)) {
		motor[left] = speed;
		motor[right] = speed;
	}
	motor[left] = 0;
	motor[right] = 0;
	encoderClear();
}

task main() {
	// x1, y1, x2, and y2 store the joystick values for the driver.
  int x1, y1, x2, y2;
  // last* variables are for toggle states
  bool controlDriveMode = false;
  int lastControlDriveMode = 0;
  bool acquirerActive = false;
  int lastAcquirerActive = 0;

>>>>>>> ac20b839961d166b32bd98b845e272532e972ead
  while (true) {
    eraseDisplay();
    nxtDisplayString(2, "FLEnc: %i", nMotorEncoder[driveFL]);
    nxtDisplayString(3, "BLEnc: %i", nMotorEncoder[driveBL]);
    nxtDisplayString(4, "FREnc: %i", nMotorEncoder[driveFR]);
    nxtDisplayString(5, "BREnc: %i", nMotorEncoder[driveBR]);
    wait1Msec(10);
  }
}

void clearEncoders() {
	nMotorEncoder[driveFL] = 0;
	nMotorEncoder[driveBL] = 0;
	nMotorEncoder[driveFR] = 0;
	nMotorEncoder[driveBR] = 0;
}

task main() {
	StartTask(outputEncoderValues);
	clearEncoders();
	while (abs(nMotorEncoder[driveFL]) < 10000) {
		motor[driveFL] = -50;
		motor[driveBL] = -50;
		motor[driveFR] = 50;
		motor[driveBR] = 50;
	}
	motor[driveFL] = 0;
	motor[driveBL] = 0;
	motor[driveFR] = 0;
	motor[driveBR] = 0;
	wait1Msec(10000);
}
