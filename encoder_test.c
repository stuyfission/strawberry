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

task outputEncoderValues() {
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

void driveMotors(int encoderTicks, int speed) {
	clearEncoders();
	while (abs(nMotorEncoder[driveFL]) < encoderTicks &&
				 abs(nMotorEncoder[driveFR]) < encoderTicks) {
		motor[driveFL] = -speed;
		motor[driveBL] = -speed;
		motor[driveFR] = speed;
		motor[driveBR] = speed;
	}
	motor[driveFL] = 0;
	motor[driveBL] = 0;
	motor[driveFR] = 0;
	motor[driveBR] = 0;
	clearEncoders();
}

void rotate180() {
	clearEncoders();
	while (abs(nMotorEncoder[driveFL]) < encoderTicks &&
				 abs(nMotorEncoder[driveFR]) < encoderTicks) {
		motor[driveFL] = speed;
		motor[driveBL] = speed;
		motor[driveFR] = speed;
		motor[driveBR] = speed;
	}
	motor[driveFL] = 0;
	motor[driveBL] = 0;
	motor[driveFR] = 0;
	motor[driveBR] = 0;
	clearEncoders();
}

task main() {
	StartTask(outputEncoderValues);
	driveMotors(5000, -100);
	wait1Msec(10000);
}
