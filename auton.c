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
 * 1120 ticks per 26
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

void driveMotors(int leftSpeed, int rightSpeed) {
	motor[driveFL] = -leftSpeed;
	motor[driveBL] = -leftSpeed;
	motor[driveFR] = rightSpeed;
	motor[driveBR] = rightSpeed;
}

void stopMotors() {
	driveMotors(0, 0);
}

int averageMotors(tMotor frontMotor, tMotor backMotor) {
	return (nMotorEncoder[frontMotor] +	nMotorEncoder[backMotor]) / 2;
}

void driveMotors(int leftSpeed, int rightSpeed, int encoderTicks) {
	clearEncoders();
	while (averageMotors(driveFL, driveBL) < encoderTicks &&
				 averageMotors(driveFR, driveBR) < encoderTicks) {
		driveMotors(leftSpeed, rightSpeed);
	}
	stopMotors();
	clearEncoders();
}

void driveStraight(int encoderTicks, int speed) {
	clearEncoders();
	driveMotors(speed, speed, encoderTicks);
	stopMotors();
	clearEncoders();
}

void rotate90Left() {
	clearEncoders();
	driveMotors(100, -100, 1000);
}

void rotate90Right() {
	clearEncoders();
	driveMotors(-100, 100, 1000);
	clearEncoders();
}

void activateLift(int power, int time) {
	motor[lift1] = power;
	motor[lift2] = power;
	wait1Msec(time);
	motor[lift1] = 0;
	motor[lift2] = 0;
}

// drives down ramp, clamps goal, drives back up
void auton1() {
	clearEncoders();
	driveStraight(250, 50);
	driveStraight(5500, 100);
	driveStraight(250, 50);
	clearEncoders();
	activateLift(100, 100);
	servo[liftBox] = 90;
	wait1Msec(1000);
	servo[liftBox] = 225;
	activateLift(-100, 100);
	servo[goalClamp] = 0;
	driveStraight(-250, 50);
	driveStraight(-5500, 100);
	driveStraight(-250, 50);
	clearEncoders();
	wait1Msec(120000);
}

// drives straight?? blocks center
void auton2() {
	driveStraight(500, 50);
	driveStraight(3600, 100);
	driveStraight(500, 50);
	wait1Msec(120000);
}

// drives straight as well?? blocks rolling
void auton3() {
	driveStraight(500, 50);
	driveStraight(14400, 100);
	driveStraight(500, 50);
	wait1Msec(120000);
}

task main() {
	StartTask(outputEncoderValues);
	auton1();
}
