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

#define TICKS_PER_ROTATION 1120

// Copyright Stuy Fission 310
/**
 * @author Alvin Lin (alvin.lin@stuypulse.com)
 * Auton for f(x) bot.
 */

task outputEncoderValues() {
  while (true) {
    eraseDisplay();
    nxtDisplayString(2, "FL: %i", nMotorEncoder[driveFL]);
    nxtDisplayString(3, "BL: %i", nMotorEncoder[driveBL]);
    nxtDisplayString(4, "FR: %i", nMotorEncoder[driveFR]);
    nxtDisplayString(5, "BR: %i", nMotorEncoder[driveBR]);
    wait1Msec(10);
  }
}

/**
 * @param inches The number of inches to move forward
 */
int toTicks (float inches) {
	return (int) (inches / (4 * PI)) * TICKS_PER_ROTATION;
}

void clearEncoders() {
  nMotorEncoder[driveFL] = 0;
  nMotorEncoder[driveBL] = 0;
  nMotorEncoder[driveFR] = 0;
  nMotorEncoder[driveBR] = 0;
  nMotorEncoder[lift1] = 0;
  nMotorEncoder[lift2] = 0;
}

int averageMotors(tMotor frontMotor, tMotor backMotor) {
  return (nMotorEncoder[frontMotor] +	nMotorEncoder[backMotor]) / 2;
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

void stopMotors() {
  driveMotors(0, 0);
  motor[lift1] = 0;
  motor[lift2] = 0;
}

/**
 * Sets the drivetrain to run at a certain speed for a certain distance.
 * @param leftSpeed The speed to run the left side of the drivetrain.
 * @param rightSpeed The speed to run the right side of the drivetrain.
 * @param encoderTicks The distance in ticks to run the drivetrain for.
 */
void driveMotors(int leftSpeed, int rightSpeed, int encoderTicks) {
  clearEncoders();
  while (averageMotors(driveFL, driveBL) < abs(encoderTicks) &&
         averageMotors(driveFR, driveBR) < abs(encoderTicks)) {
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

void activateLift(int power, int encoderTicks) {
	clearEncoders();
	while (nMotorEncoder[lift1] < abs(encoderTicks) &&
				 nMotorEncoder[lift2] < abs(encoderTicks)) {
		motor[lift1] = power;
    motor[lift2] = power;
  }
  stopMotors();
  clearEncoders();
}

// drives down ramp
void auton0() {
	clearEncoders();
  driveStraight(750, -50);
  driveStraight(5500, -100);
  wait1Msec(1000);
}

// drives down ramp, scores in medium goal
void auton1() {
	servo[goalClamp] = 0;//initializes servos
	servo[liftBox] = 225;	//intiialize servos
  clearEncoders();
  driveStraight(750, -50);
  driveStraight(5500, -100);
  driveMotors(-100, 100, 2900);
  driveStraight(1500, 100);
  motor[acquirer] = -50;
  wait1Msec(1500);
  motor[acquirer] = 0;
  activateLift(100, 500);
  servo[liftBox] = 90;
  wait1Msec(1000);
}

// drives down center, blocks opposing center goal
void auton2() {
  clearEncoders();
  driveStraight(750, -50);
  driveStraight(5500, -100);
  driveMotors(-100, 100, 1500);
  driveStraight(3000, -100);
  wait1Msec(1000);
}

// drives straight as well?? blocks rolling
void auton3() {
  driveStraight(500, 50);
  driveStraight(14400, 100);
  driveStraight(500, 50);
  wait1Msec(120000);
}

task main() {
	//waitForStart();
  StartTask(outputEncoderValues);

	servo[liftBox] = 0;
  servo[liftBox] = 150;
  wait1Msec(1000);
  servo[liftBox] = 0;
  wait1Msec(1000);

  //auton1();
}
