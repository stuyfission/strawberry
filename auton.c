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
 * Autonomous code for f(x) bot. Split into three autonomous programs that can
 * be selected at will. The tasks and helper functions are sorted in order of
 * complexity and interdependence.
 *
 * The defined constants are specific for f(x) bot. For 4 inch wheels on AndyMark
 * NeveRest motors, 1120 ticks represents one rotation.
 */

#include "fx_header.h"

#define GYRO_NORMAL 596

/**
 * Task that runs synchronously to the main task.
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
 * Sets the servos to their initial positions.
 */
void initializeServos() {
  servo[goalClamp] = 0; // initializes servos
  servo[liftBox] = 0;	// initializes servos
  wait1Msec(1000);
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

/**
 * Stops all the motors on the robot.
 */
void stopMotors() {
  motor[driveFL] = 0;
  motor[driveBL] = 0;
  motor[driveFR] = 0;
  motor[driveBR] = 0;
  motor[lift1] = 0;
  motor[lift2] = 0;
}

/**
 * Sets the drivetrain to run at a certain speed for a certain distance
 * with no adjustment from the encoders or gyroscope.
 * Used for turning n shit.
 * A positive speed moves forward and a negative speed moves backward.
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

/**
 * Sets the drivetrain to move straight at a certain speed for a certain
 * distance without using the gyroscope.
 * Setting too low of a power causes weirdness.
 * A positive speed moves forward and a negative speed moves backward.
 * @param speed The speed to the the drivetrain at.
 * @param encoderTicks The distance in ticks to run the drivetrain for.
 */
void driveStraightEncoders(int speed, int encoderTicks) {
  clearEncoders();
  while (averageMotors(driveFL, driveBL) < abs(encoderTicks) &&
         averageMotors(driveFR, driveBR) < abs(encoderTicks)) {
    int deviation = normalizeDeviation(
                                       averageMotors(driveFL, driveBL) - averageMotors(driveFR, driveBR));
    int leftSpeed = speed - deviation;
    int rightSpeed = speed + deviation;
    driveMotors(leftSpeed, rightSpeed);
  }
  stopMotors();
  clearEncoders();
}

/**
 * Sets the drivetrain to move straight at a certain speed for a certain
 * distance using the gyroscope. The gyroscope reads > 600 when turning
 * right, and < 600 when turning left.
 * Setting too low of a power causes weirdness.
 * A positive speed moves forward and a negative speed moves backward.
 * @param speed The speed to the the drivetrain at.
 * @param encoderTicks The distance in ticks to run the drivetrain for.
 */
void driveStraightGyro(int speed, int encoderTicks) {
  clearEncoders();
  while (averageMotors(driveFL, driveBL) < abs(encoderTicks) &&
         averageMotors(driveFR, driveBR) < abs(encoderTicks)) {
    int deviation = 3 * normalizeDeviation(
                                       SensorValue[gyro] - GYRO_NORMAL);
    int leftSpeed = speed - (deviation * sgn(speed));
    int rightSpeed = speed + (deviation * sgn(speed));
    driveMotors(leftSpeed, rightSpeed);
  }
  stopMotors();
  clearEncoders();
}

/**
 * Raises or lowers the lift for a certain distance.
 * A positive power raises the lift and a negative power lowers it.
 * @param speed The speed and direction to power the lift at.
 * @param encoderTicks The distance to run the lift for.
 */
void activateLift(int speed, int encoderTicks) {
  clearEncoders();
  while (abs(nMotorEncoder[lift1]) < abs(encoderTicks) &&
         abs(nMotorEncoder[lift2]) < abs(encoderTicks)) {
    int deviation = normalizeDeviation(
                                       nMotorEncoder[lift1] - nMotorEncoder[lift2]);
    motor[lift1] = normalizeSpeed(speed - deviation);
    motor[lift2] = normalizeSpeed(speed + deviation);
  }
  stopMotors();
  clearEncoders();
}

void locateGoal() {
  while (SensorValue[sonar] > 250) {
    driveMotors(-25, 25);
  }
  stopMotors();
  wait1Msec(1000);
  driveMotors(20, -20, 50);
  stopMotors();
}

void rampAcceleration() {
  for (int i = 0; i < 100; i += 5) {
    driveMotors(i, i);
    wait1Msec(400);
  }
}

/** ############################## AUTON CODE ############################## */

/**
 * Autonomous code that drives down the ramp only.
 */
void auton0() {
  clearEncoders();
  initializeServos();

  driveStraightGyro(-40, 5000);
  driveStraightEncoders(-100, 4500);
  wait1Msec(1000);
}

/**
 * Autonomous code that scores in the medium goal.
 */
void auton1() {
	clearEncoders();
  initializeServos();
	wait1Msec(1000);
  driveStraightGyro(-100, 6900);
  driveMotors(-100, 100, 1600);
  driveStraightGyro(100, 750);
  wait1Msec(1000);
  bFloatDuringInactiveMotorPWM = false;
  locateGoal();
  driveStraightGyro(100, 900);

  motor[acquirer] = -50;
  wait1Msec(2000);
  motor[acquirer] = 0;

  servo[goalClamp] = 200;
  driveMotors(0, -30, 3000);
  driveMotors(-30, 30, 400);
  driveMotors(30, 30, 600);
  wait1Msec(1000);
  activateLift(100, 2500);
  servo[liftBox] = 150;
  wait1Msec(1000);
  servo[liftBox] = 0;
  wait1Msec(1000);

  bFloatDuringInactiveMotorPWM = true;
  activateLift(-50, 2000);
  /*
    driveMotors(100, 0, 4300);
    driveStraight(-100, 5000);
    wait1Msec(1000);*/
}

/**
 * Defensive autonomous code that blocks the opposing center goal.
 */
void auton2() {
  clearEncoders();
  initializeServos();

  driveStraightGyro(-50, 2000);
  driveStraightEncoders(-100, 4300);
}

task main() {
  //waitForStart();
  StartTask(outputEncoderValues);
  bFloatDuringInactiveMotorPWM = true;
 	auton1();
  StopAllTasks();
}
