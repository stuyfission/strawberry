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
 * Autonomous code for f(x) bot. Split into three autonomous programs that can
 * be selected at will.
 */

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
 * @param inches The number of inches to move forward.
 * @return The number of ticks to move in order to move forward
 *   the specified number of inches.
 */
int toTicks (float inches) {
  return (int) (inches / (4 * PI)) * TICKS_PER_ROTATION;
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

/**
 * Sets the drivetrain to move straight at a certain speed for a certain
 * distance.
 * @param speed The speed to the the drivetrain at.
 * @param encoderTicks The distance in ticks to run the drivetrain for.
 */
void driveStraight(int speed, int encoderTicks) {
  clearEncoders();
  driveMotors(speed, speed, encoderTicks);
  clearEncoders();
}

/**
 * Raises or lowers the lift for a certain distance.
 * @param power The speed and direction to power the lift at.
 * @param encoderTicks The distance to run the lift for.
 */
void activateLift(int power, int encoderTicks) {
  clearEncoders();
  while (abs(nMotorEncoder[lift1]) < abs(encoderTicks) &&
         abs(nMotorEncoder[lift2]) < abs(encoderTicks)) {
    motor[lift1] = power;
    motor[lift2] = power;
  }
  stopMotors();
  clearEncoders();
}

/**
 * Autonomous code that drives down the ramp only.
 */
void auton0() {
  clearEncoders();
  initializeServos();

  driveStraight(-30, 750);
  driveStraight(-100, 5500);
  wait1Msec(1000);
}

/**
 * Autonomous code that scores in the medium goal.
 */
void auton1() {
  clearEncoders();
  initializeServos();

  driveStraight(-40, 750);
  driveStraight(-100, 5000);
  driveMotors(-100, 100, 3050);
  driveStraight(100, 3500);

  motor[acquirer] = -50;
  wait1Msec(2000);
  motor[acquirer] = 0;

  activateLift(100, 2500);

  servo[liftBox] = 150;
  servo[goalClamp] = 200;
  wait1Msec(1000);
  servo[liftBox] = 0;
  wait1Msec(1000);

  activateLift(-30, 2500);

	driveMotors(100, 0, 4300);
	driveStraight(-100, 5000);
	wait1Msec(1000);
}

/**
 * Defensive autonomous code that blocks the opposing center goal.
 */
void auton2() {
  clearEncoders();
  initializeServos();

  driveStraight(-30, 750);
  driveStraight(-100, 5000);
}

task main() {
  //waitForStart();
  StartTask(outputEncoderValues);
  auton1();
}
