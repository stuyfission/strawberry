// Copyright Stuy Fission 310
/**
 * @author Alvin Lin (alvin.lin@stuypulse.com)
 * Header file with base functions needed by both auton and teleop for
 * f(x) bot.
 */

#define TICKS_PER_ROTATION 1120
#define WHEEL_DIAMETER 4
#define MIN_MOTOR_SPEED -100
#define MAX_MOTOR_SPEED 100
#define MIN_DEVIATION -40
#define MAX_DEVIATION 40

/**
 * @param inches The number of inches to move forward.
 * @return The number of ticks to move in order to move forward
 *   the specified number of inches.
 */
int toTicks (float inches) {
  return (int) (inches / (WHEEL_DIAMETER * PI)) * TICKS_PER_ROTATION;
}

/**
 * Bounds a given number within a given set of bounds.
 * @param x The number to bound
 * @param lowerBound The lowest number it can be
 * @param upperBound The uppermost or highest number it can be
 */
int bound (int x, int lowerBound, int upperBound) {
	if (x <= lowerBound) {
		return lowerBound;
	} else if (x >= upperBound) {
		return upperBound;
	} else {
		return x;
	}
}

/**
 * Normalizes a speed to a motor speed.
 * TODO: where the fuck is min() and max()
 * @param speed The speed to bound
 */
int normalizeSpeed (int speed) {
	return bound(speed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
}

/**
 * Normalizes deviations for motor adjustments.
 * TODO: where the fuck is min() and max()
 * @param deviation The deviation to bound
 */
int normalizeDeviation (int deviation) {
	return bound(deviation, MIN_DEVIATION, MAX_DEVIATION);
}

/**
 * @param tMotor The name of the first motor to average.
 * @param tMotor The name of the second motor to average.
 * @return The average of the encoder values of the two specified motors.
 */
int averageMotors(tMotor motor1, tMotor motor2) {
  return abs((nMotorEncoder[motor1] +	nMotorEncoder[motor2]) / 2);
}
