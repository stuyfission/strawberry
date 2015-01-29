// Copyright Stuy Fission 310
/**
 * @author Alvin Lin (alvin.lin@stuypulse.com)
 * Header file with base functions needed by both auton and teleop.
 */

#define TICKS_PER_ROTATION 1120

/**
 * @param inches The number of inches to move forward.
 * @return The number of ticks to move in order to move forward
 *   the specified number of inches.
 */
int toTicks (float inches) {
  return (int) (inches / (4 * PI)) * TICKS_PER_ROTATION;
}

/**
 * Bounds a given number within a given set of bounds.
 * @param x The number to bound
 * @param upperBound The uppermost or highest number it can be
 * @param lowerBound The lowest number it can be
 */
int bound (int x, int upperBound, int lowerBound) {
	if (x > upperBound) {
		return upperBound;
	} else if (x < lowerBound) {
		return lowerBound;
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
	return bound(speed, 100, -100);
}

/**
 * Normalizes deviations for motor adjustments.
 * TODO: where the fuck is min() and max()
 * @param deviation The deviation to bound
 */
int normalizeDeviation (int deviation) {
	return bound(deviation, 40, -40);
}

/**
 * @param tMotor The name of the front motor on a side.
 * @param tMotor The name of the back motor on the same side.
 * @return The average of the encoder values of the two specified motors.
 */
int averageMotors(tMotor motor1, tMotor motor2) {
  return abs((nMotorEncoder[motor1] +	nMotorEncoder[motor2]) / 2);
}
