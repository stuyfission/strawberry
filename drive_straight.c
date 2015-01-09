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

task main()
{
	while(true) {
		motor[driveFL] = -50;
		motor[driveBL] = -50;
		motor[driveFR] = 50;
		motor[driveBR] = 50;
		wait1Msec(1000);
		if (nMotorEncoder[driveFL] < nMotorEncoder[driveFR]) {
			motor[driveFL] = -50;
			motor[driveBL] = -50;
			motor[driveFR] = 0;
			motor[driveBR] = 0;
		} else if (nMotorEncoder[driveFL] > nMotorEncoder[driveFR]) {
			motor[driveFL] = 0;
			motor[driveBL] = 0;
			motor[driveFR] = 50;
			motor[driveBR] = 50;
		} else {
			motor[driveFL] = 0;
			motor[driveBL] = 0;
			motor[driveFR] = 0;
			motor[driveBR] = 0;
		}
	}
}
