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

// Copyright Stuy Fission 310
/**
 * @author Kenneth Li (kenneth.li@stuypulse.com)
 * Contributor: Alvin Lin (alvin.lin@stuypulse.com)
 * This code forms a utility program that allows us to actuate
 * the lift.
 * The left NXT button will lower the lift while the right one
 * will raise it. The center NXT button will apply tension to the
 * pulley strings.
 */

task main() {
  while (true) {
    eraseDisplay();
    nxtDisplayString(3, "Button: %i", nNxtButtonPressed);
    if (nNxtButtonPressed == 1) {
    	nxtDisplayString(4, "Raising lift");
      if (nMotorEncoder[lift1] > nMotorEncoder[lift2]) {
        motor[lift1] = 75;
        motor[lift2] = 100;
      } else {
        motor[lift1] = 100;
        motor[lift2] = 75;
      }
    } else if (nNxtButtonPressed == 2) {
    	nxtDisplayString(4, "Lowering lift");
      if (nMotorEncoder[lift1] > nMotorEncoder[lift2]) {
        motor[lift1] = -100;
        motor[lift2] = -75;
      } else {
        motor[lift1] = -75;
        motor[lift2] = -100;
      }
    } else if (nNxtButtonPressed == 3) {
    	nxtDisplayString(4, "Tensioning lift");
    	motor[lift1] = 10;
    	motor[lift2] = 10;
  	} else {
    	nxtDisplayString(4, "Left to lower");
    	nxtDisplayString(5, "Right to raise");
    	nxtDisplayString(6, "Middle to tension");
      motor[lift1] = 0;
      motor[lift2] = 0;
    }
    wait1Msec(10);
  }
}
