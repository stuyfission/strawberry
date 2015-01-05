#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTMotor)
#pragma config(Hubs,  S2, HTServo,  none,     none,     none)
#pragma config(Sensor, S3,     HTSPB,          sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     driveFL,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     driveBL,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     lift1,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     lift2,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     acquirer,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     blank,         tmotorTetrix, openLoop)
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
 * Authored by Alvin Lin (alvin.lin@stuypulse.com)
 * http://omgimanerd.github.io
 * http://310fission.com
 * This file is a basic teleop program for a tankdrive.
 * It also contains toggle code which will be recycled next year.
 */

// Control constants.
const int controlModeSpeed = 20;
const int joystickThreshold = 25;

task main() {
  int x1, y1, x2, y2;
  // last* variables are for toggle states
  bool controlDriveMode = false;
  int lastControlDriveMode = 0;
  bool acquirerActive = false;
  int lastAcquirerActive = 0;

  while (true) {
    /* JOYSTICK 1 - DRIVETRAIN */
    // Update the values of the variables storing the joystick positions.
    getJoystickSettings(joystick);
    x1 = joystick.joy1_x1;
    y1 = joystick.joy1_y1;
    x2 = joystick.joy1_x2;
    y2 = joystick.joy1_y2;

    // If the joysticks are less than 25 in position, then they
    // are set to zero as a threshold of movement.
    if (abs(y1) < joystickThreshold) {
      y1 = 0;
    }
    if (abs(y2) < joystickThreshold) {
      y2 = 0;
    }

    // Joystick button 1 toggles the drive mode on and off.
    if (joy1Btn(1) && lastControlDriveMode == 0) {
      controlDriveMode = !controlDriveMode;
    }
    lastControlDriveMode = joy1Btn(1);

    if (controlDriveMode) {
      motor[driveFL] = controlModeSpeed * (y1 / abs(y1));
      motor[driveBL] = controlModeSpeed * (y1 / abs(y1));
      motor[driveFR] = controlModeSpeed * (y2 / abs(y2));
      motor[driveBR] = controlModeSpeed * (y2 / abs(y2));
    } else {
      motor[driveFL] = y1;
      motor[driveBL] = y1;
      motor[driveFR] = y2;
      motor[driveBR] = y2;
    }

    // Joystick 1 buttons 5 and 7 clamp the rolling goal.
    // Joystick 1 buttons 6 and 8 release the rolling goal clamp.
    if (joy1Btn(6) || joy1Btn(5)) {
      servo[goalClamp] = 0;
    }
    if (joy1Btn(7) || joy1Btn(8)) {
      servo[goalClamp] = 200;
    }

    // Outputs Joystick 1 states to the screen.
    eraseDisplay();
    nxtDisplayString(2, "X2: %i", x2);
    nxtDisplayString(3, "Y2: %i", y2);
    nxtDisplayString(4, "X1: %i", x1);
    nxtDisplayString(5, "Y1: %i", y1);

    /* JOYSTICK 2 - MECHANISMS */
    // Joystick 2 button 2 toggles the acquirer on and off.
    if (joy2Btn(2) && lastAcquirerActive == 0) {
      acquirerActive = !acquirerActive;
      if (acquirerActive) {
        motor[acquirer] = -50;
      } else {
        motor[acquirer] = 0;
      }
    }
    lastAcquirerActive = joy2Btn(1);

    // Joystick 2 button 3 will release the balls from the lifted box.
    if (joy2Btn(3)) {
      servo[liftBox] = 90;
    } else {
      servo[liftBox] = 225;
    }

    // Joystick 2 buttons 5 and 6 raise the lift mechanism.
    // Joystick 2 buttons 7 and 8 lower the lift mechanism.
    if (joy2Btn(5) || joy2Btn(6)) {
      motor[lift1] = 100;
      motor[lift2] = 100;
    } else if (joy2Btn(7) || joy2Btn(8)) {
      motor[lift1] = -100;
      motor[lift2] = -100;
    } else {
      motor[lift1] = 0;
      motor[lift2] = 0;
    }
  }
}
