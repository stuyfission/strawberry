#pragma config(Sensor, S1,     HTSPB,                sensorI2CCustom9V)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*   HiTechnic Experimenter's Kit for the SuperPro

Experiment - 6 Reaction Time Measurement

This program measures the time taken to press a button switch after an LED is turned on.

*/

#include "drivers/HTSPB-driver.h"

task main() {
  // The data to be written: 0x30 = 00110000 binary,
  // makes B4,B5 digital ports outputs.
  HTSPBsetupIO(HTSPB, 0x03);

  while(true) {

    // Turn off the LED
    HTSPBwriteIO(HTSPB, 0x00);

    eraseDisplay();
    nxtDisplayTextLine(2, "running");

    // Wait a random time between 3 and 8 seconds.
    wait1Msec(random(5000) + 3000);

    // Switch on the LED and reset the timer
    HTSPBwriteIO(HTSPB, 0x01);
    time1[T1] = 0;

    // Wait for user to press the stop button
    while (HTSPBreadIO(HTSPB, 0x30) != 0x10) {
      wait1Msec(5);
    }

    eraseDisplay();
    nxtDisplayTextLine(2, "Time: %d", time1[T1]);

    // Wait for user to reset
    while (HTSPBreadIO(HTSPB, 0x30) != 0x30) {
      wait1Msec(5);
    }
  }
}
