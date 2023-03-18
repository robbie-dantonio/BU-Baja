/* Quick test code to map Adafruit Automotive
 * Gauge stepper mottor to GPS speed so it can
 * be used as an analog-looking speedometer needle
 *
 * Motor has 600 steps per rotation
 *
 * In this code, wire pins 8, 9, 10, 11 to the H-bridge
 */

#include <Stepper.h>

#define STEPS_PER_REVOLUTION 600

/* instantiate Stepper class on pins 8, 9, 10, 11 */
Stepper speedo(STEPS_PER_REVOLUTION, 8, 9, 10, 11); 

int stepCount = 0;	// Number of steps the motor has taken

//speedo.setSpeed(500);	// Speed at which the motor rotates. This might be too fast, need to test!

void setup(){
	/* TODO:
	 * 	- GPS Setup
	 *	- Initialize stepper motor to position 0?
	 * 	- ^^ Maybe not necesary, need to test!
	 */

	// Set stepper motor speed to 60 RPM. Test different values to make sure this is ok!
	speedo.setSpeed(60);
}

void loop(){
	/* TODO:
	 *	- Get GPS speed, for now say it's int gpsSpeed
	 *  	- Find out what units the GPS uses
	 * 	- If necesary, convert to MPH
	 */

	int gpsSteps = map(gpsSpeed, 0, 15, 0, 600); // Calculate how many steps correspond to current GPS Speed. Assumes max speed is 15, but need to confirm!

		
	/* If GPS Speed is greater than motor position, step forward.
	 * Else if GPS speed is less than position, step backward.
	 * Else don't step!
	 */
	if (gpsSteps > stepCount) { 		// If gpsSteps is higher, then step forward
		speedo.step(gpsSteps - stepCount);
		stepCount = stepCount + gpsSteps;	
	}
	else if (gpsSteps < stepCount) { 	// If gpsSteps is lower than current position, step backward
		speedo.step(stepCount - gpsSteps);
		stepCount = stepCount - gpsSteps;
	}
	// Otherwise, don't change position!	
}
