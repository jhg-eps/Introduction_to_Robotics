// 5150 Robot example code
// This example should initialize the robot, move the base 20 degrees clocwise
// close the gripper, then return to home

#include "labvolt.h"


int main(void) {
	init();
	
	zero();					// Remember the home position
	moverel(-356,0,0,0,0);	// Rotate base 20 degrees clockwise
	gripperClose();			// Open the gripper
	nest();					// Return arm to home position

	shutdown();

	return 0;
}

