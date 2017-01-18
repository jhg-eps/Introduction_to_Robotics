#ifndef __LABVOLT_H__
#define __LABVOLT_H__

#ifdef __cplusplus	// If this is a C++ compiler, use C linkage
extern "C" {
#endif

void gripperClose(void);			// Close the gripper
void gripperOpen(void);				// Open the gripper
									// Move motors to relative positions
void moverel(int base, int shoulder, int elbow, int wrist1, int wrist2);
void nest(void);					// Returns to home
void zero(void);					// Set home position
void hardHome(void);				// Homes robot, sets home position
void init(void);					// Connects to robot and hard homes

void shutdown(void);

#ifdef __cplusplus					// If this is a C++ compiler, use C linkage
}
#endif

#endif	//__LABVOLT_H__
