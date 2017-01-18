#ifndef __LABVOLT_H__
#define __LABVOLT_H__

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
extern "C" {
#endif

void gripperClose(void);				//Close the gripper
void gripperOpen(void);					//Open the gripper
void moverel(int base, int shoulder, int elbow, int wrist1, int wrist2);		//move motors to relative positions
void nest(void);					//returns to home
void zero(void);					//set home position
void hardHome(void);					//homes robot, sets home position
void init(void);					//Connects to robot and hard homes

void shutdown(void);

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
}
#endif
	
#endif  //__ECE417_H__
