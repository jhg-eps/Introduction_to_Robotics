//Jason Monk and Tyler Lalime
//Using 5150USB.h written by Andy Sheaff

#define GPCLOSE 3000

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <dirent.h>
#include <string.h>

#include "5150USB.h"
#include "labvolt.h"

void init(void)
{
	int ret;
	char i;

	ret=DLLGetDevicePortNumber();
	if (ret<0) { printf("DLLGetDevicePortNumber failed %d\n",ret); }

	ret=DLLOpenDevice();
	if (ret==DLL_ERROR_NO_DEVICE) {	printf("DLLOpenDevice failed %d\n",ret); }
	if (ret!=DLL_SUCCESS) { exit(1); }

	ret=DLLIsDeviceOpen();
	if (ret!=DLL_SUCCESS) {	printf("DLLIsDeviceOpen failed %d\n",ret); }

	ret=DLLResetBoard();
	if (ret!=DLL_SUCCESS) {	printf("DLLResetBoard failed %d\n",ret); }

	ret=DLLHardHome();
	if (ret!=DLL_SUCCESS) {	printf("DLLHardHome failed %d\n",ret); }

	ret=DLLEnableMotors();
	if (ret!=DLL_SUCCESS) {	printf("DLLEnableMotors failed %d\n",ret); }

	for (i=0; i<0x07; i++)
	{
		DLLSetFrequency(i,500);
	}
	zero();
}


void zero(void)					//set home position
{
	char i;
	for(i=0; i<0x07; i++)
	{
		DLLSetPosition(i,0);
	}
	return;
}

void moverel (int base, int shoulder, int elbow, int wrist1, int wrist2)
{
	signed int offsets[7]={0,wrist1,wrist2,elbow,shoulder,base,0};
	signed int currentPos[7];
	unsigned char isMoving;
	int i,des;
	
	for (i=0; i<=0x06; i++)
	{
		DLLGetPosition(i,&currentPos[i]);
	}
	for (i=0; i<=0x06; i++)
	{
		des = ((offsets[i])+(currentPos[i]));
		DLLSetDesiredPosition(i,des);
	}

	DLLRunMotors();

	do {
		DLLMotorRunStatus(&isMoving);
	} while (isMoving);
}

void nest(void)					//returns to home
{
	unsigned char i;
	for(i=0; i<0x07; i++)
	{
		DLLSetDesiredPosition(i,0);
	}
	DLLRunMotors();
	do {
		DLLMotorRunStatus(&i);
	} while (i);
	return;
}

void hardHome(void)				//homes robot, sets home position
{
	DLLHardHome();
}

void shutdown(void)					//Disables motors and closes connection to device
{
	DLLDisableMotors();
	DLLCloseDevice();
}

void gripperClose(void)                                //Close the gripper
{
	unsigned char i;
	DLLSetDesiredPosition(0x00,-GPCLOSE);
	DLLRunMotors();
	do {
		DLLMotorRunStatus(&i);
	} while (i);
}	
	
void gripperOpen(void)                                 //Open the gripper
{
	unsigned char i;
	DLLSetDesiredPosition(0x00,GPCLOSE);
	DLLRunMotors();
	do {
		DLLMotorRunStatus(&i);
	} while (i);
}	

