#ifndef __5150USB_H__
#define __5150USB_H__

// A. Sheaff University of Maine 1/21/2008
// Electrical and Computer Engineering

// 5150 routines.  Documented in the accompanying PDF

#include <sys/types.h>

long int DLLGetDevicePortNumber(void);
long int DLLIsDeviceOpen(void);
long int DLLOpenDevice(void);
long int DLLCloseDevice(void);
long int DLLSetTTLOutput(u_int8_t byOutputId, u_int8_t byState );
long int DLLGetTTLOutput(u_int8_t byOutputId, u_int8_t *pbyState );
long int DLLGetTTLInput(u_int8_t byInputId, u_int8_t *pbyState );
long int DLLGetSwitch(u_int8_t bySwitchId, u_int8_t *pbyState );
long int DLLSetSolenoidOutput( u_int8_t bySolenoidId, u_int8_t byState );
long int DLLGetSolenoidOutput( u_int8_t bySolenoidId, u_int8_t *pbyState );
long int DLLGetSelectStatus( u_int8_t *pbyState );
long int DLLResetBoard(void);
long int DLLEnableMotors(void);
long int DLLDisableMotors(void);
long int DLLMoveMotor(u_int8_t byMotorId, u_int8_t byDirection);
long int DLLSetPosition(u_int8_t byMotorId, int sPosition);
long int DLLGetPosition(u_int8_t byMotorId, int *psPosition);
long int DLLSetDesiredPosition(u_int8_t byMotorId, int sPosition);
long int DLLGetDesiredPosition(u_int8_t byMotorId, int *psPosition);
long int DLLResetDesiredPosition(void);
long int DLLSetDesiredOffset(u_int8_t byMotorId, int sOffset);
long int DLLSetFrequency(u_int8_t byMotorId, unsigned short usFrequency);
long int DLLGetFrequency(u_int8_t byMotorId, unsigned short *pusFrequency);
long int DLLRunMotors(void);
long int DLLStopMotors(void);
long int DLLHardHome(void);
long int DLLGetHardwareInfo(void *pvData);
long int DLLUpdateFirmware(char *pszFileName);
long int DLLGetUpdateStatus(int *pnStatus, int *pnLineCount, int *pnMaxCount);
long int DLLTerminateUpdate(void);
long int DLLSetSelectMonitor(u_int8_t byState);
long int DLLMotorRunStatus(u_int8_t *pbyStatus);
long int DLLGetAllCurrentPosition(u_int8_t * runningMotorFlag, int * positions);
long int DLLMoveToCalibPos(void);
long int DLLSaveCalib(void);
long int DLLDefaultCalib(void);

#define  DLL_SUCCESS                100
#define  DLL_ERROR_NO_DEVICE        101
#define  DLL_ERROR_EMPTY_COMMAND    102
#define  DLL_ERROR_FILE_WRITE       103
#define  DLL_ERROR_FILE_READ        104
#define  DLL_ERROR_BAD_SYNC         105
#define  DLL_ERROR_BAD_REPLY        106

#define  DLL_STATUS_UPDATE_WORKING  200
#define  DLL_STATUS_UPDATE_SUCCESS  201
#define  DLL_STATUS_UPDATE_ERROR    202

#define  REPLY_TIMEOUT              1000           // 1000ms for all commands
#define  HARDHOME_REPLY_TIMEOUT     60000          // 60 seconds for Hard Home

#endif //__5150USB_H__
