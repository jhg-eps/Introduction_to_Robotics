// A. Sheaff University of Maine 1/21/2008
// Electrical and Computer Engineering

// 5150 robot routines

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
#include <libudev.h>

#include "serial.h"
#include "5150USB.h"

#define ROBOT_TXC	(0x55)
#define ROBOT_RXC	(0x77)

struct robot_io_status {
	u_int8_t	r_status;
	u_int8_t	r_state;
};

 
// Helpers
static void robot_add_checksum(char * buf, size_t count);
static int robot_verify_checksum(char * buf, size_t count);
static int robot_read_response(char * buf, size_t count);
static int robot_check_response(char * buf, size_t count);
static int robot_command(char * packet, size_t plen, void * data, size_t dlen);

// 	udevadm info -a -p /sys/bus/usb-serial/devices/ttyUSBX
// 	gives information about the serial device and all 
// 	parent device nodes
long DLLGetDevicePortNumber(void)
{
	const char *dev_path;
	const char *id_vendor;
	const char *id_product;
	const char *s_port_no;
	long port_no = -1;

	struct udev *udev;
	struct udev_enumerate *enumerate;
	struct udev_list_entry *devices, *dev_list_entry;
	struct udev_device *dev, *pdev;

	// Create the udev object
	udev = udev_new();
	if (!udev) { return port_no; }

	// Get a list of usb-serial devices
	enumerate = udev_enumerate_new(udev);
	if (!enumerate) { 
		udev_unref(udev);
		return port_no;
	}
	udev_enumerate_add_match_subsystem(enumerate,"usb-serial");
	udev_enumerate_scan_devices(enumerate);
	devices = udev_enumerate_get_list_entry(enumerate);
	if (!devices) {
		udev_enumerate_unref(enumerate);
		udev_unref(udev);
		return port_no;
	}

	udev_list_entry_foreach(dev_list_entry,devices) {
		// Get the filename of the /sys entry for the device
		dev_path = udev_list_entry_get_name(dev_list_entry);
		dev = udev_device_new_from_syspath(udev,dev_path);

		// List is null terminated
		if (!dev) { break; }

		// Get the parent node and check if it's a LabVolt
		// Repeat
		// Start with the usb-serial device
		pdev = dev;
		do {
			// Get the vendor and product ids
			id_vendor = udev_device_get_sysattr_value(pdev,"idVendor");
			id_product = udev_device_get_sysattr_value(pdev,"idProduct");
			if (!id_vendor || !id_product) { continue; }

			// Check if this is a LabVolt robot
			// If not go to the next usb-serial device
			// id_Vendor == 0x10c4
			// idProduct == 0xea60
			if (strncmp(id_vendor,"10c4",4) || strncmp(id_product,"ea60",4)) { continue; }

			s_port_no = udev_device_get_sysattr_value(dev,"port_number");
			if (!s_port_no) { continue; }

			if (sscanf(s_port_no, "%li", &port_no) == 1) { 
				// We found the device we wanted
				// Break out
				break;
			}

		// Go until a null parent is found
		} while((pdev = udev_device_get_parent(pdev)));
		udev_device_unref(dev);
		if (port_no != -1) { break; }
	}
	udev_enumerate_unref(enumerate);
	udev_unref(udev);

	return port_no;
}

long int DLLIsDeviceOpen(void)
{
	
	if (serial_isinit()) return DLL_SUCCESS;
	if (DLLGetDevicePortNumber()==-1) return DLL_ERROR_NO_DEVICE;
	return DLL_ERROR_FILE_WRITE;
}

long int DLLOpenDevice(void)
{
	char name[256];

	if (DLLGetDevicePortNumber()==-1) return DLL_ERROR_NO_DEVICE;
	sprintf(name,"/dev/ttyUSB%ld",DLLGetDevicePortNumber());
	if (serial_init(name,B9600)<0) return DLL_ERROR_NO_DEVICE;
	return DLL_SUCCESS;
	
}

long int DLLCloseDevice(void)
{
	if (DLLGetDevicePortNumber()==-1) return DLL_ERROR_NO_DEVICE;
	if (DLLIsDeviceOpen()!=DLL_SUCCESS) return DLL_ERROR_NO_DEVICE;
	if (serial_release()==0) return DLL_SUCCESS;
	return DLL_ERROR_NO_DEVICE;
}

// DLLSetTTLOutput Tx: Header, Count,  Cmd,  Id, State, CSum
// DLLSetTTLOutput Tx:   0x55,  0x04, 0x01,  Id, State, CSum
// DLLSetTTLOutput Rx: Header, Count, Stat, CSum
// DLLSetTTLOutput Rx:   0x77,  0x02, Stat, CSum
long int DLLSetTTLOutput(u_int8_t byOutputId, u_int8_t byState )
{
	char buf[]={ROBOT_TXC,0x00,0x01,0x00,0x00,0x00};
	unsigned long int ret;
	unsigned char status;

	buf[1]=sizeof(buf)-2;
	buf[3]=byOutputId;
	buf[4]=byState;
	ret=robot_command(buf,sizeof(buf),(void *)&status,1);
	return ret;
}

// DLLGetTTLOutput Tx: Header, Count,  Cmd,  Id, CSum
// DLLGetTTLOutput Tx:   0x55,  0x03, 0x02,  Id, CSum
// DLLGetTTLOutput Rx: Header, Count, Stat, State, CSum
// DLLGetTTLOutput Rx:   0x77,  0x03, Stat, State, CSum
long int DLLGetTTLOutput(u_int8_t byOutputId, u_int8_t *pbyState)
{
	char buf[]={ROBOT_TXC,0x00,0x02,0x00,0x00};
	long int ret;
	struct robot_io_status rios;

	buf[1]=sizeof(buf)-2;
	buf[3]=byOutputId;
	ret=robot_command(buf,sizeof(buf),(void *)&rios,sizeof(struct robot_io_status));
	if (ret==DLL_SUCCESS) pbyState[0]=rios.r_state;
	return ret;
}

// DLLGetTTLInput Tx: Header, Count,  Cmd,  Id, CSum
// DLLGetTTLInput Tx:   0x55,  0x03, 0x03,  Id, CSum
// DLLGetTTLInput Rx: Header, Count, Stat, State, CSum
// DLLGetTTLInput Rx:   0x77,  0x03, Stat, State, CSum
long int DLLGetTTLInput(u_int8_t byInputId, u_int8_t *pbyState)
{
	char buf[]={ROBOT_TXC,0x00,0x03,0x00,0x00};
	long int ret;
	struct robot_io_status rios;

	buf[1]=sizeof(buf)-2;
	buf[3]=byInputId;
	ret=robot_command(buf,sizeof(buf),(void *)&rios,sizeof(struct robot_io_status));
	if (ret==DLL_SUCCESS) pbyState[0]=rios.r_state;
	return ret;
}

// DLLGetSwitch Tx: Header, Count,  Cmd,  Id, CSum
// DLLGetSwitch Tx:   0x55,  0x03, 0x04,  Id, CSum
// DLLGetSwitch Rx: Header, Count, Stat, State, CSum
// DLLGetSwitch Rx:   0x77,  0x03, Stat, State, CSum
long int DLLGetSwitch(u_int8_t bySwitchId, u_int8_t *pbyState)
{
	char buf[]={ROBOT_TXC,0x00,0x04,0x00,0x00};
	long int ret;
	struct robot_io_status rios;

	buf[1]=sizeof(buf)-2;
	buf[3]=bySwitchId;
	ret=robot_command(buf,sizeof(buf),(void *)&rios,sizeof(struct robot_io_status));
	if (ret==DLL_SUCCESS) pbyState[0]=rios.r_state;
	return ret;
}

// DLLSetSolenoidOutput Tx: Header, Count,  Cmd,  Id, State, CSum
// DLLSetSolenoidOutput Tx:   0x55,  0x04, 0x05,  Id, State, CSum
// DLLSetSolenoidOutput Rx: Header, Count, Stat, CSum
// DLLSetSolenoidOutput Rx:   0x77,  0x02, Stat, CSum
long int DLLSetSolenoidOutput( u_int8_t bySolenoidId, u_int8_t byState)
{
	char buf[]={ROBOT_TXC,0x00,0x05,0x00,0x00,0x00};
	long int ret;
	unsigned char status;

	buf[1]=sizeof(buf)-2;
	buf[3]=bySolenoidId;
	buf[4]=byState;
	ret=robot_command(buf,sizeof(buf),(void *)&status,1);
	return ret;
}

// DLLGetSolenoidOutput Tx: Header, Count,  Cmd,  Id, CSum
// DLLGetSolenoidOutput Tx:   0x55,  0x03, 0x06,  Id, CSum
// DLLGetSolenoidOutput Rx: Header, Count, Stat, State, CSum
// DLLGetSolenoidOutput Rx:   0x77,  0x03, Stat, State, CSum
long int DLLGetSolenoidOutput( u_int8_t bySolenoidId, u_int8_t *pbyState)
{
	char buf[]={ROBOT_TXC,0x00,0x06,0x00,0x00};
	long int ret;
	struct robot_io_status rios;

	buf[1]=sizeof(buf)-2;
	buf[3]=bySolenoidId;
	ret=robot_command(buf,sizeof(buf),(void *)&rios,sizeof(struct robot_io_status));
	if (ret==DLL_SUCCESS) pbyState[0]=rios.r_state;
	return ret;
}

// DLLGetSolenoidOutput Tx: Header, Count,  Cmd, CSum
// DLLGetSolenoidOutput Tx:   0x55,  0x02, 0x07, CSum
// DLLGetSolenoidOutput Rx: Header, Count, Stat, State, CSum
// DLLGetSolenoidOutput Rx:   0x77,  0x03, Stat, State, CSum
long int DLLGetSelectStatus( u_int8_t *pbyState)
{
	char buf[]={ROBOT_TXC,0x00,0x07,0x00};
	long int ret;
	struct robot_io_status rios;

	buf[1]=sizeof(buf)-2;
	ret=robot_command(buf,sizeof(buf),(void *)&rios,sizeof(struct robot_io_status));
	if (ret==DLL_SUCCESS) pbyState[0]=rios.r_state;
	return ret;
}

// DLLResetBoard Tx: Header, Count,  Cmd, CSum
// DLLResetBoard Tx:   0x55,  0x02, 0x08, CSum
// DLLResetBoard Rx: Header, Count, Stat, CSum
// DLLResetBoard Rx:   0x77,  0x02, Stat, CSum
long int DLLResetBoard(void)
{
	char buf[]={ROBOT_TXC,0x00,0x08,0x00};
	long int ret;
	unsigned char status;

	buf[1]=sizeof(buf)-2;
	ret=robot_command(buf,sizeof(buf),(void *)&status,1);
	return ret;
}

// DLLEnableMotors Tx: Header, Count,  Cmd, CSum
// DLLEnableMotors Tx:   0x55,  0x02, 0x09, CSum
// DLLEnableMotors Rx: Header, Count, Stat, CSum
// DLLEnableMotors Rx:   0x77,  0x02, Stat, CSum
long int DLLEnableMotors(void)
{
	char buf[]={ROBOT_TXC,0x00,0x09,0x00};
	long int ret;
	unsigned char status;

	buf[1]=sizeof(buf)-2;
	ret=robot_command(buf,sizeof(buf),(void *)&status,1);
	return ret;
}

// DLLDisableMotors Tx: Header, Count,  Cmd, CSum
// DLLDisableMotors Tx:   0x55,  0x02, 0x0a, CSum
// DLLDisableMotors Rx: Header, Count, Stat, CSum
// DLLDisableMotors Rx:   0x77,  0x02, Stat, CSum
long int DLLDisableMotors(void)
{
	char buf[]={ROBOT_TXC,0x00,0x0a,0x00};
	long int ret;
	unsigned char status;

	buf[1]=sizeof(buf)-2;
	ret=robot_command(buf,sizeof(buf),(void *)&status,1);
	return ret;
}

// DLLMoveMotor Tx: Header, Count,  Cmd,  Id, Dir, CSum
// DLLMoveMotor Tx:   0x55,  0x04, 0x0b,  Id, Dir, CSum
// DLLMoveMotor Rx: Header, Count, Stat, CSum
// DLLMoveMotor Rx:   0x77,  0x02, Stat, CSum
long int DLLMoveMotor(u_int8_t byMotorId, u_int8_t byDirection)
{
	char buf[]={ROBOT_TXC,0x00,0x0b,0x00,0x00,0x00};
	unsigned long int ret;
	unsigned char status;

	buf[1]=sizeof(buf)-2;
	buf[3]=byMotorId;
	buf[4]=byDirection;
	ret=robot_command(buf,sizeof(buf),(void *)&status,1);
	return ret;
}

// DLLSetPosition Tx: Header, Count,  Cmd, Id, PosLSB, Pos1, Pos2, PosMSB, CSum
// DLLSetPosition Tx:   0x55,  0x07, 0x0c, Id, PosLSB, Pos1, Pos2, PosMSB, CSum
// DLLSetPosition Rx: Header, Count, Stat, CSum
// DLLSetPosition Rx:   0x77,  0x02, Stat, CSum
long int DLLSetPosition(u_int8_t byMotorId, int sPosition)
{
	char buf[]={ROBOT_TXC,0x00,0x0c,0x00,0x00,0x00,0x00,0x00,0x00};
	unsigned long int ret;
	unsigned char status;

	buf[1]=sizeof(buf)-2;
	buf[3]=byMotorId;
	buf[4]=sPosition&0xff;
	buf[5]=(sPosition>>8)&0xff;
	buf[6]=(sPosition>>16)&0xff;
	buf[7]=(sPosition>>24)&0xff;
	ret=robot_command(buf,sizeof(buf),(void *)&status,1);
	return ret;
}

// DLLGetPosition Tx: Header, Count,  Cmd, Id, CSum
// DLLGetPosition Tx:   0x55,  0x03, 0x0d, Id, CSum
// DLLGetPosition Rx: Header, Count, Stat, PosLSB, Pos1, Pos2, PosMSB, CSum
// DLLGetPosition Rx:   0x77,  0x06, Stat, PosLSB, Pos1, Pos2, PosMSB, CSum
// Lab-Volt should have used network byte order rather than little-endian.
// It's a hack to convert in a portable way....
long int DLLGetPosition(u_int8_t byMotorId, int *psPosition)
{
	char buf[]={ROBOT_TXC,0x00,0x0d,0x00,0x00};
	unsigned long int ret;
	unsigned char rbuf[5];

	buf[1]=sizeof(buf)-2;
	buf[3]=byMotorId;
	ret=robot_command(buf,sizeof(buf),(void *)&rbuf,5);
	if (ret==DLL_SUCCESS) psPosition[0]=(rbuf[4]<<24)+(rbuf[3]<<16)+(rbuf[2]<<8)+rbuf[1];
	return ret;
}

// DLLSetDesiredPosition Tx: Header, Count,  Cmd, Id, PosLSB, Pos1, Pos2, PosMSB, CSum
// DLLSetDesiredPosition Tx:   0x55,  0x07, 0x0e, Id, PosLSB, Pos1, Pos2, PosMSB, CSum
// DLLSetDesiredPosition Rx: Header, Count, Stat, CSum
// DLLSetDesiredPosition Rx:   0x77,  0x02, Stat, CSum
long int DLLSetDesiredPosition(u_int8_t byMotorId, int sPosition)
{
	char buf[]={ROBOT_TXC,0x00,0x0e,0x00,0x00,0x00,0x00,0x00,0x00};
	unsigned long int ret;
	unsigned char status;

	buf[1]=sizeof(buf)-2;
	buf[3]=byMotorId;
	buf[4]=sPosition&0xff;
	buf[5]=(sPosition>>8)&0xff;
	buf[6]=(sPosition>>16)&0xff;
	buf[7]=(sPosition>>24)&0xff;
	ret=robot_command(buf,sizeof(buf),(void *)&status,1);
	return ret;
}

// DLLGetDesiredPosition Tx: Header, Count,  Cmd, Id, CSum
// DLLGetDesiredPosition Tx:   0x55,  0x03, 0x0f, Id, CSum
// DLLGetDesiredPosition Rx: Header, Count, Stat, PosLSB, Pos1, Pos2, PosMSB, CSum
// DLLGetDesiredPosition Rx:   0x77,  0x06, Stat, PosLSB, Pos1, Pos2, PosMSB, CSum
long int DLLGetDesiredPosition(u_int8_t byMotorId, int *psPosition)
{
	char buf[]={ROBOT_TXC,0x00,0x0f,0x00,0x00};
	unsigned long int ret;
	unsigned char rbuf[5];

	buf[1]=sizeof(buf)-2;
	buf[3]=byMotorId;
	ret=robot_command(buf,sizeof(buf),(void *)&rbuf,5);
	if (ret==DLL_SUCCESS) psPosition[0]=(rbuf[4]<<24)+(rbuf[3]<<16)+(rbuf[2]<<8)+rbuf[1];
	return ret;
}

// DLLResetDesiredPosition Tx: Header, Count,  Cmd, CSum
// DLLResetDesiredPosition Tx:   0x55,  0x02, 0x10, CSum
// DLLResetDesiredPosition Rx: Header, Count, Stat, CSum
// DLLResetDesiredPosition Rx:   0x77,  0x02, Stat, CSum
long int DLLResetDesiredPosition(void)
{
	char buf[]={ROBOT_TXC,0x00,0x10,0x00};
	long int ret;
	unsigned char status;

	buf[1]=sizeof(buf)-2;
	ret=robot_command(buf,sizeof(buf),(void *)&status,1);
	return ret;
}

// DLLSetDesiredOffset Tx: Header, Count,  Cmd, Id, PosLSB, Pos1, Pos2, PosMSB, CSum
// DLLSetDesiredOffset Tx:   0x55,  0x07, 0x11, Id, PosLSB, Pos1, Pos2, PosMSB, CSum
// DLLSetDesiredOffset Rx: Header, Count, Stat, CSum
// DLLSetDesiredOffset Rx:   0x77,  0x02, Stat, CSum
long int DLLSetDesiredOffset(u_int8_t byMotorId, int sOffset)
{
	char buf[]={ROBOT_TXC,0x00,0x11,0x00,0x00,0x00,0x00,0x00,0x00};
	unsigned long int ret;
	unsigned char status;

	buf[1]=sizeof(buf)-2;
	buf[3]=byMotorId;
	buf[4]=sOffset&0xff;
	buf[5]=(sOffset>>8)&0xff;
	buf[6]=(sOffset>>16)&0xff;
	buf[7]=(sOffset>>24)&0xff;
	ret=robot_command(buf,sizeof(buf),(void *)&status,1);
	return ret;
}

// DLLSetFrequency Tx: Header, Count,  Cmd, Id, FreqLSB, FreqMSB, CSum
// DLLSetFrequency Tx:   0x55,  0x05, 0x12, Id, FreqLSB, FreqMSB, CSum
// DLLSetFrequency Rx: Header, Count, Stat, CSum
// DLLSetFrequency Rx:   0x77,  0x02, Stat, CSum
long int DLLSetFrequency(u_int8_t byMotorId, unsigned short usFrequency)
{
	char buf[]={ROBOT_TXC,0x00,0x12,0x00,0x00,0x00,0x00};
	unsigned long int ret;
	unsigned char status;

	buf[1]=sizeof(buf)-2;
	buf[3]=byMotorId;
	buf[4]=usFrequency&0xff;
	buf[5]=(usFrequency>>8)&0xff;
	ret=robot_command(buf,sizeof(buf),(void *)&status,1);
	return ret;
}

// DLLGetFrequency Tx: Header, Count,  Cmd, Id, CSum
// DLLGetFrequency Tx:   0x55,  0x03, 0x13, Id, CSum
// DLLGetFrequency Rx: Header, Count, Stat, FreqLSB, FreqMSB, CSum
// DLLGetFrequency Rx:   0x77,  0x04, Stat, FreqLSB, FreqMSB, CSum
long int DLLGetFrequency(u_int8_t byMotorId, unsigned short *pusFrequency)
{
	char buf[]={ROBOT_TXC,0x00,0x13,0x00,0x00};
	unsigned long int ret;
	unsigned char rbuf[3];

	buf[1]=sizeof(buf)-2;
	buf[3]=byMotorId;
	ret=robot_command(buf,sizeof(buf),(void *)&rbuf,3);
	if (ret==DLL_SUCCESS) pusFrequency[0]=(rbuf[2]<<8)+rbuf[1];
	return ret;
}

// DLLRunMotors Tx: Header, Count,  Cmd, CSum
// DLLRunMotors Tx:   0x55,  0x02, 0x14, CSum
// DLLRunMotors Rx: Header, Count, Stat, CSum
// DLLRunMotors Rx:   0x77,  0x02, Stat, CSum
long int DLLRunMotors(void)
{
	char buf[]={ROBOT_TXC,0x00,0x14,0x00};
	long int ret;
	unsigned char status;

	buf[1]=sizeof(buf)-2;
	ret=robot_command(buf,sizeof(buf),(void *)&status,1);
	return ret;
}

// DLLStopMotors Tx: Header, Count,  Cmd, CSum
// DLLStopMotors Tx:   0x55,  0x02, 0x15, CSum
// DLLStopMotors Rx: Header, Count, Stat, CSum
// DLLStopMotors Rx:   0x77,  0x02, Stat, CSum
long int DLLStopMotors(void)
{
	char buf[]={ROBOT_TXC,0x00,0x15,0x00};
	long int ret;
	unsigned char status;

	buf[1]=sizeof(buf)-2;
	ret=robot_command(buf,sizeof(buf),(void *)&status,1);
	return ret;
}

// DLLHardHome Tx: Header, Count,  Cmd, CSum
// DLLHardHome Tx:   0x55,  0x02, 0x16, CSum
// DLLHardHome Rx: Header, Count, Stat, CSum
// DLLHardHome Rx:   0x77,  0x02, Stat, CSum
// This needs a 60 second timeout on RX
long int DLLHardHome(void)
{
	char buf[]={ROBOT_TXC,0x00,0x16,0x00};
	long int ret;
	unsigned char status;

	buf[1]=sizeof(buf)-2;
	ret=robot_command(buf,sizeof(buf),(void *)&status,1);
	return ret;
}

// DLLGetHardwareInfo Tx: Header, Count,  Cmd, CSum
// DLLGetHardwareInfo Tx:   0x55,  0x02, 0x17, CSum
// DLLGetHardwareInfo Rx: Header,  Count, Stat, Data, ..., CSum
// DLLGetHardwareInfo Rx:   0x77, Varies, Stat, Data, ..., CSum
long int DLLGetHardwareInfo(void *pvData)
{
	char buf[]={ROBOT_TXC,0x00,0x13,0x00};
	unsigned long int ret;
	unsigned char rbuf[32];

	buf[1]=sizeof(buf)-2;
	ret=robot_command(buf,sizeof(buf),(void *)&rbuf,32);
	if (ret==DLL_SUCCESS) {
		if (pvData) {
			memcpy((char *)pvData,(char *)&rbuf[1],4);
		}
	}
	return ret;
}

// Unknown
long int DLLUpdateFirmware(char *pszFileName)
{
	return DLL_ERROR_NO_DEVICE;
}

// Unknown
long int DLLGetUpdateStatus(int *pnStatus, int *pnLineCount, int *pnMaxCount)
{
	return DLL_ERROR_NO_DEVICE;
}

// Unknown
long int DLLTerminateUpdate(void)
{
	return DLL_ERROR_NO_DEVICE;
}

// DLLSetSelectMonitor Tx: Header, Count,  Cmd, State, CSum
// DLLSetSelectMonitor Tx:   0x55,  0x03, 0x19, State, CSum
// DLLSetSelectMonitor Rx: Header, Count, Stat, CSum
// DLLSetSelectMonitor Rx:   0x77,  0x02, Stat, CSum
long int DLLSetSelectMonitor(u_int8_t byState)
{
	char buf[]={ROBOT_TXC,0x00,0x19,0x00,0x00};
	unsigned long int ret;
	unsigned char status;

	buf[1]=sizeof(buf)-2;
	buf[3]=byState;
	ret=robot_command(buf,sizeof(buf),(void *)&status,1);
	return ret;
}

// DLLMotorRunStatus Tx: Header, Count,  Cmd, CSum
// DLLMotorRunStatus Tx:   0x55,  0x02, 0x1a, CSum
// DLLMotorRunStatus Rx: Header, Count, Stat, Mstat, CSum
// DLLMotorRunStatus Rx:   0x77,  0x03, Stat, Mstat, CSum
long int DLLMotorRunStatus(u_int8_t *pbyStatus)
{
	char buf[]={ROBOT_TXC,0x00,0x1a,0x00};
	long int ret;
	struct robot_io_status rios;

	buf[1]=sizeof(buf)-2;
	ret=robot_command(buf,sizeof(buf),(void *)&rios,sizeof(struct robot_io_status));
	if (ret==DLL_SUCCESS) pbyStatus[0]=rios.r_state;
	return ret;
}

// DLLGetAllCurrentPosition Tx: Header, Count,  Cmd, CSum
// DLLGetAllCurrentPosition Tx:   0x55,  0x02, 0x1b, CSum
// DLLGetAllCurrentPosition Rx: Header, Count, Stat, CSum
// DLLGetAllCurrentPosition Rx:   0x77,  0x1f, Stat, Pos, ..., CSum
long int DLLGetAllCurrentPosition(u_int8_t * runningMotorFlag, int * positions)
{
	char buf[]={ROBOT_TXC,0x00,0x1b,0x00};
	unsigned long int ret;
	unsigned char rbuf[5];
	int i;

	buf[1]=sizeof(buf)-2;
	ret=robot_command(buf,sizeof(buf),(void *)&rbuf,5);
	if (ret==DLL_SUCCESS) {
		for (i=0;i<7;i++) {
			positions[i]=(rbuf[4*i+4]<<24)+(rbuf[4*i+3]<<16)+(rbuf[4*i+2]<<8)+rbuf[4*i+1];
		}
	}
	return ret;
}

// DLLMoveToCalibPos Tx: Header, Count,  Cmd, CSum
// DLLMoveToCalibPos Tx:   0x55,  0x02, 0x1c, CSum
// DLLMoveToCalibPos Rx: Header, Count, Stat, CSum
// DLLMoveToCalibPos Rx:   0x77,  0x02, Stat, CSum
long int DLLMoveToCalibPos(void)
{
	char buf[]={ROBOT_TXC,0x00,0x1c,0x00};
	long int ret;
	unsigned char status;

	buf[1]=sizeof(buf)-2;
	ret=robot_command(buf,sizeof(buf),(void *)&status,1);
	return ret;
}

// DLLSaveCalib Tx: Header, Count,  Cmd, CSum
// DLLSaveCalib Tx:   0x55,  0x02, 0x1d, CSum
// DLLSaveCalib Rx: Header, Count, Stat, CSum
// DLLSaveCalib Rx:   0x77,  0x02, Stat, CSum
long int DLLSaveCalib(void)
{
	char buf[]={ROBOT_TXC,0x00,0x1d,0x00};
	long int ret;
	unsigned char status;

	buf[1]=sizeof(buf)-2;
	ret=robot_command(buf,sizeof(buf),(void *)&status,1);
	return ret;
}

// DLLDefaultCalib Tx: Header, Count,  Cmd, CSum
// DLLDefaultCalib Tx:   0x55,  0x02, 0x1e, CSum
// DLLDefaultCalib Rx: Header, Count, Stat, CSum
// DLLDefaultCalib Rx:   0x77,  0x02, Stat, CSum
long int DLLDefaultCalib(void)
{
	char buf[]={ROBOT_TXC,0x00,0x1e,0x00};
	long int ret;
	unsigned char status;

	buf[1]=sizeof(buf)-2;
	ret=robot_command(buf,sizeof(buf),(void *)&status,1);
	return ret;
}

//-------------------------------------------------------
//
//              HELPERS
//
//-------------------------------------------------------

// Read a response.
// Response data starts with 0x77 followed by a count.
static int robot_read_response(char * buf, size_t count)
{
	int i, r;

	i=0;
	buf[1]=0xff;
	do {
		r=serial_read(&buf[i],1);
		if (r<0) return -1;
		i=i+r;
	} while (((i-2)<(unsigned char)buf[1])&&(i<count));

	return i;
}

static int robot_verify_checksum(char * buf, size_t count)
{
	int i;
	unsigned char csum=0;

	i=0;
	for (i=0;i<count-1;i++) {
		csum+=(unsigned char)buf[i];
	}
	return (csum==(unsigned char)buf[i]);
}

static void robot_add_checksum(char * buf, size_t count)
{
	int i;
	unsigned char *ucp=(unsigned char *)buf;

	buf[count-1]=0;
	for (i=0;i<count-1;i++) {
		ucp[count-1]+=ucp[i];
	}
	return;
}

static int robot_check_response(char * buf, size_t count)
{
	if (buf[0]!=ROBOT_RXC) return 0;
	if (buf[1]!=(count-2)) return 0;
	return 1;
}

static int robot_command(char * packet, size_t plen, void * data, size_t dlen)
{
	char buf[256];
	int rxlen;
	long int ret;

	if (!data) {
		return DLL_ERROR_FILE_WRITE;
	}
	ret=DLLIsDeviceOpen();
	if (ret!=DLL_SUCCESS) return ret;
	robot_add_checksum(packet,plen);
	serial_write(packet,plen);
	rxlen=robot_read_response(buf,256);
	if (rxlen<0) return DLL_ERROR_FILE_READ;
	if (!robot_verify_checksum(buf,rxlen)) return DLL_ERROR_BAD_REPLY;
	if (!robot_check_response(buf,rxlen)) return DLL_ERROR_BAD_REPLY;
	if (dlen<=(buf[1])) {
		memcpy((char *)data,&buf[2],dlen);
	} else {
		memcpy((char *)data,&buf[2],(unsigned char)buf[1]);
	}
	return DLL_SUCCESS;
}
