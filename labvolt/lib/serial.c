// A. Sheaff University of Maine 1/21/2008
// Electrical and Computer Engineering

// Serial routines

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/time.h>
#include <string.h>

#include "serial.h"

static struct {
	int fd;
} serial = {-1};

int serial_init(const char * dev, unsigned int baud)
{
	struct termios t;
	int err;

	serial.fd=open(dev,O_RDWR|O_NOCTTY);
	if (serial.fd<0) {
		serial.fd=-1;
		perror("open()");
		return -errno;
	}
	memset(&t,0,sizeof(struct termios));
	cfmakeraw(&t);
	cfsetspeed(&t,baud);
	err=tcsetattr(serial.fd,TCSANOW,&t);
	if (err<0) {
		perror("tcsetattr");
		close(serial.fd);
		serial.fd=-errno;
	}
	return serial.fd;
}

ssize_t serial_read(char * buf, size_t count)
{
	fd_set rs;
	struct timeval tv;
	size_t cnt=0;
	int ret;
	static int to=0;

	while (cnt<count) {
		FD_ZERO(&rs);
		FD_SET(serial.fd,&rs);
		tv.tv_sec=60;
		tv.tv_usec=0;
		ret=select(serial.fd+1,&rs,NULL,NULL,&tv);
		if (ret==0) {
			fprintf(stderr,"Timeout.  Read %zu bytes.\n",cnt);
			to++;
			if (to>5) {
				fprintf(stderr,"Excessive timeouts! Aborting.\n");
				fprintf(stderr,"Check cables and scope power and serial settings.\n");
				return -1;
			}
			return cnt;
		}
		if (ret<0) {
			perror("select()");
			return -1;
		}
		if (ret>0) {
			cnt+=read(serial.fd,&buf[cnt],1);
			//printf("->%x\n",(unsigned char)buf[cnt-1]);
		}
	}
	return cnt;
}

ssize_t serial_write(const char * buf, size_t count)
{
	//int i;
	
	//for (i=0;i<count;i++) {
	//	printf(" <-%x\n",(unsigned char)buf[i]);
	//}
	return write(serial.fd,buf,count);
}

int serial_release(void)
{
	return close(serial.fd);
}

int serial_isinit(void)
{
	if (serial.fd<0) return 0;
	return 1;
}
