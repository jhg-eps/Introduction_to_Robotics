#ifndef __SERIAL_H__
#define __SERIAL_H__

// A. Sheaff University of Maine 1/21/2008
// Electrical and Computer Engineering

// Serial routines

int serial_init(const char * dev, unsigned int baud);
ssize_t serial_read(char * buf, size_t count);
ssize_t serial_write(const char * buf, size_t count);
int serial_release(void);
int serial_isinit(void);

#endif // __SERIAL_H__
