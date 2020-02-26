#ifndef SERIALCOMM_H
#define SERIALCOMM_H

// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

int serial_port;
struct termios tty;

void initSerialComm();
void readSerialPort();
void WriteSerialPort(unsigned char msg);
void CloseSerialPort();

#endif // SERIALCOMM_H
