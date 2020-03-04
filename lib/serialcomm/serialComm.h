// Got info from https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/

#ifndef SERIALCOMM_H
#define SERIALCOMM_H

// C library headers
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#define LINELENGTH 512

typedef struct nmealine {
    char line[LINELENGTH];
    int size;
} nmealine;

int serial_port;
struct termios tty;

void initSerialComm(const char*);
int readLineSerialPort(nmealine* line_ptr);
void WriteSerialPort(unsigned char* msg, int size);
void CloseSerialPort();
int checksumcheck(nmealine*, int);

#endif // SERIALCOMM_H
