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
    char fullline[LINELENGTH];
    int size;
} nmealine;

int initSerialComm(const char*);
int readLineSerialPort(int serial_port, nmealine* line_ptr);
void WriteSerialPort(int serial_port, char* msg, int size);
void CloseSerialPort(int serial_port);
int checksumcheck(nmealine*, int);

#endif // SERIALCOMM_H
