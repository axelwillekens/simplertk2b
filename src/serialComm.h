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

typedef struct nmealine {
    char* line;
    int size;
} nmealine;

int serial_port;
struct termios tty;

void initSerialComm(const char*);
nmealine* readLineSerialPort();
void WriteSerialPort(unsigned char*);
void CloseSerialPort();

int checksumcheck(nmealine*, int);
void freeNmeaLine(nmealine**);

#endif // SERIALCOMM_H
