#include "serialComm.h"

// Got info from https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/

void initSerialComm() {
    // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
    serial_port = open("/dev/ttyACM0", O_RDWR);
    memset(&tty, 0, sizeof tty);

    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B57600);
    cfsetospeed(&tty, B57600);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }
}

nmealine* readLineSerialPort() {
    // Allocate memory for read buffer, set size according to your needs
    nmealine* line_ptr = (nmealine*) malloc(sizeof(nmealine));
    if (line_ptr == NULL) {
        printf("Error readLineSerialPort: allocation of nmealine didn't work out! \n");
        return NULL;
    }
    line_ptr->bufsize = 512;
    line_ptr->size = 0;
    line_ptr->line = (char*) calloc(line_ptr->bufsize, sizeof(char));
    if (line_ptr->line == NULL) {
        printf("Error readLineSerialPort: allocation of nmealine didn't work out in method! \n");
        return NULL;
    }

    // Read bytes. The behaviour of read() (e.g. does it block?,
    // how long does it block for?) depends on the configuration
    // settings above, specifically VMIN and VTIME
    char readchar;
    int num_bytes = read(serial_port, &readchar, sizeof(char));
    // read sentence
    while (readchar != '*' && num_bytes >= 0) {
       line_ptr->line[line_ptr->size] = readchar;
       num_bytes = read(serial_port, &readchar, sizeof(char));
       line_ptr->size++;
    }

    // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
    if (num_bytes < 0) {
        printf("Error readLineSerialPort: reading: %s \n", strerror(errno));
        return NULL;
    }

    // read checksum
    char checksumbuf[5] = {'\0'};
    int i = 0;
    num_bytes = read(serial_port, &readchar, sizeof(char));
    while (readchar != '\n' && num_bytes >= 0) {
        checksumbuf[i] = readchar;
        num_bytes = read(serial_port, &readchar, sizeof(char));
        i++;
    }
    line_ptr->checksum = atoi(checksumbuf);
    
    // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
    if (num_bytes < 0) {
        printf("Error readLineSerialPort: reading: %s \n", strerror(errno));
        return NULL;
    }

    return line_ptr;
}

void WriteSerialPort(unsigned char* msg) {
    // Write to serial port
    write(serial_port, msg, sizeof(msg));
}

void CloseSerialPort() {
    close(serial_port);
}

void freeNmeaLine(nmealine** line) {
    free((*line)->line);
    free(*line);
}

