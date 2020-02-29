#include "serialComm.h"

void initSerialComm(const char* portname) {
    // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
    serial_port = open(portname, O_RDWR);
    memset(&tty, 0, sizeof tty);

    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) {
        fprintf(stderr, "Error %i from tcgetattr: %s\n", errno, strerror(errno));
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
        fprintf(stderr, "Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }
}

// Read a NMEA line from the port
int readLineSerialPort(nmealine* line_ptr) {
    // init parameters
    line_ptr->size = 0;
    for(int i=0; i < LINELENGTH; i++ ) {
        line_ptr->line[i] = '\0';
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
        fprintf(stderr, "Error readLineSerialPort: reading: %s \n", strerror(errno));
        return -1;
    }

    // read checksum
    char checksumbuf[5] = {0};
    int i = 0;
    num_bytes = read(serial_port, &readchar, sizeof(char));
    while (readchar != '\n' && num_bytes >= 0) {
        checksumbuf[i] = readchar;
        num_bytes = read(serial_port, &readchar, sizeof(char));
        i++;
    }
    if (checksumcheck(line_ptr, (int)strtol(checksumbuf, NULL, 16)) < 0) {
        fprintf(stderr, "Bad line: Checksum does not match! \n");
    }
    
    // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
    if (num_bytes < 0) {
        fprintf(stderr, "Error readLineSerialPort: reading: %s \n", strerror(errno));
        return -1;
    }

    return 0;
}

// Write to serial port
void WriteSerialPort(unsigned char* msg) {
    write(serial_port, msg, sizeof(msg));
}

// Close the serial port
void CloseSerialPort() {
    close(serial_port);
}

// method that does the checksum. 
// returns 0 if OK
// returns -1 if NOT OK
int checksumcheck(nmealine* nmealine, int checksum) {
    int sum = (int) nmealine->line[1];
    for (int i=2; i < nmealine->size; i++) {
        sum = sum ^ (int) nmealine->line[i];
    }

    if (sum != checksum) return -1;
    else return 0;
}
