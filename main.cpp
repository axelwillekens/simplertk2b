#include <iostream>
#include <string>

// https://github.com/jmaye/libposlv --> for NTRIP client
// https://ava.upuaut.net/?p=768 --> info over gps

extern "C" {
    #include "lib/serialcomm/serialComm.h"
}

int main() {
    std::string portname1 = "/dev/ttyACM0";
    initSerialComm(portname1.c_str());

    nmealine nmealine;
    while (readLineSerialPort(&nmealine) == 0) {
        std::cout << "Read line: " << nmealine.line << std::endl;
    }

    CloseSerialPort();
}

