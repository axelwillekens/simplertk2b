#include <iostream>
#include <string>

extern "C" {
    #include "src/serialComm.h"
}

int main() {
    std::string portname1 = "/dev/ttyACM0";
    initSerialComm(portname1.c_str());

    nmealine* nmealine = readLineSerialPort();
    while (nmealine != NULL) {
        if (nmealine == NULL) {
            std::cerr << "Read Line is failed!" << std::endl;
        }
        std::cout << "Read line: " << nmealine->line << std::endl;
        freeNmeaLine(&nmealine);
        nmealine = readLineSerialPort();
    }


    CloseSerialPort();
}

