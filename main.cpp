#include <iostream>

extern "C" {
    #include "src/serialComm.h"
}

int main() {
    std::cout << "hello world" << std::endl;
    initSerialComm();

    nmealine* line = readLineSerialPort();
    while (line != NULL) {
        if (line == NULL) {
            std::cerr << "Read Line is failed!" << std::endl;
        }
        std::cout << "Read line: " << line->line << std::endl;
        std::cout << "Checksum: " << line->checksum << std::endl;
        freeNmeaLine(&line);
        line = readLineSerialPort();
    }


    CloseSerialPort();
}

