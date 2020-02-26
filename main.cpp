#include <iostream>

extern "C" {
    #include "src/serialComm.h"
}

int main() {
    std::cout << "hello world" << std::endl;
    initSerialComm();
    readSerialPort();
    CloseSerialPort();
}

