#include <iostream>
#include <string>
#include <thread>
#include <chrono>

// https://github.com/jmaye/libposlv --> for NTRIP client
// https://ava.upuaut.net/?p=768 --> info over gps

extern "C" {
    #include "lib/serialcomm/serialComm.h"
}

extern "C" {
    #include "lib/ntrip/ntrip.h"
}

int main() {
    // std::string portname1 = "/dev/ttyACM0";
    // initSerialComm(portname1.c_str());

    // nmealine nmealine;
    // while (readLineSerialPort(&nmealine) == 0) {
    //     std::cout << "Read line: " << nmealine.line << std::endl;
    // }
    // CloseSerialPort();

    connectNtrip("flepos.vlaanderen.be", "FLEPOSVRS32GREC", 2101, "852a009", "97115");

    std::thread t(socketcallback);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    while (sendGGA("$GPGGA,122724.00,5104.07582,N,00337.45089,E,1,12,0.54,13.8,M,46.0,M,,*69\r\n", 74) != -1) {
        printf("GGA string sended!\r\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(100000));
    }
    
}

