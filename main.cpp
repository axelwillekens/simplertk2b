#include <iostream>
#include <string>

// https://github.com/jmaye/libposlv --> for NTRIP client
// https://ava.upuaut.net/?p=768 --> info over gps

extern "C" {
    #include "lib/serialcomm/serialComm.h"
}
#include "lib/ntrip/NTRIPClient.h"

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

    // NTRIPClient ntripclient("flepos.vlaanderen.be", 2101, "FLEPOSVRS32GREC", "852a009", "97115");
    // ntripclient.open();

    // char buf[1024];
    // ntripclient.getStreamReader().read(buf,1024);
    // std::cout << buf << std::endl;

    // ntripclient.close();
}

