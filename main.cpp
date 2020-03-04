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
    // Init serial port
    // std::string portname1 = "/dev/ttyACM0";
    // initSerialComm(portname1.c_str());
    // nmealine nmealine;

    auto starttime = std::chrono::system_clock::now();
    std::thread t;

    // Init of NTRIP server
    if (connectNtrip("flepos.vlaanderen.be", "FLEPOSVRS32GREC", 2101, "852a009", "97115") == 0) { 
        // Only when NTRIP server is cool with it!
        t = std::thread(socketcallback);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        // Send data first time
        sendGGA("$GPGGA,122724.00,5104.07582,N,00337.45089,E,1,12,0.54,13.8,M,46.0,M,,*69\r\n", 74);
    } else {
        std::cerr << "Connection with NTRIP server failed! See above error for more info." << std::endl;
    }
    
    while (sendGGA("$GPGGA,122724.00,5104.07582,N,00337.45089,E,1,12,0.54,13.8,M,46.0,M,,*69\r\n", 74) != 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    }
    

    // Main loop
    // while (readLineSerialPort(&nmealine) == 0) {
    //     // std::cout << "Read line: " << nmealine.line << std::endl;

    //     auto curtime = std::chrono::system_clock::now();
    //     if ((starttime - curtime).count() > 10) { // send gga string @ 0.1 Hz
    //         if (sendGGA("$GPGGA,122724.00,5104.07582,N,00337.45089,E,1,12,0.54,13.8,M,46.0,M,,*69\r\n", 74) != 0) {
    //             perror("sending GGA string failed!\r\n");
    //             starttime = curtime;
    //         }
    //     }
    // }

    CloseSerialPort();  
    t.join();  
}

