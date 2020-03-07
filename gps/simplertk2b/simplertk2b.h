#ifndef SIMPLERTKB_H
#define SIMPLERTKB_H

#ifdef __cplusplus
extern "C"{
#endif

#include "../serialcomm/serialComm.h"
#include "../ntrip/ntrip.h"

#ifdef __cplusplus
}
#endif

#include <iostream>
#include <string>
#include <thread>
#include <chrono>

class Simplertk2b
{
private:
    std::chrono::system_clock::time_point starttime;

    std::thread ntripThread;
    std::thread serialThread;
    bool ntripActive;

    std::string mountpoint;
    std::string username;
    std::string passwd;

    std::string portname;
    int serial_port;
    int sockfd;

    int ntripdelay;
    bool firstntripsent;
public:
    Simplertk2b();
    Simplertk2b(std::string portname, std::string mountpoint, std::string username, std::string passwd);
    ~Simplertk2b();

    bool isNtripActive();
    std::chrono::system_clock::time_point getStartTime();
    void setStartTime(std::chrono::system_clock::time_point starttime);

    int getSerialPort();
    void setSerialPort(int);
    int getSockfd();
    std::string getPortName();
    int getNTRIPdelay();
    void setNTRIPdelay(int);
    bool getFirstNTRIPsent();
    void setFirstNTRIPsent(bool);
};

void serialCallback(Simplertk2b* simplertk2b);

#endif //SIMPLERTKB_H