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
#include <sstream>
#include <string>
#include <algorithm>
#include <iterator>
#include <vector>
#include <thread>
#include <chrono>

#include <boost/algorithm/string.hpp>
#include "gganmealine.h"
#include "rmcnmealine.h"

class Simplertk2b
{
private:
    std::chrono::system_clock::time_point starttime;
    bool ntripActive;
    // std::string mountpoint;
    // std::string username;
    // std::string passwd;
    int ntripdelay;
    bool firstntripsent;
    std::string ntripnmealine;
    struct Args args; // args for NTRIP client

    std::string serialportname[2];

    std::thread ntripThread;
    std::thread serialThread[2];

    void (*ggacallback)(GGAnmealine&, int);
    void (*rmccallback)(RMCnmealine&, int);
public:
    Simplertk2b();
    Simplertk2b(std::string serialportnameMaster, std::string serialportnameSlave, std::string server, std::string mountpoint, std::string username, std::string passwd);
    ~Simplertk2b();

    int getSerialPort(int index);
    void setSerialPort(int index, int value);
    std::string getPortName(int index);

    int getSockfd();
    bool isNtripActive();
    std::chrono::system_clock::time_point getStartTime();
    void setStartTime(std::chrono::system_clock::time_point starttime);
    int getNTRIPdelay();
    void setNTRIPdelay(int);
    bool getFirstNTRIPsent();
    void setFirstNTRIPsent(bool);
    std::string getNtripnmealine();

    void setGGAcallback(void (*ggacallback)(GGAnmealine&, int));
    void setRMCcallback(void (*rmccallback)(RMCnmealine&, int));

    void processNMEAline(int index, std::string nmealine, std::string fullnmealine);
};

void serialWork(Simplertk2b* simplertk2b, int index);

#endif //SIMPLERTKB_H