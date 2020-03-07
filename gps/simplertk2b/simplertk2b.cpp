#include "simplertk2b.h"


Simplertk2b::Simplertk2b(std::string serialportname, std::string mountpoint, std::string username, std::string passwd) : starttime(std::chrono::system_clock::now())
    , ntripActive(false), mountpoint(mountpoint), username(username), passwd(passwd), portname(serialportname), ntripdelay(3), firstntripsent(false)
{
    // Init of NTRIP server
    struct Args args = {0};
    args.server = "flepos.vlaanderen.be";
    args.mount = mountpoint.c_str();
    args.port = 2101;
    args.user = username.c_str();
    args.password = passwd.c_str();

    if ((this->sockfd = connectNtrip(&args)) != -1) { 
        this->ntripActive = true;
        ntripThread = std::thread(socketcallback,this->sockfd,&args);
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    } else {
        std::cerr << "Connection with NTRIP server failed! See above error for more info." << std::endl;
    }

    // Init Serial Port
    this->setSerialPort(initSerialComm(portname.c_str()));
    if (this->serial_port == -1) {
        std::cerr << "Cannot make a connection on this port" << std::endl;
    } else {
        serialThread = std::thread(serialCallback, this);
    }
}

Simplertk2b::~Simplertk2b() {
    CloseSerialPort(this->serial_port);  
    if (serialThread.joinable()) serialThread.join();
    if (ntripThread.joinable()) ntripThread.join();  
}

bool Simplertk2b::isNtripActive() {
    return this->ntripActive;
}

std::chrono::system_clock::time_point Simplertk2b::getStartTime() {
    return starttime;
}

void Simplertk2b::setStartTime(std::chrono::system_clock::time_point starttime) {
    this->starttime = starttime;
}

int Simplertk2b::getSerialPort() {
    return this->serial_port;
}

void Simplertk2b::setSerialPort(int serial_port) {
     this->serial_port = serial_port;
     updateSerial_port(serial_port);
}

int Simplertk2b::getSockfd() {
    return this->sockfd;
}

int Simplertk2b::getNTRIPdelay() {
    return this->ntripdelay;
}

void Simplertk2b::setNTRIPdelay(int ntripdelay) {
    this->ntripdelay = ntripdelay;
}

std::string Simplertk2b::getPortName() {
    return this->portname;
}

bool Simplertk2b::getFirstNTRIPsent() {
    return this->firstntripsent;
}

void Simplertk2b::setFirstNTRIPsent(bool firstntripsent) {
    this->firstntripsent = firstntripsent;
}


void serialCallback(Simplertk2b* simplertk2b) {
    nmealine nmealine;
    int ret;

    while (1) {
        ret = readLineSerialPort(simplertk2b->getSerialPort(), &nmealine);
        if (ret == 0) { // SUCCESS
            char name[7];
            for (int i = 0; i < 7; i++) name[i] = '\0';
            strncpy(name, nmealine.line, 6);
            if (strcmp(name, "$GNGGA") == 0 || strcmp(name, "$GNRMC") == 0) {
                std::cout << "Read line: " << nmealine.line << std::endl;
            }

            if (simplertk2b->isNtripActive()) {
                auto curtime = std::chrono::system_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(curtime - simplertk2b->getStartTime()).count() > simplertk2b->getNTRIPdelay()) { // send gga string @ 0.1 Hz
                    if (sendGGA(simplertk2b->getSockfd(), "$GPGGA,122724.00,5104.07582,N,00337.45089,E,1,12,0.54,13.8,M,46.0,M,,*69\r\n", 74) == 0) {
                        std::cout << "GGA string sent!\r\n" << std::endl;
                        simplertk2b->setStartTime(curtime);
                        if (simplertk2b->getFirstNTRIPsent()) {
                            simplertk2b->setNTRIPdelay(10);
                            simplertk2b->setFirstNTRIPsent(false);
                        }
                    }
                }
            }
        } else if (ret == -1) { // Init Serial port again!
            std::cerr << "file descriptor failed init new one!" << std::endl;
            int serial_port;
            if ((serial_port = initSerialComm(simplertk2b->getPortName().c_str())) == -1) {
                std::cerr << "Cannot make a connection on this port" << std::endl;
            } else {
                simplertk2b->setSerialPort(serial_port);
            }
        }
    }
    
}