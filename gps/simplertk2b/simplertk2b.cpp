#include "simplertk2b.h"
#include "gganmealine.h"
#include "rmcnmealine.h"

Simplertk2b::Simplertk2b(std::string serialportname, std::string mountpoint, std::string username, std::string passwd) : starttime(std::chrono::system_clock::now())
    , ntripActive(false), mountpoint(mountpoint), username(username), passwd(passwd), portname(serialportname), ntripdelay(4), firstntripsent(false)
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

void Simplertk2b::processNMEAline(std::string nmealine, std::string fullnmealine) {
    std::vector<std::string> words;
    boost::split(words, nmealine, boost::is_any_of(","));

    std::vector<std::string>::iterator i;
    if (*words.begin() == "$GNGGA") {
        GGAnmealine ggaline = GGAnmealine();
        ggaline.setFix_taken_time(std::stoi(*(words.begin()+1)));
        ggaline.setLat(std::atof((*(words.begin()+2)).c_str()));
        ggaline.setLatorientation((*(words.begin()+3)).c_str()[0]);
        ggaline.setLon(std::atof((*(words.begin()+4)).c_str()));
        ggaline.setLonorientation((*(words.begin()+5)).c_str()[0]);
        ggaline.setFix(std::stoi(*(words.begin()+6)));
        ggaline.setNumSats(std::stoi(*(words.begin()+7)));
        ggaline.setDilution(std::atof((*(words.begin()+8)).c_str()));
        ggaline.setAltitude(std::atof((*(words.begin()+9)).c_str()));

        // set line to sent to NTRIP server
        this->ntripnmealine = fullnmealine.substr(0, fullnmealine.size()-1) + "\r\n";

        // print data
        std::cout.precision(10);
        std::cout << "GGA line * Latitude: " << ggaline.getLat() << " - " << "Longitude: " << ggaline.getLon() << "\t--\t Fix: " << ggaline.getFix() << std::endl;
    } else if (*words.begin() == "$GNRMC") {
        RMCnmealine rmcline = RMCnmealine();
        rmcline.setFix_taken_time(std::stoi(*(words.begin()+1)));
        rmcline.setStatus((*(words.begin()+2)).c_str()[0]);
        rmcline.setLat(std::atof((*(words.begin()+3)).c_str()));
        rmcline.setLatorientation((*(words.begin()+4)).c_str()[0]);
        rmcline.setLon(std::atof((*(words.begin()+5)).c_str()));
        rmcline.setLonorientation((*(words.begin()+6)).c_str()[0]);
        rmcline.setSpeed(std::stod(*(words.begin()+7)));
        rmcline.setAngle_deg(std::atof((*(words.begin()+8)).c_str()));
        rmcline.setDate(std::stoi(*(words.begin()+9)));
        rmcline.setMagneticvar((*(words.begin()+10)).c_str()[0]);
        rmcline.setDirmagneticvar((*(words.begin()+11)).c_str()[0]);
    }
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

std::string Simplertk2b::getNtripnmealine() {
    return this->ntripnmealine;
}

void serialCallback(Simplertk2b* simplertk2b) {
    nmealine nmealine;
    int ret;

    while (1) {
        ret = readLineSerialPort(simplertk2b->getSerialPort(), &nmealine);
        if (ret == 0) { // SUCCESS
            // Process incoming line
            simplertk2b->processNMEAline(nmealine.line, nmealine.fullline);
            
            // NTRIP 
            if (simplertk2b->isNtripActive()) {
                auto curtime = std::chrono::system_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(curtime - simplertk2b->getStartTime()).count() > simplertk2b->getNTRIPdelay()) { // send gga string @ 0.1 Hz
                    std::cout << "String to sent to ntrip server: " << simplertk2b->getNtripnmealine() << "A" << std::endl;
                    std::cout << "size is: " << simplertk2b->getNtripnmealine().length() << std::endl;
                    if (sendGGA(simplertk2b->getSockfd(), simplertk2b->getNtripnmealine().c_str(), simplertk2b->getNtripnmealine().length()) == 0) {                      
                        std::cout << "GGA string sent!\r\n" << std::endl;
                        simplertk2b->setStartTime(curtime);
                        if (!simplertk2b->getFirstNTRIPsent()) {
                            simplertk2b->setNTRIPdelay(20);
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