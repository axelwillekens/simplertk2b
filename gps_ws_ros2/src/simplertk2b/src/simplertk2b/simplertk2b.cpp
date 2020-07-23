#include "simplertk2b.h"

Simplertk2b::Simplertk2b(std::string serialportnameMaster, std::string serialportnameSlave, std::string server, std::string mountpoint, std::string username, std::string passwd) : starttime(std::chrono::system_clock::now())
    , ntripActive(false), ntripdelay(4), firstntripsent(false), ggacallback(nullptr), rmccallback(nullptr) 
{
    // initializement
    serialportname[0] = serialportnameMaster;
    serialportname[1] = serialportnameSlave;

    // Init of NTRIP args
    args.server = server.c_str();
    args.mount = mountpoint.c_str();
    args.port = 2101;
    args.user = username.c_str();
    args.password = passwd.c_str();
    args.num_serial_ports = 2;
    args.serial_ports[0] = 0;
    args.serial_ports[1] = 0;

    if ((this->args.sockfd = connectNtrip(&args)) != -1) { 
        this->ntripActive = true;
        ntripThread = std::thread(socketWork,this->args.sockfd,&args);
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    } else {
        std::cerr << "Connection with NTRIP server failed! See above error for more info." << std::endl;
        std::cerr << "Check your internet connection (firewalls may be blocking the TCP connection." << std::endl;
        std::cerr << "Check if your NTRIP account settings are correct." << std::endl;
    }

    // Init Serial Port master
    for (int i=0; i < args.num_serial_ports; i++) {
        serialThread[i] = std::thread(serialWork, this, i);
    }
}

Simplertk2b::~Simplertk2b() {
    CloseSerialPort(this->args.serial_ports[0]);  
    CloseSerialPort(this->args.serial_ports[1]);  
    if (serialThread[0].joinable()) serialThread[0].join();
    if (serialThread[1].joinable()) serialThread[1].join();
    if (ntripThread.joinable()) ntripThread.join();  
}

void Simplertk2b::processNMEAline(int index, std::string nmealine, std::string fullnmealine) {
    std::vector<std::string> words;
    boost::split(words, nmealine, boost::is_any_of(","));

    std::vector<std::string>::iterator i;
    if (*words.begin() == "$GNGGA") {
        // avoid segmentation fault (memory access out of array bounds)
        if (words.size() != 15) {
            printf("GGA string is : %d long, this is not the right size, which is 15.\n", (int) words.size());
            return;
        }

        ggaline.setFix_taken_time(std::stoi(*(words.begin()+1)));
        std::string lat = (*(words.begin()+2)).c_str();
        try {
            std::string lat_deg = lat.substr(0,2);
            std::string lat_sec = lat.substr(2);
            ggaline.setLat(std::atof(lat_deg.c_str())+std::atof(lat_sec.c_str())/60);
        } catch(const std::out_of_range& e) {
            std::cerr << "cannot index GGA string correctly" << std::endl;
        } 
        ggaline.setLatorientation((*(words.begin()+3)).c_str()[0]);
        std::string lon = (*(words.begin()+4)).c_str();
        try {
            std::string lon_deg = lon.substr(0,3);
            std::string lon_sec = lon.substr(3);
            ggaline.setLon(std::atof(lon_deg.c_str())+std::atof(lon_sec.c_str())/60);
        } catch (const std::out_of_range& e) {
            std::cerr << "cannot index GGA string correctly" << std::endl;
        }
        ggaline.setLonorientation((*(words.begin()+5)).c_str()[0]);
        ggaline.setFix(std::stoi(*(words.begin()+6)));
        ggaline.setNumSats(std::stoi(*(words.begin()+7)));
        ggaline.setDilution(std::atof((*(words.begin()+8)).c_str()));
        ggaline.setAltitude(std::atof((*(words.begin()+9)).c_str()));

        // set line to sent to NTRIP server
        this->ntripnmealine = fullnmealine.substr(0, fullnmealine.size()-1) + "\r\n";

        // initiate callback
        if (ggacallback != nullptr) {
            ggacallback(ggaline, index);
        }
    } else if (*words.begin() == "$GNRMC") {
        // avoid segmentation fault (memory access out of array bounds)
        if (words.size() != 14) {
            printf("RMC string is : %d long, this is not the right size, which is 13.\n", (int) words.size());
            return;
        }

        rmcline.setFix_taken_time(std::stoi(*(words.begin()+1)));
        rmcline.setStatus((*(words.begin()+2)).c_str()[0]);
        std::string lat = (*(words.begin()+3)).c_str();
        try {
            std::string lat_deg = lat.substr(0,2);
            std::string lat_sec = lat.substr(2);
            rmcline.setLat(std::atof(lat_deg.c_str())+std::atof(lat_sec.c_str())/60);
        } catch (const std::out_of_range& e) {
            std::cerr << "cannot index RMC string correctly" << std::endl;
        }
        rmcline.setLatorientation((*(words.begin()+4)).c_str()[0]);
        std::string lon = (*(words.begin()+5)).c_str();
        try {
            std::string lon_deg = lon.substr(0,3);
            std::string lon_sec = lon.substr(3);
            rmcline.setLon(std::atof(lon_deg.c_str())+std::atof(lon_sec.c_str())/60);
        } catch (const std::out_of_range& e) {
            std::cerr << "cannot index RMC string correctly" << std::endl;
        }
        rmcline.setLonorientation((*(words.begin()+6)).c_str()[0]);
        rmcline.setSpeed(std::atof((*(words.begin()+7)).c_str()));
        rmcline.setAngle_deg(std::atof((*(words.begin()+8)).c_str()));
        rmcline.setDate(std::stoi(*(words.begin()+9)));
        rmcline.setMagneticvar((*(words.begin()+10)).c_str()[0]);
        rmcline.setDirmagneticvar((*(words.begin()+11)).c_str()[0]);

        // initiate callback
        if (rmccallback != nullptr) {
            rmccallback(rmcline, index);
        }
    }
}

bool Simplertk2b::isNtripActive() { return this->ntripActive; }

std::chrono::system_clock::time_point Simplertk2b::getStartTime() { return starttime; }
void Simplertk2b::setStartTime(std::chrono::system_clock::time_point starttime) { this->starttime = starttime; }

int Simplertk2b::getSerialPort(int index) { return this->args.serial_ports[index]; }
void Simplertk2b::setSerialPort(int index, int serial_port) { this->args.serial_ports[index] = serial_port; }

int Simplertk2b::getSockfd() { return this->args.sockfd; }
std::string Simplertk2b::getPortName(int index) { return this->serialportname[index]; }

int Simplertk2b::getNTRIPdelay() { return this->ntripdelay; }
void Simplertk2b::setNTRIPdelay(int ntripdelay) { this->ntripdelay = ntripdelay; }


bool Simplertk2b::getFirstNTRIPsent() { return this->firstntripsent; }
void Simplertk2b::setFirstNTRIPsent(bool firstntripsent) { this->firstntripsent = firstntripsent; }

std::string Simplertk2b::getNtripnmealine() { return this->ntripnmealine; }

void Simplertk2b::setGGAcallback(void (*ggacallback)(GGAnmealine&, int)) { this->ggacallback = ggacallback; }
void Simplertk2b::setRMCcallback(void (*rmccallback)(RMCnmealine&, int)) { this->rmccallback = rmccallback; }

void serialWork(Simplertk2b* simplertk2b, int index) {
    // create serial port 
    int serialport = initSerialComm(simplertk2b->getPortName(index).c_str());
    if (serialport == -1) {
        std::cerr << "Cannot make a connection on this port" << std::endl;
        std::cerr << "Check if your serial port is correct" << std::endl;
    } else {
        simplertk2b->setSerialPort(index, serialport);
    }

    // Start the actual loop
    nmealine nmealine;
    int ret;

    while (1) {
        ret = readLineSerialPort(simplertk2b->getSerialPort(index), &nmealine);
        if (ret == 0) { // SUCCESS
            // Process incoming line
            simplertk2b->processNMEAline(index, nmealine.line, nmealine.fullline);
            
            // NTRIP 
            if (simplertk2b->isNtripActive()) {
                auto curtime = std::chrono::system_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(curtime - simplertk2b->getStartTime()).count() > simplertk2b->getNTRIPdelay()) { // send gga string @ 0.1 Hz
                    if (sendGGA(simplertk2b->getSockfd(), simplertk2b->getNtripnmealine().c_str(), simplertk2b->getNtripnmealine().length()) == 0) {                      
                        std::cout << "GGA string sent to ntrip server: " << simplertk2b->getNtripnmealine() << std::endl;
                        simplertk2b->setStartTime(curtime);
                        if (!simplertk2b->getFirstNTRIPsent()) {
                            simplertk2b->setNTRIPdelay(20);
                            simplertk2b->setFirstNTRIPsent(false);
                        }
                    }
                }
            }
        } else if (ret == -1) { // Init Serial port again!
            std::cerr << "File descriptor failed!" << std::endl;
        }
    }
    
}