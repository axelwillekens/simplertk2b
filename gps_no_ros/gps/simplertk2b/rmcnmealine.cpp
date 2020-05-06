#include "rmcnmealine.h"

RMCnmealine::RMCnmealine() {}
RMCnmealine::~RMCnmealine() {}

char RMCnmealine::getStatus() const { return this->status; }
double RMCnmealine::getSpeed() const { return this->speed; }
double RMCnmealine::getAngle_deg() const { return this->angle_deg; }
int RMCnmealine::getDate() const { return this->date; }
double RMCnmealine::getMagneticvar() const { return this->magneticvar; }
char RMCnmealine::getDirmagneticvar() const { return this->dirmagneticvar; }

void RMCnmealine::setStatus(char status) { this->status = status; }
void RMCnmealine::setSpeed(double speed) { this->speed = speed; }           
void RMCnmealine::setAngle_deg(double angle_deg) { this->angle_deg = angle_deg; }
void RMCnmealine::setDate(int date) { this->date = date; }
void RMCnmealine::setMagneticvar(double magneticvar) { this->magneticvar = magneticvar; }
void RMCnmealine::setDirmagneticvar(char dirmagneticvar) { this->dirmagneticvar = dirmagneticvar; }

std::ostream& operator<<(std::ostream& outputStream, const RMCnmealine& rmcline) {
    outputStream.precision(10);
    outputStream << "RMC line * Latitude: " << rmcline.getLat() << " - " << "Longitude: " << rmcline.getLon() << "\t--\t Speed: " << rmcline.getSpeed();
    return outputStream;
}