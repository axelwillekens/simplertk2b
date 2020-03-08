#include "rmcnmealine.h"

RMCnmealine::RMCnmealine() {}
RMCnmealine::~RMCnmealine() {}

char RMCnmealine::getStatus() { return this->status; }
double RMCnmealine::getSpeed() { return this->speed; }
double RMCnmealine::getAngle_deg() { return this->angle_deg; }
int RMCnmealine::getDate() { return this->date; }
double RMCnmealine::getMagneticvar() { return this->magneticvar; }
char RMCnmealine::getDirmagneticvar() { return this->dirmagneticvar; }

void RMCnmealine::setStatus(char status) { this->status = status; }
void RMCnmealine::setSpeed(double speed) { this->speed = speed; }           
void RMCnmealine::setAngle_deg(double angle_deg) { this->angle_deg = angle_deg; }
void RMCnmealine::setDate(int date) { this->date = date; }
void RMCnmealine::setMagneticvar(double magneticvar) { this->magneticvar = magneticvar; }
void RMCnmealine::setDirmagneticvar(char dirmagneticvar) { this->dirmagneticvar = dirmagneticvar; }