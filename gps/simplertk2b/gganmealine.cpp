#include "gganmealine.h"

GGAnmealine::GGAnmealine() {}

GGAnmealine::~GGAnmealine() {}

int GGAnmealine::getFix() { return this->fix; }
int GGAnmealine::getNumSats() { return this->numSats; }
double GGAnmealine::getDilution() { return this->dilution; }
double GGAnmealine::getAltitude() { return this->altitude; }

void GGAnmealine::setFix(int fix) { this->fix = fix; }  
void GGAnmealine::setNumSats(int numSats) { this->numSats = numSats; }  
void GGAnmealine::setDilution(double dilution) { this->dilution = dilution; }  
void GGAnmealine::setAltitude(double altitude) { this->altitude = altitude; }  