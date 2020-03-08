#include "gganmealine.h"

GGAnmealine::GGAnmealine() {}

GGAnmealine::~GGAnmealine() {}

int GGAnmealine::getFix() const { return this->fix; }
int GGAnmealine::getNumSats() const { return this->numSats; }
double GGAnmealine::getDilution() const { return this->dilution; }
double GGAnmealine::getAltitude() const { return this->altitude; }

void GGAnmealine::setFix(int fix) { this->fix = fix; }  
void GGAnmealine::setNumSats(int numSats) { this->numSats = numSats; }  
void GGAnmealine::setDilution(double dilution) { this->dilution = dilution; }  
void GGAnmealine::setAltitude(double altitude) { this->altitude = altitude; }  

std::ostream& operator<<(std::ostream& outputStream, const GGAnmealine& ggaline) {
    outputStream.precision(10);
    outputStream << "GGA line * Latitude: " << ggaline.getLat() << " - " << "Longitude: " << ggaline.getLon() << "\t--\t Fix: " << ggaline.getFix();
    return outputStream;
}