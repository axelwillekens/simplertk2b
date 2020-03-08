#include "nmealine.h"

Nmealine::Nmealine() {}
Nmealine::~Nmealine() {}

int Nmealine::getFix_taken_time() { return this->fix_taken_time; }  
double Nmealine::getLat() { return this->lat; }
char Nmealine::getLatorientation() { return this->latorientation; }
double Nmealine::getLon() { return this->lon; }
char Nmealine::getLonorientation() { return this->lonorientation; }

void Nmealine::setFix_taken_time(int fix_taken_time) { this->fix_taken_time = fix_taken_time; }  
void Nmealine::setLat(double lat) { this->lat = lat; }   
void Nmealine::setLatorientation(char latorientation) { this->latorientation = latorientation; }  
void Nmealine::setLon(double lon) { this->lon = lon; }  
void Nmealine::setLonorientation(char lonorientation) { this->lonorientation = lonorientation; }  

