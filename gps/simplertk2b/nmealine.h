#ifndef NMEALINE_H
#define NMEALINE_H

#include <string>

class Nmealine
{
private:
    int fix_taken_time;
    double lat; char latorientation;
    double lon; char lonorientation;
public:
    Nmealine();
    ~Nmealine();

    int getFix_taken_time();   
    double getLat(); 
    char getLatorientation();
    double getLon(); 
    char getLonorientation(); 
    int getFix();                        
    int getNumSats();               
    double getDilution();
    double getAltitude();

    void setFix_taken_time(int);   
    void setLat(double); 
    void setLatorientation(char);
    void setLon(double); 
    void setLonorientation(char); 
    void setFix(int);                        
    void setNumSats(int);               
    void setDilution(double);
    void setAltitude(double);
};


#endif // MEALINE_H