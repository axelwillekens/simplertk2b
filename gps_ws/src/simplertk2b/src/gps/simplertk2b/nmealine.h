#ifndef NMEALINE_H
#define NMEALINE_H

#include <string>
#include <iostream>

class Nmealine
{
private:
    int fix_taken_time;
    double lat; char latorientation;
    double lon; char lonorientation;
public:
    Nmealine();
    ~Nmealine();

    int getFix_taken_time() const;   
    double getLat() const; 
    char getLatorientation() const;
    double getLon() const; 
    char getLonorientation() const; 

    void setFix_taken_time(int);   
    void setLat(double); 
    void setLatorientation(char);
    void setLon(double); 
    void setLonorientation(char); 
};


#endif // MEALINE_H