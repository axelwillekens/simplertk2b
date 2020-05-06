#ifndef RMCNMEALINE_H
#define RMCNMEALINE_H

#include <string>
#include "nmealine.h"

class RMCnmealine : public Nmealine
{
private:
                                                //  RMC          Recommended Minimum sentence C
                                                //  123519       Fix taken at 12:35:19 UTC
    char status;                                //  A            Status A=active or V=Void.
                                                //  4807.038,N   Latitude 48 deg 07.038' N
                                                //  01131.000,E  Longitude 11 deg 31.000' E
    double speed;                               //  022.4        Speed over the ground in knots (1 knot = 0.514444444 m/s)
    double angle_deg;                           //  084.4        Track angle in degrees True
    int date;                                   //  230394       Date - 23rd of March 1994
    double magneticvar; char dirmagneticvar;    //  003.1,W      Magnetic Variation
                                                //  *6A          The checksum data, always begins with *
public:
    RMCnmealine();
    ~RMCnmealine();

    char getStatus() const;
    double getSpeed() const;              
    double getAngle_deg() const;
    int getDate() const;
    double getMagneticvar() const;
    char getDirmagneticvar() const;

    void setStatus(char);
    void setSpeed(double);              
    void setAngle_deg(double);
    void setDate(int);
    void setMagneticvar(double);
    void setDirmagneticvar(char);
};

std::ostream& operator<<(std::ostream& outputStream, const RMCnmealine& rmcline);

#endif // RMCNMEALINE_H