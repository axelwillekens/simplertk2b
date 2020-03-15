#ifndef GGANMEALINE_H
#define GGANMEALINE_H

#include <string>
#include "nmealine.h"

class GGAnmealine : public Nmealine
{
private:
                        // GGA          Global Positioning System Fix Data
                        // 123519       Fix taken at 12:35:19 UTC
                        // 4807.038,N   Latitude 48 deg 07.038' N
                        // 01131.000,E  Longitude 11 deg 31.000' E
    int fix;            //      1            Fix quality: 0 = invalid
                        //                1 = GPS fix (SPS)
                        //                2 = DGPS fix
                        //                3 = PPS fix
                        //                4 = Real Time Kinematic
                        //                5 = Float RTK
                        //                6 = estimated (dead reckoning) (2.3 feature)
                        //                7 = Manual input mode
                        //                8 = Simulation mode
    int numSats;        // 08           Number of satellites being tracked
    double dilution;    // 0.9          Horizontal dilution of position
    double altitude;    // 545.4,M      Altitude, Meters, above mean sea level
                        // 46.9,M       Height of geoid (mean sea level) above WGS84
                        //                   ellipsoid
                        // (empty field) time in seconds since last DGPS update
                        // (empty field) DGPS station ID number
                        // *47          the checksum data, always begins with *
public:
    GGAnmealine();
    ~GGAnmealine();

    int getFix() const;                        
    int getNumSats() const;               
    double getDilution() const;
    double getAltitude() const;

    void setFix(int);                        
    void setNumSats(int);               
    void setDilution(double);
    void setAltitude(double);
};

std::ostream& operator<<(std::ostream& outputStream, const GGAnmealine& ggaline);

#endif // GGANMEALINE_H