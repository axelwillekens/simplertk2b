#include "gps/simplertk2b/simplertk2b.h"
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>

#define MAX_SEQ 2^32

void ggaNMEAcallback(GGAnmealine&);
void rmcNMEAcallback(RMCnmealine&);

ros::Publisher gga_pub;
ros::Publisher rmc_pub;

u_int32_t gga_seq;
u_int32_t rmc_seq;

int main(int argc, char **argv) {

    ros::init(argc, argv, "gps1");
    ros::NodeHandle n;
    gga_pub = n.advertise<sensor_msgs::NavSatFix>("gga_pub", 1000);
    rmc_pub = n.advertise<geometry_msgs::Vector3Stamped>("rmc_pub", 1000);

    Simplertk2b simplertk2b("/dev/ttyACM0", "flepos.vlaanderen.be", "FLEPOSVRS32GREC", "852a009", "97115");
    simplertk2b.setGGAcallback(ggaNMEAcallback);
    simplertk2b.setRMCcallback(rmcNMEAcallback);

    gga_seq = 0;
    rmc_seq = 0;
}

void ggaNMEAcallback(GGAnmealine& nmealine) {
    // std::cout << nmealine << std::endl;
    sensor_msgs::NavSatFix msg;
    // Header
    msg.header.frame_id = "gps1_base";
    msg.header.seq = gga_seq;
    msg.header.stamp = ros::Time::now();

    // Body
    msg.status.status = nmealine.getFix();

    if (nmealine.getLatorientation() == 'N') msg.latitude = nmealine.getLat();
    else msg.latitude = - nmealine.getLat();

    if (nmealine.getLonorientation() == 'E') msg.longitude = nmealine.getLon();
    else msg.longitude = nmealine.getLon();

    msg.altitude = nmealine.getAltitude();

    // msg.position_covariance = 
    // msg.position_covariance_type = 

    // Publish data
    gga_pub.publish(msg);
    gga_seq = (gga_seq + 1) % MAX_SEQ;
}

void rmcNMEAcallback(RMCnmealine& nmealine) {
    // std::cout << nmealine << std::endl;
    geometry_msgs::Vector3Stamped msg;
    // Header
    msg.header.frame_id = "gps1_base";
    msg.header.seq = rmc_seq;
    msg.header.stamp = ros::Time::now();

    // Body
    msg.vector.x = nmealine.getSpeed() * cos(nmealine.getAngle_deg() * M_PI/180);
    msg.vector.y = nmealine.getSpeed() * sin(nmealine.getAngle_deg() * M_PI/180);
    msg.vector.z = 0;   

    // Publish data
    rmc_pub.publish(msg);
    rmc_seq = (rmc_seq + 1) % MAX_SEQ;
}

