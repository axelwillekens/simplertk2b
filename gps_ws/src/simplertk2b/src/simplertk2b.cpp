#include "gps/simplertk2b/simplertk2b.h"
#include "gps/utm/UTM.h"
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>

#define MAX_SEQ 2^32

void ggaNMEAcallback(GGAnmealine&);
void rmcNMEAcallback(RMCnmealine&);

void publishGGAline(GGAnmealine, std::string, u_int32_t&);
void publishRMCline(RMCnmealine, std::string, u_int32_t&);

ros::Publisher gga_pub;
ros::Publisher rmc_pub;

std::string frame_gps;
int zone;

u_int32_t gga_seq;
u_int32_t rmc_seq;

int main(int argc, char **argv) {

    ros::init(argc, argv, "gps_system");
    ros::NodeHandle n("~");

    std::string port_gps;
    std::string server;
    std::string mountpoint;
    std::string username;
    std::string pwd;

    n.param<std::string>("port_gps", port_gps, "/dev/ttyACM0");
    n.param<std::string>("server", server, "flepos.vlaanderen.be");
    n.param<std::string>("mountpoint", mountpoint, "FLEPOSVRS32GREC");
    n.param<std::string>("username", username, "852a009");
    n.param<std::string>("pwd", pwd, "97115");
    n.param<std::string>("frame_gps", frame_gps, "gps_frame");
    n.param<int>("zone", zone, 31);

    gga_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("gga_pub", 1000);
    rmc_pub = n.advertise<geometry_msgs::Vector3Stamped>("rmc_pub", 1000);

    std::cout << "Portname is: " << port_gps << std::endl;
    Simplertk2b simplertk2b(port_gps, server, mountpoint, username, pwd);
    simplertk2b.setGGAcallback(ggaNMEAcallback);
    simplertk2b.setRMCcallback(rmcNMEAcallback);

    gga_seq = 0;
    rmc_seq = 0;
}

void ggaNMEAcallback(GGAnmealine& nmealine) {
    publishGGAline(nmealine, frame_gps, gga_seq);
}

void rmcNMEAcallback(RMCnmealine& nmealine) {
    publishRMCline(nmealine, frame_gps, rmc_seq);
}

void publishGGAline(GGAnmealine nmealine, std::string frame_id, u_int32_t& sequencer) {
    if(nmealine.getFix() == 4 || nmealine.getFix() == 5) { // Only if GPS is fixed or float!
        // std::cout << nmealine << std::endl;
        geometry_msgs::PoseWithCovarianceStamped msg;
        // Header
        msg.header.frame_id = frame_id.c_str();
        msg.header.seq = sequencer;
        msg.header.stamp = ros::Time::now();

        // Body
        // Store the euler angles in the quaternion x,y,z and the gps fix in the quaternion w
        msg.pose.pose.orientation.w = nmealine.getFix();

        FLOAT x,y;
        FLOAT lat, lon;
        if (nmealine.getLatorientation() == 'N') lat = nmealine.getLat();
        else lat = - nmealine.getLat();
        if (nmealine.getLonorientation() == 'E') lon = nmealine.getLon();
        else lon = - nmealine.getLon();

        LatLonToUTMXY(lat, lon, zone, x, y);

        msg.pose.pose.position.x = x;
        msg.pose.pose.position.y = y;
        msg.pose.pose.position.z = nmealine.getAltitude();

        double delta;
        if (nmealine.getFix() == 4) { // RTK FIX: accuracy < 1cm (1 SIGMA)
            delta = 0.01;
        } else if(nmealine.getFix() == 5) { // RTK FLOAT: accuracy < 10cm (1 SIGMA)
            delta = 0.1;
        } 
        msg.pose.pose.orientation.w = nmealine.getFix();

        // Fill in covariance matrix
        boost::array<double,36UL> cov_matrix = {0};
        cov_matrix[0] = pow(delta,2);
        cov_matrix[7] = pow(delta,2);
        cov_matrix[14] = pow(delta,2);

        msg.pose.covariance = cov_matrix;
        
        // Publish data
        gga_pub.publish(msg);
        sequencer = (sequencer + 1) % MAX_SEQ;
    }
}

void publishRMCline(RMCnmealine nmealine, std::string frame_id, u_int32_t& sequencer) {
    // std::cout << nmealine << std::endl;
    geometry_msgs::Vector3Stamped msg;
    // Header
    msg.header.frame_id = frame_id.c_str();
    msg.header.seq = sequencer;
    msg.header.stamp = ros::Time::now();

    // Body
    msg.vector.x = nmealine.getSpeed() * cos(nmealine.getAngle_deg() * M_PI/180);
    msg.vector.y = nmealine.getSpeed() * sin(nmealine.getAngle_deg() * M_PI/180);
    msg.vector.z = 0;   

    // Publish data
    rmc_pub.publish(msg);
    sequencer = (sequencer + 1) % MAX_SEQ;
}

