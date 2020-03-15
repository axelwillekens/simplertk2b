#include "gps/simplertk2b/simplertk2b.h"
#include "gps/utm/UTM.h"
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>

#define MAX_SEQ 2^32

void ggaNMEAcallback_front(GGAnmealine&);
void rmcNMEAcallback_front(RMCnmealine&);
void ggaNMEAcallback_back(GGAnmealine&);
void rmcNMEAcallback_back(RMCnmealine&);

void publishGGAline(GGAnmealine, std::string, u_int32_t&);
void publishRMCline(RMCnmealine, std::string, u_int32_t&);

ros::Publisher gga_pub;
ros::Publisher rmc_pub;

std::string frame_gps1;
std::string frame_gps2;
int zone;

u_int32_t gga_seq_front;
u_int32_t rmc_seq_front;
u_int32_t gga_seq_back;
u_int32_t rmc_seq_back;

int main(int argc, char **argv) {

    ros::init(argc, argv, "gps_system");
    ros::NodeHandle n;

    std::string port_gps1;
    std::string port_gps2;
    std::string server;
    std::string mountpoint;
    std::string username;
    std::string pwd;

    n.param<std::string>("port_gps1", port_gps1, "/dev/ttyACM0");
    n.param<std::string>("port_gps2", port_gps2, "/dev/ttyACM1");
    n.param<std::string>("server", server, "flepos.vlaanderen.be");
    n.param<std::string>("mountpoint", mountpoint, "FLEPOSVRS32GREC");
    n.param<std::string>("username", username, "852a009");
    n.param<std::string>("pwd", pwd, "97115");
    n.param<std::string>("frame_gps1", frame_gps1, "gps_front_frame");
    n.param<std::string>("frame_gps2", frame_gps2, "gps_back_frame");
    n.param<int>("zone", zone, 31);

    gga_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("gga_pub", 1000);
    rmc_pub = n.advertise<geometry_msgs::Vector3Stamped>("rmc_pub", 1000);

    Simplertk2b simplertk2b_front(port_gps1, server, mountpoint, username, pwd);
    simplertk2b_front.setGGAcallback(ggaNMEAcallback_front);
    simplertk2b_front.setRMCcallback(rmcNMEAcallback_front);

    // Simplertk2b simplertk2b_back(port_gps2, server, mountpoint, username, pwd);
    // simplertk2b_back.setGGAcallback(ggaNMEAcallback_back);
    // simplertk2b_back.setRMCcallback(rmcNMEAcallback_back);

    gga_seq_front = 0;
    rmc_seq_front = 0;
    gga_seq_back = 0;
    rmc_seq_back = 0;
}

void ggaNMEAcallback_front(GGAnmealine& nmealine) {
    publishGGAline(nmealine, "frame_gps1", gga_seq_front);
}

void rmcNMEAcallback_front(RMCnmealine& nmealine) {
    publishRMCline(nmealine, "frame_gps1", rmc_seq_front);
}

void ggaNMEAcallback_back(GGAnmealine& nmealine) {
    publishGGAline(nmealine, "frame_gps2", gga_seq_front);
}

void rmcNMEAcallback_back(RMCnmealine& nmealine) {
    publishRMCline(nmealine, "frame_gps2", rmc_seq_front);
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
        if (nmealine.getLatorientation() == 'N') lat = nmealine.getLat() / 100;
        else lat = - nmealine.getLat() / 100;
        if (nmealine.getLonorientation() == 'E') lon = nmealine.getLon() / 100;
        else lon = nmealine.getLon() / 100;

        LatLonToUTMXY(lat, lon, zone, x, y);

        msg.pose.pose.position.x = x;
        msg.pose.pose.position.y = y;
        msg.pose.pose.position.z = nmealine.getAltitude();

        // Write code for euler angles

        boost::array<double,36UL> cov_matrix = {0};
        if (nmealine.getFix() == 4) { // RTK FIX
            cov_matrix[0] = std::pow(0.01,2);
            cov_matrix[7] = std::pow(0.01,4);
            cov_matrix[14] = std::pow(0.01,8);
        } else if(nmealine.getFix() == 5) { // RTK FLOAT
            cov_matrix[0] = std::pow(1,2);
            cov_matrix[7] = std::pow(1,2);
            cov_matrix[14] = std::pow(1,2);
        } 
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

