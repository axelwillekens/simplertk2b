#include "simplertk2bsystem.h"

Simplertk2bPublisher::Simplertk2bPublisher(std::string framegps_left, std::string framegps_right)
: Node("Simplertk2b"), zone(31), frame_gps({framegps_left, framegps_right}), X1(0), Y1(0), Z1(0), X2(0), Y2(0), Z2(0), yaw(0), roll(0)
{
    pos_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pos_topic", 1000);
    vel_pub = this-> create_publisher<geometry_msgs::msg::Vector3Stamped>("vel_topic", 1000);
    timer_ = this->create_wall_timer(200ms, std::bind(&Simplertk2bPublisher::timer_callback, this));
}

void Simplertk2bPublisher::timer_callback() {
    // ** pos_topic **
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    // Header
    std::string frame = "gps_frame";
    msg.header.frame_id = frame.c_str();
    msg.header.stamp = rclcpp::Clock().now();

    msg.pose.pose.position.x = (X1+X2)/2;
    msg.pose.pose.position.y = (Y1+Y2)/2;
    msg.pose.pose.position.z = (Z1+Z2)/2;

    // Calc euler angles and their covariances
    double num = (X2-X1);
    double denum = (Y2-Y1);
    if (num != 0) {
        if (num > 0) { 
            if (denum >= 0) yaw = atan(denum/num); // 1e kwadrant 
            else yaw = atan(denum/num) + (2*M_PI); // 4e kwadrant
        } else { // num < 0: 2e kwadrant & 3e kwadrant
            yaw = atan(denum/num) + M_PI;
        } 
    } else { // num == 0
        if (denum >= 0) yaw = M_PI; // 1e kwadrant 
        else yaw = (3/2)*M_PI; // 4e kwadrant 
    }
    msg.pose.pose.orientation.z = yaw; 

    double b = pow((X2-X1), 2) + pow((Y2-Y1), 2);
    roll = atan2((Z2-Z1), sqrt(b));
    msg.pose.pose.orientation.x = roll;
    
    // Publish data
    pos_pub->publish(msg);

    // ** vel_topic **
    // vel_topic is published when ready.
}

void Simplertk2bPublisher::processGGAline(GGAnmealine nmealine, std::string frame_id) {
    if(nmealine.getFix() == 4 || nmealine.getFix() == 5) { // Only if GPS is fixed or float!
        FLOAT x,y;
        FLOAT lat, lon;
        if (nmealine.getLatorientation() == 'N') lat = nmealine.getLat();
        else lat = - nmealine.getLat();
        if (nmealine.getLonorientation() == 'E') lon = nmealine.getLon();
        else lon = - nmealine.getLon();

        LatLonToUTMXY(lat, lon, zone, x, y);

        // store last x,y,z
        if (frame_id.compare(frame_gps[0]) == 0) {
            X1 = x; 
            Y1 = y; 
            Z1 = nmealine.getAltitude();
        } else {
            X2 = x; 
            Y2 = y; 
            Z2 = nmealine.getAltitude();
        }
    }
}

void Simplertk2bPublisher::processRMCline(RMCnmealine nmealine, std::string frame_id) {
    // std::cout << nmealine << std::endl;
    geometry_msgs::msg::Vector3Stamped msg;
    // Header
    std::string frame = "gps_frame";
    msg.header.frame_id = frame.c_str();
    msg.header.stamp = rclcpp::Clock().now();

    // Body
    msg.vector.x = 0.514444444 * nmealine.getSpeed() * cos(yaw);
    msg.vector.y = 0.514444444 * nmealine.getSpeed() * sin(yaw);
    msg.vector.z = 0;   

    // Publish data
    vel_pub->publish(msg);
}

std::string Simplertk2bPublisher::getFrame_gps(int idx) {
    return frame_gps[idx];
}
