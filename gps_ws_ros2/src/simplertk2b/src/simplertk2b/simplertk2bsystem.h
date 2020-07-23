#ifndef SIMPLERTK2B_SYSTEM_H
#define SIMPLERTK2B_SYSTEM_H

#define MAX_SEQ 2^32


#include "../simplertk2b/simplertk2b.h"
#include "../utm/UTM.h"
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <math.h>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class Simplertk2bPublisher: public rclcpp::Node
{
    private:
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pos_pub;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr vel_pub;

        int zone;
        std::string frame_gps[2];

        double X1,Y1,Z1;
        double X2,Y2,Z2;

    public:
        Simplertk2bPublisher(std::string framegps_left, std::string framegps_right);
        std::string getFrame_gps(int idx);
        void timer_callback();

        void processGGAline(GGAnmealine, std::string);
        void processRMCline(RMCnmealine, std::string);
        rclcpp::TimerBase::SharedPtr timer_;
};

#endif // SIMPLERTK2B_SYSTEM_H