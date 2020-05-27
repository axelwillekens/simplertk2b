#ifndef SIMPLERTK2B_SYSTEM_H
#define SIMPLERTK2B_SYSTEM_H

#define MAX_SEQ 2^32


#include "gps/simplertk2b/simplertk2b.h"
#include "utm/UTM.h"
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <math.h>

#include <rclcpp/rclcpp.hpp>



class Simplertk2bPublisher: public rclcpp::Node
{
    public:
        Simplertk2bPublisher(std::string framegps_left, std::string framegps_right);
        std::string getFrame_gps(int idx);

        void publishGGAline(GGAnmealine, std::string);
        void publishRMCline(RMCnmealine, std::string);

    private:
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gga_pub;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rmc_pub;
        
        int zone;
        std::string frame_gps[2];

        double X1,Y1,Z1;
        double X2,Y2,Z2;
};

#endif // SIMPLERTK2B_SYSTEM_H