#include "simplertk2bsystem/simplertk2bsystem.h"

Simplertk2bPublisher::Simplertk2bPublisher(std::string framegps_left, std::string framegps_right)
: Node("Simplertk2b"), zone(31), frame_gps({framegps_left, framegps_right}), X1(0), Y1(0), Z1(0), X2(0), Y2(0), Z2(0)
{
    gga_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("gga_pub", 1000);
    rmc_pub = this-> create_publisher<geometry_msgs::msg::Vector3Stamped>("rmc_pub", 1000);
}

void Simplertk2bPublisher::publishGGAline(GGAnmealine nmealine, std::string frame_id) {
    if(nmealine.getFix() == 4 || nmealine.getFix() == 5) { // Only if GPS is fixed or float!
        // std::cout << nmealine << std::endl;
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        // Header
        msg.header.frame_id = frame_id.c_str();
        msg.header.stamp = rclcpp::Clock().now();

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

        // Set sigma for delta
        double delta;
        if (nmealine.getFix() == 4) { // RTK FIX: accuracy < 1cm (1 SIGMA)
            delta = 0.01;
        } else if(nmealine.getFix() == 5) { // RTK FLOAT: accuracy < 10cm (1 SIGMA)
            delta = 0.1;
        } 
        msg.pose.pose.orientation.w = nmealine.getFix();

        // Calc euler angles and their covariances
        msg.pose.pose.orientation.z = atan2((Y2-Y1),(X2-X1));
        double b = pow((X2-X1), 2) + pow((Y2-Y1), 2);
        msg.pose.pose.orientation.x = atan2((Z2-Z1), sqrt(b));

        // Fill in covariance matrix
        std::array<double,36UL> cov_matrix = {0};
        cov_matrix[0] = pow(delta,2);
        cov_matrix[7] = pow(delta,2);
        cov_matrix[14] = pow(delta,2);
        cov_matrix[21] = pow( 1/(1 + pow((Z2-Z1)/sqrt(b), 2)) , 2) * ( pow(b,-1) * (2*pow(delta, 2)) + pow(b,-3) * (pow((2*(X2-X1)), 2) * (2*pow(delta, 2)) + pow((2*(Y2-Y1)), 2) * (2*pow(delta, 2)) ) );
        cov_matrix[35] = pow( 1/(1 + pow((Y2-Y1)/(X2-X1), 2)) , 2) * (2* pow( (Y2-Y1)*delta/pow(X2-X1, 2) , 2) + 2 * pow( delta/(X2-X1) , 2) );
        
        msg.pose.covariance = cov_matrix;
        
        // Publish data
        gga_pub->publish(msg);
    }
}

void Simplertk2bPublisher::publishRMCline(RMCnmealine nmealine, std::string frame_id) {
    // std::cout << nmealine << std::endl;
    geometry_msgs::msg::Vector3Stamped msg;
    // Header
    msg.header.frame_id = frame_id.c_str();
    msg.header.stamp = rclcpp::Clock().now();

    // Body
    msg.vector.x = 0.514444444 * nmealine.getSpeed() * cos(DegToRad(nmealine.getAngle_deg()));
    msg.vector.y = 0.514444444 * nmealine.getSpeed() * sin(DegToRad(nmealine.getAngle_deg()));
    msg.vector.z = 0;   

    // Publish data
    rmc_pub->publish(msg);
}

std::string Simplertk2bPublisher::getFrame_gps(int idx) {
    return frame_gps[idx];
}