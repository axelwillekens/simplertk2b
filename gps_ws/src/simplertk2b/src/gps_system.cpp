#include "gps/utm/UTM.h"
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>

double X1,Y1,Z1;
double X2,Y2,Z2;


int main(int* argc, int** argv) {
        // Calc euler angles and their covariances
        // msg.pose.pose.orientation.z = atan2((Y2-Y1),(X2-X1));
        // double b = pow((X2-X1), 2) + pow((Y2-Y1), 2);
        // msg.pose.pose.orientation.x = atan2((Z2-Z1), sqrt(b));

        // cov_matrix[21] = pow( 1/(1 + pow((Z2-Z1)/sqrt(b), 2)) , 2) * ( pow(b,-1) * (2*pow(delta, 2)) + pow(b,-3) * (pow((2*(X2-X1)), 2) * (2*pow(delta, 2)) + pow((2*(Y2-Y1)), 2) * (2*pow(delta, 2)) ) );
        // cov_matrix[35] = pow( 1/(1 + pow((Y2-Y1)/(X2-X1), 2)) , 2) * (2* pow( (Y2-Y1)*delta/pow(X2-X1, 2) , 2) + 2 * pow( delta/(X2-X1) , 2) );

        // store last x,y,z
        // if (frame_id.compare(frame_gps1) == 0) {
        //     X1 = x; 
        //     Y1 = y; 
        //     Z1 = nmealine.getAltitude();
        // } else {
        //     X2 = x; 
        //     Y2 = y; 
        //     Z2 = nmealine.getAltitude();
        // }
}