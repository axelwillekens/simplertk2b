#ifndef SIMPLERTK2B_H
#define SIMPLERTK2B_H

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

double X1,Y1,Z1;
double X2,Y2,Z2;

#endif