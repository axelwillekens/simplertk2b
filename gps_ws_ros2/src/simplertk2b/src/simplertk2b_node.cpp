#include <rclcpp/rclcpp.hpp>
#include "simplertk2b/simplertk2bsystem.h"

std::shared_ptr<Simplertk2bPublisher> simplertk2bpub;

void ggaNMEAcallback(GGAnmealine&,int);
void rmcNMEAcallback(RMCnmealine&,int);

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    simplertk2bpub = std::make_shared<Simplertk2bPublisher>("gps_left", "gps_right");
    Simplertk2b simplertk2b("/dev/ttyACM0", "/dev/ttyACM1", "flepos.vlaanderen.be", "FLEPOSVRS32GREC", "852a009", "97115");
    simplertk2b.setGGAcallback(ggaNMEAcallback);
    simplertk2b.setRMCcallback(rmcNMEAcallback);

    rclcpp::spin(simplertk2bpub);
    rclcpp::shutdown();
    return 0;
}


void ggaNMEAcallback(GGAnmealine& nmealine, int index) {
    simplertk2bpub->processGGAline(nmealine, simplertk2bpub->getFrame_gps(index));
}

void rmcNMEAcallback(RMCnmealine& nmealine, int index) {
    simplertk2bpub->processRMCline(nmealine, simplertk2bpub->getFrame_gps(index));
}