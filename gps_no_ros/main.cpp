#include "gps/simplertk2b/simplertk2b.h"

#define MAX_SEQ 2^32

void ggaNMEAcallback(GGAnmealine&, int);
void rmcNMEAcallback(RMCnmealine&, int);

int main(int argc, char **argv) {
    Simplertk2b simplertk2b("/dev/ttyACM0", "/dev/ttyACM1", "flepos.vlaanderen.be", "FLEPOSVRS32GREC", "852a009", "97115");
    simplertk2b.setGGAcallback(ggaNMEAcallback);
    simplertk2b.setRMCcallback(rmcNMEAcallback);
}

void ggaNMEAcallback(GGAnmealine& nmealine, int index) {
    std::cout << "From index: " << index << "\t" << nmealine << std::endl;
}

void rmcNMEAcallback(RMCnmealine& nmealine, int index) {
    std::cout << "From index: " << index << "\t" << nmealine << std::endl;
}

