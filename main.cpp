#include "gps/simplertk2b/simplertk2b.h"

void ggaNMEAcallback(GGAnmealine&);
void rmcNMEAcallback(RMCnmealine&);

int main() {
    Simplertk2b simplertk2b("/dev/ttyACM0", "flepos.vlaanderen.be", "FLEPOSVRS32GREC", "852a009", "97115");
    simplertk2b.setGGAcallback(ggaNMEAcallback);
    simplertk2b.setRMCcallback(rmcNMEAcallback);
}

void ggaNMEAcallback(GGAnmealine& nmealine) {
    std::cout << nmealine << std::endl;
}

void rmcNMEAcallback(RMCnmealine&  nmealine) {
    std::cout << nmealine << std::endl;
}

