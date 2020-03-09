#include "gps/simplertk2b/simplertk2b.h"

int main() {
    Simplertk2b simplertk2b("/dev/ttyACM0", "flepos.vlaanderen.be", "FLEPOSVRS32GREC", "852a009", "97115");
}

