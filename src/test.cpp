
#include <stdlib.h>
#include <iostream>
#include "DynamixelV2.h"

using namespace ssr::dynamixel;

int main(const int argc, const char* argv[]) {
  if (argc != 4) {
    std::cout << "Invalid Usage." << std::endl;
    std::cout << "USAGE: $./test filename baudrate id led" << std::endl;
    return -1;
  }
  
  try {
    DynamixelV2 m(argv[1], atoi(argv[2]));

    m.SetLED(atoi(argv[3]), atoi(argv[4]) > 0);

  } catch (DynamixelException& ex) {
    std::cout << "Exception: " << ex.what() << std::endl;
    return -1;
  }

  return 0;
}
