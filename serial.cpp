/**
 *  @file   serial.cpp
 *  @brief  Simple Serial Monitor
 *  @author KrizTioaN (christiaanboersma@hotmail.com)
 *  @date   2021-07-17
 *  @note   BSD-3 licensed
 *
 ***********************************************/

#include "Serial.h"

#include <iostream>

#define BUFF_SIZE 128

int main(int argc, char *argv[], char **envp) {

  if (argc != 3) {
    std::cerr << "Simply Serial Monitor"
              << "\n\n"
              << "Usage:\n\n\t" << argv[0] << " port baudrate\n\n"
              << "Example: \n\n\t" << argv[0]
              << " /dev/cu.usbmodem14301 115200\n\n";
    return 1;
  }

  int baudrate = strtol(argv[2], nullptr, 10);

  Serial serial(argv[1], baudrate);
  if (!serial.good()) {
    std::cerr << serial.getErrorMessage() << std::endl;
    return serial.getErrorCode();
  }

  char buff[BUFF_SIZE];
  while (true) {
    ssize_t nbytes = serial.read(buff, BUFF_SIZE);
    if (nbytes <= 0) {
      std::cerr << serial.getErrorMessage() << std::endl;
      return serial.getErrorCode();
    }
    std::cout.write(buff, nbytes);
  }

  return 0;
}