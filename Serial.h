/**
 *  @file   Serial.h
 *  @brief  C++ Header-only Serial Port I/O
 *  @author KrizTioaN (christiaanboersma@hotmail.com)
 *  @date   2021-07-17
 *  @note   BSD-3 licensed
 *
 ***********************************************/

#ifndef Serial_H_
#define Serial_H_

#include <cerrno>  // Error integer and strerror() function
#include <cstring> // Contains strerror
#include <fcntl.h> // Contains file controls like O_RDWR
#include <string>
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

#include <sys/file.h>

class Serial {
public:
  Serial() : _fd(-1), _baudrate(0), _state(false), _errorcode(0){};

  Serial(std::string port, int baudrate = 9600) : _state(false), _fd(-1) {
    open(port, baudrate);
  }

  bool open(std::string port, int baudrate = 9600) {

    _port = port;
    _baudrate = baudrate;

    if ((_fd = ::open(_port.c_str(), O_RDWR | O_NOCTTY)) < 0) {
      _state = false;
      _errorcode = errno;
      _errormessage = strerror(_errorcode);
      return _state;
    }

    if (flock(_fd, LOCK_EX | LOCK_NB) < 0) {
      ::close(_fd);
      _fd = -1;
      _state = false;
      _errorcode = errno;
      _errormessage = strerror(_errorcode);
      return _state;
    }

    struct termios tty;
    if (tcgetattr(_fd, &tty) != 0) {
      ::close(_fd);
      _fd = -1;
      _state = false;
      _errorcode = errno;
      _errormessage = strerror(_errorcode);
      return _state;
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field (most common)
    tty.c_cflag &= ~CSIZE;  // Clear all the size bits
    tty.c_cflag |= CS8;     // 8 bits per byte (most common)
    tty.c_cflag &=
        ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |=
        CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON; // Disable canonical mode
    tty.c_lflag &= ~ECHO;   // Disable echo
    tty.c_lflag &= ~ECHOE;  // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT and SUSP

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                     ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &=
        ~OPOST; // Prevent interpretation of output bytes (e.g., newline)
    tty.c_oflag &=
        ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS;              // Prevent conversion of tabs to
    // spaces (NOT IN LINUX) tty.c_oflag &= ~ONOEOT;              // Prevent
    // removal of C-d chars (0x004) in output (NOT IN LINUX)

    tty.c_cc[VTIME] = 0; // Block
    tty.c_cc[VMIN] = 1;  // At least one byte is read

    if (cfsetspeed(&tty, _baudrate) < 0) {
      ::close(_fd);
      _fd = -1;
      _state = false;
      _errorcode = errno;
      _errormessage = strerror(_errorcode);
      return _state;
    }

    if (tcsetattr(_fd, TCSANOW, &tty) != 0) {
      ::close(_fd);
      _fd = -1;
      _state = false;
      _errorcode = errno;
      _errormessage = strerror(_errorcode);
      return _state;
    }

    _state = true;
    return _state;
  };

  void close() {
    if (_fd > -1)
      ::close(_fd);
    _fd = -1;
    _port = "";
    _baudrate = 0;
    _state = false;
    _errorcode = 0;
    _errormessage = "";
  }

  ~Serial() { ::close(_fd); };

  ssize_t write(const void *buf, size_t nbyte) {
    ssize_t size;
    if ((size = ::write(_fd, buf, nbyte)) < 0) {
      _state = false;
      _errorcode = errno;
      _errormessage = strerror(_errorcode);
    }
    return size;
  };

  ssize_t read(void *buf, size_t nbyte) {
    ssize_t size;
    if ((size = ::read(_fd, buf, nbyte)) < 0) {
      _state = false;
      _errorcode = errno;
      _errormessage = strerror(_errorcode);
    } else if (size == 0) {
      ::close(_fd);
      _fd = -1;
      _state = false;
      _errorcode = 255;
      _errormessage = "serial connection closed by remote client";
    }
    return size;
  }

  inline bool good() { return _state; }
  inline const std::string getErrorMessage() { return _errormessage; }
  inline int getErrorCode() { return _errorcode; }
  inline int filedescriptor() { return _fd; }
  inline std::string port() { return _port; }
  inline int baudrate() { return _baudrate; }

private:
  int _fd;
  std::string _port;
  int _baudrate;
  bool _state;
  int _errorcode;
  std::string _errormessage;
};

#endif