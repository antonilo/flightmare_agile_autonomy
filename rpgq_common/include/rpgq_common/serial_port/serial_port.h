#pragma once

#include <ros/ros.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <thread>

namespace RPGQ
{
  class SerialPort
  {
  public:
    // constructor & destructor
    SerialPort(void);
    ~SerialPort(void);

    // public set up & get functions
    bool Initialize(const std::string port, const int baud);
    int GetFileDescriptor(void) const {return fd_;};

    // public read/write functions
    unsigned int Write(const void* buffer, const unsigned int length);
    unsigned int Read(void* buffer, const unsigned int bufferLength);

  private:
    // general variables
    int fd_;
  };

} // namespace RPGQ
