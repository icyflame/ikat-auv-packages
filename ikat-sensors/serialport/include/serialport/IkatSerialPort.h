#ifndef IKATSERIALPORT_H
#define IKATSERIALPORT_H

#include <stdio.h>
#include <ros/ros.h>
#include <errno.h>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdlib>

namespace ikat_sensor
{
  #define PERITYODD  PARENB|PARODD
  #define PARITYEN   PARENB
  #define STOPBIT2   CSTOPB
  class IkatSerialPort
  {
    private:
      std::string port_name;
      long baudrate;
      bool port_is_open;
      int file_descriptor;
      long port_flags;
      bool configPort();

    public:
      IkatSerialPort();
      IkatSerialPort(const std::string &name,int rate, long flags);
      IkatSerialPort(const std::string &name, int rate);
      IkatSerialPort(const std::string &name);
      bool openPort();
      bool closePort();
      bool isOpen();
      bool setFlags(long flags);
      bool setRate(long rate);
      bool setPortName(const std::string &name);
      int readData(std::string &bytes, int no_of_bytes);
      bool readChar(char &data);
      int writeData(const std::string &bytes);
      virtual ~IkatSerialPort();
  };
}// end namespace ikar-sensor
#endif // IKATSERIALPORT_H
