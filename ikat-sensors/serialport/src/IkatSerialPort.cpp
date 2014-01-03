#include <serialport/IkatSerialPort.h>

namespace ikat_sensor
{
  IkatSerialPort::IkatSerialPort()
  {
      port_is_open = false;
      setPortName("");
      setRate(0);
      setFlags(0);
      file_descriptor = -1;
  }

  IkatSerialPort::IkatSerialPort(const std::string &name,int rate, long flags)
  {
      port_is_open = false;
      setPortName(name);
      setRate(rate);
      setFlags(flags);
      file_descriptor = -1;
  }

  IkatSerialPort::IkatSerialPort(const std::string &name, int rate)
  {	
      port_is_open = false;
      setPortName(name);
      setRate(rate);
      setFlags(0);
      file_descriptor = -1;
  }

  IkatSerialPort::IkatSerialPort(const std::string &name)
  {
      port_is_open = false;
      setPortName(name);
      setRate(0);
      setFlags(0);
      file_descriptor = -1;
  }

  IkatSerialPort::~IkatSerialPort()
  {
      if(port_is_open)
      {
          closePort();
      }
  }

  bool IkatSerialPort::isOpen()
  {
      return port_is_open;
  }

  bool IkatSerialPort::openPort()
  {
      if((!port_is_open)&&(port_name.c_str()!=NULL))
      {
          file_descriptor = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
      }
      else if(port_is_open)
           {
             ROS_ERROR_STREAM("open port unable to open port : "<< port_name << " port is already open");
             return false;
           }
           else
           {
             ROS_ERROR_STREAM("open port unable to open port : no port name is given");
             return false;
           }

      if(file_descriptor==-1)
      {
          ROS_ERROR_STREAM("open port unable to open port : "<< port_name);
          return false;
      }
      else
      {
          fcntl(file_descriptor,F_SETFL,0);
          port_is_open = true;
          if(configPort())
          {
            return true;
          }
          else
          {
              ROS_ERROR_STREAM("Can't configure the port named :" << port_name);
              return false;
          }
      }
  }

  bool IkatSerialPort::closePort()
  {
      if(port_is_open)
      {
          close(file_descriptor);
          port_is_open = false;
      }
      else
      {
          ROS_ERROR_STREAM("Serial Port : "<<port_name<<
                           " is already closed");
      }
      return port_is_open;
  }

  bool IkatSerialPort::setPortName(const std::string &name)
  {
      if(!port_is_open)
      {
          port_name = name;
      }
      else
      {
          ROS_ERROR_STREAM("Serial Port is already been used as name : "<<port_name<<
                           ". Hence the name cannot change be changed");
      }
      return !port_is_open;
  }

  bool IkatSerialPort::setFlags(long flags)
  {
      if(!port_is_open)
      {
          port_flags = flags;
      }
      else
      {
          ROS_ERROR_STREAM("Serial Port is already been used as name : "<<port_name<<
                           ". Hence the flags cannot be changed");
      }
      return !port_is_open;
  }

  bool IkatSerialPort::setRate(long rate)
  {
      if(!port_is_open)
      {
          switch (rate)
                {
                   case 38400:
                   default:
                      baudrate = B38400;
                      break;
                   case 19200:
                      baudrate  = B19200;
                      break;
                   case 9600:
                      baudrate  = B9600;
                      break;
                   case 4800:
                      baudrate  = B4800;
                      break;
                   case 2400:
                      baudrate  = B2400;
                      break;
                   case 1800:
                      baudrate  = B1800;
                      break;
                   case 1200:
                      baudrate  = B1200;
                      break;
                   case 600:
                      baudrate  = B600;
                      break;
                   case 300:
                      baudrate  = B300;
                      break;
                   case 200:
                      baudrate  = B200;
                      break;
                   case 150:
                      baudrate  = B150;
                      break;
                   case 134:
                      baudrate  = B134;
                      break;
                   case 110:
                      baudrate  = B110;
                      break;
                   case 75:
                      baudrate  = B75;
                      break;
                   case 50:
                      baudrate  = B50;
                      break;
                }
      }
      else
      {
          ROS_ERROR_STREAM("Serial Port is already been used as name : "<<port_name<<
                           ". Hence the rate cannot be changed");
      }
      return !port_is_open;
  }

  bool IkatSerialPort::configPort()
  {
      struct termios port_setting;
      tcgetattr(file_descriptor, &port_setting);
      cfsetispeed(&port_setting,baudrate);
      cfsetospeed(&port_setting,baudrate);
      port_setting.c_cflag |= (CLOCAL | CREAD);
      port_setting.c_cflag &= ~PARENB;
      port_setting.c_cflag &= ~CSTOPB;
      port_setting.c_cflag &= ~CSIZE;
      port_setting.c_cflag |= CS8;
      port_setting.c_cflag |= port_flags;
      port_setting.c_iflag |= (IXON | IXOFF | IXANY);
      port_setting.c_cflag &= ~CRTSCTS;
      port_setting.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 
      tcsetattr(file_descriptor,TCSANOW, &port_setting);
      return true;
  }

  int IkatSerialPort::writeData(const std::string &bytes)
  {
      int byte_written = -1;
      int no_of_bytes = bytes.size();
      if(port_is_open)
      {
          byte_written = write(file_descriptor, bytes.c_str(),no_of_bytes);
      }
      return byte_written;
  }

  int IkatSerialPort::readData(std::string &bytes, int no_of_bytes)
  {
      int byte_read = -1;
      if(port_is_open)
      {
          bytes.clear();
          bytes.resize(no_of_bytes);
          byte_read = read(file_descriptor, (char *)bytes.c_str(),no_of_bytes);
      }
      return byte_read;
  }

  bool IkatSerialPort::readChar(char &data)
  {
      int byte_read = -1;
      if(port_is_open)
      {
          data = 0;
          byte_read = read(file_descriptor,&data,1);
          return true;
      }
      return false;
  }

}	
