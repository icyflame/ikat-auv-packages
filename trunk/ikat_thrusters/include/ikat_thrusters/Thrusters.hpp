#ifndef THRUSTER_H
#define THRUSTER_H

#include <ros/ros.h>
#include <serialport/IkatSerialPort.h>
#include <cstring>
#include <string.h>


namespace ikat_hardware
{
  #define THRUSTER_V_MIN -5.0
  #define THRUSTER_V_MAX +5.0
  #define ADD1 0x01
  #define ADD2 0x02
  #define ADD3 0x03
  class Thruster
  {
    private :
      char thruster_data[20];
      ikat_sensor::IkatSerialPort serial_port;
      void setData(int thruster_no, float voltage);
    public :
      Thruster(std::string &port_name);
      Thruster(std::string &port_name,int baud_rate);
      bool connectThrusters();
      bool disconnectThrusters();
      bool sendData( float voltage,int thruster_no);
      bool haultAllThruster();
      virtual ~Thruster();
  };
}// end namespace ikat_hardware


#endif
