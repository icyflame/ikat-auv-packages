#include <ikat_thrusters/Thrusters.hpp>

namespace ikat_hardware
{
  Thruster::Thruster(std::string &port_name):serial_port(port_name,9600)
  {
      ROS_DEBUG_STREAM("Thruster in initialized");
  }
  Thruster::Thruster(std::string &port_name, int baud_rate):serial_port(port_name,baud_rate)
  {
      ROS_DEBUG_STREAM("Thruster in initialized");
  }

  bool Thruster::connectThrusters()
  {
      return serial_port.openPort();
  }
  bool Thruster::disconnectThrusters()
  {
      return serial_port.closePort();
  }
  bool Thruster::sendData(int thruster_no, float voltage)
  {
      setData(thruster_no,voltage);
      return serial_port.writeData(thruster_data);
  }
  
  void Thruster::setData(int thruster_no, float voltage)
  {
      memset(thruster_data,0,sizeof(thruster_data));
      if(voltage>5.0)
      {
          ROS_WARN_STREAM("thruster voltage with address "<<thruster_no<<" crossed +5.0 !!!");
          voltage=5.0;
      }
      if(voltage<-5.0)
      {
          ROS_WARN_STREAM("thruster voltage with address "<<thruster_no<<" crossed -5.0 !!!");
          voltage=-5.0;
      }
      if(voltage>=0.0)
      {
          sprintf(thruster_data,"#0%dC%d-0%.3lf\r",ADD1,thruster_no,voltage);
      }
      else
      {   
          sprintf(thruster_data,"#0%dC%d-0%.3lf\r",ADD1,thruster_no,-voltage);
      }
  }
  
  bool Thruster::haultAllThruster()
  {
      return (sendData(0,0.0)&&sendData(1,0.0)&&sendData(2,0.0));
  }
  
  Thruster::~Thruster()
  {
      haultAllThruster();
      serial_port.~IkatSerialPort();
      ROS_DEBUG_STREAM("Thruster in deallocated");
  }
} // end namespace ikar_hardware
