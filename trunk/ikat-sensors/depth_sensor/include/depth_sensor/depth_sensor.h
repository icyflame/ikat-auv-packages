#ifndef DEPTH_SENSOR_H
#define DEPTH_SENSOR_H

#include <ros/ros.h>
#include <serialport/IkatSerialPort.h>
#include <boost/thread.hpp>

namespace ikat_sensor
{

  class DepthSensor
  {
    private:
      float current_depth;
      IkatSerialPort serial_port;
      bool sendData(const std::string &data);
      bool recieveData(std::string &data);
      boost::recursive_mutex mutex_;
    public:
      DepthSensor(const std::string &name);
      virtual ~DepthSensor();
      bool startTransmission();
      void pauseTransmission();
      bool getDepth(float &depth);
  };

}// end namespace ikat_sensor
#endif
