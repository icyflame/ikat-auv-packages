#include <ros/ros.h>
#include <ikat_thrusters/Thrusters.hpp>

//#define DEPTH_DEBUG 1

int main(int argc, char** argv)
{
  ros::init(argc,argv,"thruster_test_code");
#ifdef DEPTH_DEBUG
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
#endif
  std::string port_name = " ";
  ikat_hardware::Thruster auv_thrusters(port_name,9600);
  if(auv_thrusters.connectThrusters())
  {
      ROS_DEBUG_STREAM("thruster are online to use");
      auv_thrusters.sendData(2.5,0);
      auv_thrusters.sendData(2.5,0);
      auv_thrusters.sendData(2.5,0);
      sleep(2);
      auv_thrusters.haultAllThruster();
  }
  else
      ROS_ERROR_STREAM("Error while conneting thruster");
  return 0;
}
