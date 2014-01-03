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
  std::string port_name = "/dev/" ;
  if(argc==2)
  {
	port_name += argv[1];
  }
  else
  {
        port_name += "ttyS0";
  }		
  ikat_hardware::Thruster auv_thrusters(port_name,9600);
  if(auv_thrusters.connectThrusters())
  {
      ROS_INFO_STREAM("thruster are online to use");
      auv_thrusters.sendData(2.5,0);
      sleep(1);
      auv_thrusters.sendData(0,0);
      //sleep(1);
      //auv_thrusters.haultAllThruster();
      auv_thrusters.sendData(2.5,1);
      sleep(1);
      //auv_thrusters.haultAllThruster();
      auv_thrusters.sendData(0,1);
      //sleep(1);
      auv_thrusters.sendData(2.5,2);
      sleep(1);
      //auv_thrusters.haultAllThruster();
      auv_thrusters.sendData(0,2);
      //sleep(1);
      auv_thrusters.sendData(2.5,3);
      sleep(1);
      //auv_thrusters.haultAllThruster();
      auv_thrusters.sendData(0,3);
      //sleep(1);
  }
  else
      ROS_ERROR_STREAM("Error while conneting thruster");
  auv_thrusters.haultAllThruster();
  auv_thrusters.disconnectThrusters();
  return 0;
}
