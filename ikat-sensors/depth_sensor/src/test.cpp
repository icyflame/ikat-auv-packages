#include <ros/ros.h>
#include <ros/console.h>
#include <depth_sensor/depth_sensor.h>
#include <ikat_sensor_data/depth_sensor_data.h>
// comment the below line to hide the data from console
//#define DEPTH_DEBUG 1

int main(int argc, char** argv)
{
  ros::init(argc,argv,"depth_sensor_node");
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
	port_name += "ttyS1";
  }	
  ikat_sensor::DepthSensor depth_sensor(port_name.c_str());
  depth_sensor.startTransmission();
  ros::NodeHandle n;
  ros::Publisher depth_data_publisher = n.advertise<ikat_sensor_data::depth_sensor_data>("current_depth",1);
  ros::Rate loop_rate(50);
  char count=0;
  while(ros::ok())
  {
      float depth;
	  float pre_depth;
      ikat_sensor_data::depth_sensor_data current_depth;
      depth_sensor.getDepth(depth);
      if(depth==0)
        depth = pre_depth;
      else
        pre_depth = depth;
      current_depth.depth = depth;
      if(true || count==0)
      {
          depth_data_publisher.publish(current_depth);
          //std::cout<<depth<<std::endl;
          count=5;
      }
      else
      {
          count--;
      }

      ros::spinOnce();
	//	ros::spin();
      loop_rate.sleep();
  }
  depth_sensor.pauseTransmission();
  depth_sensor.~DepthSensor();
  return 0;
}
