#include <ros/ros.h>
#include <ros/console.h>
#include <depth_sensor/depth_sensor.h>
#include <ikat_sensor_data/depth_sensor_data.h>
// comment the below line to hide the data from console
#define DEPTH_DEBUG 1

int main(int argc, char** argv)
{
  ros::init(argc,argv,"depth_sensor_node");

#ifdef DEPTH_DEBUG
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
#endif

  ikat_sensor::DepthSensor depth_sensor("/dev/ttyUSB0");
  depth_sensor.startTransmission();

  ros::NodeHandle n;
  ros::Publisher depth_data_publisher = n.advertise<ikat_sensor_data::depth_sensor_data>("current_depth",5);
  ros::Rate loop_rate(1);
  while(ros::ok())
  {
      float depth;
      ikat_sensor_data::depth_sensor_data current_depth;
      depth_sensor.getDepth(depth);
      current_depth.depth = depth;
      depth_data_publisher.publish(current_depth);
      std::cout<<depth<<std::endl;
      ros::spinOnce();
      loop_rate.sleep();
  }
  depth_sensor.pauseTransmission();
  depth_sensor.~DepthSensor();
  return 0;
}
port_setting.c_cflag |= (CLOCAL | CREAD);
      port_setting.c_cflag &= ~PARENB;
      port_setting.c_cflag &= ~CSTOPB;
      port_setting.c_cflag &= ~CSIZE;
      port_setting.c_cflag |= CS8;
      port_setting.c_cflag |= port_flags;
      port_setting.c_iflag |= (IXON | IXOFF | IXANY);
      port_setting.c_iflag |= (INLCR);
      port_setting.c_cflag &= ~CRTSCTS;
      port_setting.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
      tcsetattr(file_descriptor,TCSANOW, &port_setting);
