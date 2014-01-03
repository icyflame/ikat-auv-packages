#include <ros/ros.h>
#include <mt9_sensor/MT9Sensor.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"MT9_Sensor_Node");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    ros::Publisher publisher = n.advertise<ikat_sensor_data::mt9_sensor_data>("Orientation_data_from_MT9",1);
    ikat_sensor_data::mt9_sensor_data data;
	std::string port_name = "/dev/" ;
	  if(argc==2)
	  {
		port_name += argv[1];
	  }
	  else
	  {
		port_name += "ttyUSB0";
	  }	
    ikat_sensor::MT9Sensor mt9(port_name.c_str());
    ROS_INFO("sensor inint");
    int count=0;

    if(mt9.startDataProcessing())
    {
        while(ros::ok())
        {
            if(mt9.getData(data))
            {
                if(count==10)
                {
                    publisher.publish(data);
                    count=0;
                }
                else
                {
                    count++;
                }
            }
            else
            {
                ROS_ERROR_STREAM("Unable to conect to MT9");
            }
            ros::spinOnce();
            loop_rate.sleep();
            ROS_DEBUG("looping");
        }
    }
    return 0;
}
