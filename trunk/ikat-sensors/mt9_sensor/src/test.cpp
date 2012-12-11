#include <ros/ros.h>
#include <mt9_sensor/MT9Sensor.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"MT9_Sensor_Node");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);
    ros::Publisher publisher = n.advertise<ikat_sensor_data::mt9_sensor_data>("Orientation_data_from_MT9",10);
    ikat_sensor_data::mt9_sensor_data data;
    ikat_sensor::MT9Sensor mt9("/dev/ttyUSB0");
    ROS_INFO("sensor inint");
    if(mt9.startDataProcessing())
    {
        while(ros::ok())
        {
            if(mt9.getData(data))
            {
                publisher.publish(data);
            }
            ros::spinOnce();
            loop_rate.sleep();
            ROS_INFO("looping");
        }
    }
    return 0;
}
