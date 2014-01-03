#include <ros/ros.h>
#include <ikat_thrusters/Thrusters.hpp>
#include <ikat_sensor_data/thruster_data.h>

float data[3]={0};

void thrusterCallback(const ikat_sensor_data::thruster_data::ConstPtr& thruster_data)
{
    data[0]=thruster_data->heaveThrust;
    data[1]=thruster_data->surgeLeft;
    data[2]=thruster_data->surgeRight;
}

int main(int argv,char ** argc)
{
    ros::init(argv,argc,"thrusterrun");
    ros::NodeHandle n;
    ros::Subscriber thrustersub = n.subscribe("thrusterControlData",3,thrusterCallback);
    ros::Rate looprate(100);
    int count = 0;
    std::string portname ="/dev/ttyS0";
    ikat_hardware::Thruster th(portname,9600);
    if(!th.connectThrusters())
        ROS_ERROR("Thruster Serial port was not connected correctly");
    while(ros::ok())
    {
        th.sendData(data[0],1);
        th.sendData(data[0],3);
        th.sendData(data[1],0);
        th.sendData(data[2],2);
        ros::spinOnce();
        looprate.sleep();
        count++;
    }
    return 0;
}
