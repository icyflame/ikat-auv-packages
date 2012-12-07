#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ikat_sensor_data/depth_sensor_data.h>
#include <ikat_sensor_data/mt9_sensor_data.h>
#include <ikat_sensor_data/control_data.h>
#include <ikat_thrusters/Thrusters.hpp>
#include <ikat_controller/DepthController.h>
#include <ikat_controller/YawController.h>


// Thrusters definition

#define CH0 0x00
#define CH1 0x01
#define CH2 0x02
#define CH3 0x03

#define SurgeLeftThruster	CH0
#define SurgeRightThruster	CH2
#define DepthRightThruster	CH3
#define DepthLeftThruster	CH1

using namespace std;
using namespace ikat_hardware;

DepthController depthobj;
YawController yawobj;

void depthCallback(const ikat_sensor_data::depth_sensor_data::ConstPtr& msg)
{
      if (depthobj.no_of_times_from_begining_for_depth_sensor==5)
      {
           depthobj.DEPTH_AT_SURFACE=msg->depth;
      }
      if(depthobj.no_of_times_from_begining_for_depth_sensor<10)
            depthobj.no_of_times_from_begining_for_depth_sensor++;
      float data = msg->depth-depthobj.DEPTH_AT_SURFACE;
      ROS_INFO("depth: [%f]",data);
      depthobj.getdepth(data);
}

void mt9Callback(const ikat_sensor_data::mt9_sensor_data::ConstPtr& msg)
{
    float Data[3]={0};
    Data[0]=msg->MT9_data[0];
    Data[1]=msg->MT9_data[1];
    Data[2]=msg->MT9_data[2];
    if (Data[2]>0 && yawobj.no_of_times_from_begining_for_mt9==5)
    {
         yawobj.theta_relative=Data[2];
    }
    Data[2]=Data[2]-yawobj.theta_relative+180;
    if(yawobj.no_of_times_from_begining_for_mt9<20)
          yawobj.no_of_times_from_begining_for_mt9++;
    if (Data[2]<0)
    {
         Data[2]+=360;
    }
    if (Data[2]>360)
    {
         Data[2]-=360;
    }
    ROS_INFO("Roll [%f]      ",Data[0]);
    ROS_INFO("Pitch [%f]     ",Data[1]);
    ROS_INFO("Yaw [%f]       ",Data[2]);
    yawobj.get_Data(Data);
}
/*void controllSignalCallback(const ikat_sensor_data::control_data::ConstPtr& msg)
{
    obj.steady_angle=msg->angle;
    //obj.steady_depth=msg->depth;
    obj.is_image=msg->use_image;
    obj.x_coordinate_of_image=msg->X;
    obj.y_coordinate_of_image=msg->Y;
    obj.KP_YAW_IMAGE=msg->Kp;
    obj.KI_YAW_IMAGE=msg->Ki;
    obj.KD_YAW_IMAGE=msg->Kd;
}*/


int main(int argc, char **argv)
{
      ros::init(argc, argv, "controller");
      ros::NodeHandle n;
      ros::Subscriber depth_callback = n.subscribe("current_depth", 10,depthCallback);
      ros::Subscriber mt9_callback =   n.subscribe("Orientation_data_from_MT9", 10, mt9Callback);
      //ros::Subscriber task_planner_callback=n.subscribe("control_signal",10, controllSignalCallback);
      ros::Rate loopRate(1);
      std::string port_name = "/dev/ttyACM0";
      ikat_hardware::Thruster th(port_name,9600);
      if(th.connectThrusters())
         cout<<"The serial port of thruster is connected\n";
      int is_image = 0;
      while(ros::ok())
      {
          ros::spinOnce();
          if(!is_image)
          {
            yawobj.yawController(30);
            depthobj.depthController(.7);
          }
          else
          {
              //yawobj.yawControllerImage();
              //obj.depthControllerImage();
          }

          ///////////////////////////////////////////////
          //////sending information to thrusters
          ///////////////////////////////////////////////
          //th.sendData(obj.thruster_surge_left,SurgeLeftThruster);
          //th.sendData(obj.thruster_surge_right,SurgeRightThruster);
          //th.sendData(obj.vertical_speed,DepthRightThruster);
          //th.sendData(obj.vertical_speed,DepthLeftThruster);

          loopRate.sleep();

      }
      return 0;
}
