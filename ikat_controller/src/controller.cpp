//contact for TOWING TANK KEY
//Jerin: 9775051559
//Atanu: 09434538984

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ikat_sensor_data/depth_sensor_data.h>
#include <ikat_sensor_data/mt9_sensor_data.h>
#include <ikat_thrusters/Thrusters.hpp>
#include <ikat_controller/DepthController.h>
#include <ikat_controller/YawController.h>
#include <ikat_sensor_data/thruster_data.h>
#include <ikat_ip_data/ip_buoy_data.h>
#include <ikat_ip_data/ip_marker_data.h>
#include <ikat_controller/ImageController.h>
#include <task_planner_data/task_planner_data.h>
#include <task_planner_data/controller_data.h>
#include <task_planner/TASKS_PARAM.h>
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
ImageController imageControllerObj;
float x_buoy,y_buoy,area_buoy,x_marker,y_marker,angle_marker;
float mt9_data[3]={0};
float depth_sensor_data=0;
char  task=100;
char  prev_task = 100;


void buoyDetectCallback(const ikat_ip_data::ip_buoy_data::ConstPtr &msg)
{
    x_buoy = msg->Buoy_Center_data[0];
    y_buoy = msg->Buoy_Center_data[1];
    area_buoy=msg->Buoy_area;
}

void taskPlannerCallback(const task_planner_data::task_planner_data::ConstPtr &msg)
{
    task=msg->task_choice;
}

void markerDetectCallback(const ikat_ip_data::ip_marker_data::ConstPtr &msg)
{
    x_marker=msg->Marker_data[0];
    y_marker=msg->Marker_data[1];
    angle_marker=msg->Marker_data[2];

}

void depthCallback(const ikat_sensor_data::depth_sensor_data::ConstPtr& msg)
{
      if (depthobj.no_of_times_from_begining_for_depth_sensor==5)
      {
           depthobj.DEPTH_AT_SURFACE=msg->depth;
      }
      if(depthobj.no_of_times_from_begining_for_depth_sensor<=5)
            depthobj.no_of_times_from_begining_for_depth_sensor++;
      depth_sensor_data = msg->depth-depthobj.DEPTH_AT_SURFACE;
      ROS_DEBUG("depth: [%f]",depth_sensor_data);
      depthobj.getdepth(depth_sensor_data);
}

void mt9Callback(const ikat_sensor_data::mt9_sensor_data::ConstPtr& msg)
{
    mt9_data[0]=msg->MT9_data[0];
    mt9_data[1]=msg->MT9_data[1];
    mt9_data[2]=msg->MT9_data[2];
    if (mt9_data[2]>0 && yawobj.no_of_times_from_begining_for_mt9==5)
    {
         yawobj.theta_relative=mt9_data[2];
    }
    mt9_data[2]=mt9_data[2]-yawobj.theta_relative+180;
    if(yawobj.no_of_times_from_begining_for_mt9<=5)
          yawobj.no_of_times_from_begining_for_mt9++;
    if (mt9_data[2]<0)
    {
         mt9_data[2]+=360;
    }
    if (mt9_data[2]>360)
    {
         mt9_data[2]-=360;
    }
    ROS_DEBUG("Roll [%f]      ",mt9_data[0]);
    ROS_DEBUG("Pitch [%f]     ",mt9_data[1]);
    ROS_DEBUG("Yaw [%f]       ",mt9_data[2]);
    yawobj.get_Data(mt9_data);
}


int main(int argc, char **argv)
{
      ros::init(argc, argv, "controller");
      ros::NodeHandle n;
      ros::NodeHandle p;
      ros::Subscriber depth_callback = n.subscribe("current_depth", 10,depthCallback);
      ros::Subscriber mt9_callback =   n.subscribe("Orientation_data_from_MT9", 10, mt9Callback);
      ros::Subscriber task_planner_callback=n.subscribe("task_planner_data",10, taskPlannerCallback);
      ros::Subscriber buoy_callback = n.subscribe("BuoyData",10,buoyDetectCallback);
      ros::Subscriber marker_callback = n.subscribe("markerData",10,markerDetectCallback);
      ros::Publisher thrusterPub = p.advertise<ikat_sensor_data::thruster_data>("thrusterControlData",3);
      ros::Publisher taskPlannerPub = p.advertise<task_planner_data::controller_data>("controller_data",3);
      ros::Rate loopRate(5);
      ikat_sensor_data::thruster_data thrusterData;
      task_planner_data::controller_data controllerData;
      class TASKS_PARAM TASKS;
      TASKS.readFromFile();
      float * buff = new float[2];
      int i=0;
      while(ros::ok())
      { 
          i++;
          if(i>20)
              break;
          ros::spinOnce();
          loopRate.sleep();
      }
      float prev_task_yaw=180,prev_task_depth=0;
      while(ros::ok())
      {
        ros::spinOnce();
        if(prev_task!=task)
        {
            imageControllerObj.setControlParamters(task);
            prev_task = task;
        }
        switch (task)
        {
            case START:
                    thrusterData.heaveThrust= 0;//depthobj.depthController(0.0);
                    yawobj.yawController(180,buff);
                    thrusterData.surgeLeft=0;//buff[0];
                    thrusterData.surgeRight=0;//buff[1];
                    thrusterPub.publish(thrusterData);
                    cout<<"going down"<<endl;
                    break;
            case MARKER:
                    imageControllerObj.yawController(angle_marker,buff);
                    thrusterData.heaveThrust= depthobj.depthController(0.0);
                    thrusterData.surgeLeft = buff[0];
                    thrusterData.surgeRight = buff[1];
                    thrusterPub.publish(thrusterData);
                    controllerData.param[0]=angle_marker;
                    controllerData.param[1]=mt9_data[2];
                    controllerData.param[2]=depth_sensor_data;
                    taskPlannerPub.publish(controllerData);
                    prev_task_yaw=mt9_data[2];
                    prev_task_depth=depth_sensor_data;
                    cout<<"MARKER"<<endl;
                    break;
            case GO_FORWARD:
                    thrusterData.heaveThrust= depthobj.depthController(prev_task_depth);
                    yawobj.yawController(prev_task_yaw,buff);
                    thrusterData.surgeLeft=buff[0]+2;
                    thrusterData.surgeRight=buff[1]+2;
                    thrusterPub.publish(thrusterData);
                    cout<<"going forwrd"<<endl;
                    break;
            case GO_DOWN:
                    thrusterData.heaveThrust= depthobj.depthController(0.4);
                    yawobj.yawController(prev_task_yaw,buff);
                    thrusterData.surgeLeft=buff[0];
                    thrusterData.surgeRight=buff[1];
                    thrusterPub.publish(thrusterData);
                    prev_task_depth=0.4;
                    cout<<"going down"<<thrusterData.heaveThrust<<endl;
                    break;
            case BUOY:
                    imageControllerObj.yawController(x_buoy,buff);
                    thrusterData.heaveThrust= imageControllerObj.depthController(y_buoy);
                    thrusterData.surgeLeft = buff[0];
                    thrusterData.surgeRight = buff[1];
                    thrusterPub.publish(thrusterData);
                    controllerData.param[0]=area_buoy;
                    controllerData.param[1]=mt9_data[2];
                    controllerData.param[2]=depth_sensor_data;
                    taskPlannerPub.publish(controllerData);
                    ros::spinOnce();
                    prev_task_yaw=mt9_data[2];
                    prev_task_depth=depth_sensor_data;
                    cout<<"buoy"<<endl;
                    break;
            case GO_BACK:
                    thrusterData.heaveThrust= depthobj.depthController(prev_task_depth);
                    yawobj.yawController(prev_task_yaw,buff);
                    thrusterData.surgeLeft=buff[0]-3;
                    thrusterData.surgeRight=buff[1]-3;
                    thrusterPub.publish(thrusterData);
                    cout<<"going back"<<endl;
                    break;
            case GO_UP:
                    thrusterData.heaveThrust= depthobj.depthController(0.05);
                    yawobj.yawController(prev_task_yaw,buff);
                    thrusterData.surgeLeft=buff[0];
                    thrusterData.surgeRight=buff[1];
                    thrusterPub.publish(thrusterData);
                    cout<<"going up"<<endl;
                    break;
            default:
                    thrusterData.heaveThrust= 0;
                    thrusterData.surgeLeft=0;
                    thrusterData.surgeRight=0;
                    thrusterPub.publish(thrusterData);
                    break;
        }
      loopRate.sleep();
    }

      thrusterData.heaveThrust=0;
      thrusterData.surgeLeft = 0;
      thrusterData.surgeRight = 0;
      thrusterPub.publish(thrusterData);
      sleep(1);
      delete(buff);
      return 0;
}
