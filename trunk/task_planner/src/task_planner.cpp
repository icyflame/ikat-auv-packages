#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sys/types.h>
#include <unistd.h>
#include <iostream>
#include <task_planner_data/task_planner_data.h>
#include <task_planner_data/controller_data.h>
#include <task_planner/TASKS_PARAM.h>
#include <task_planner_data/task_planner_to_front_cam.h>
#include <task_planner_data/task_planner_to_bottom_cam.h>

using namespace std;

float controller_param[3]={0};

void controllerDataCallback(const task_planner_data::controller_dataConstPtr &msg)
{
    controller_param[0]=msg->param[0];
    controller_param[1]=msg->param[1];
    controller_param[2]=msg->param[2];
}

int main(int argc, char **argv)
{     
      TASKS_PARAM inputObj;
      inputObj.readFromFile();
      ros::init(argc, argv,"task_planner");
      ros::NodeHandle n;
      ros::NodeHandle p;
      ros::NodeHandle r;
      ros::NodeHandle s;
      ros::Subscriber contrller_data_callback = n.subscribe<task_planner_data::controller_data>("controller_data",5,controllerDataCallback);
      ros::Publisher tasksPub = p.advertise<task_planner_data::task_planner_data>("task_planner_data",1);
      ros::Publisher frontcamPub = r.advertise<task_planner_data::task_planner_to_front_cam>("task_planner_data", 1);
      ros::Publisher bottomcamPub = s.advertise<task_planner_data::task_planner_to_bottom_cam>("task_planner_data", 1);
      task_planner_data::task_planner_data choice;
      task_planner_data::task_planner_to_front_cam frontcamChoice;
      task_planner_data::task_planner_to_bottom_cam bottomcamChoice;
      float task_start_time=0,task_end_time=0;

      cout<<"STARTING VEHICLE"<<endl;
      task_start_time=ros::Time::now().toSec();
      choice.task_choice=inputObj.START;
      tasksPub.publish(choice);
      frontcamPub.publish(frontcamChoice);
      bottomcamPub.publish(bottomcamChoice);
      while(ros::Time::now().toSec()-task_start_time<inputObj.INTERVAL_VEHICLE_START);

      cout<<"FOLLOWING MARKER"<<endl;
      task_start_time=ros::Time::now().toSec();
      choice.task_choice=inputObj.MARKER;
      tasksPub.publish(choice);
      frontcamPub.publish(frontcamChoice);
      bottomcamPub.publish(bottomcamChoice);
      while(abs(controller_param[0])>inputObj.INTERVAL_FOLLOWING_MARKER);

      cout<<"MARKER LOCKED"<<endl;
      task_start_time=ros::Time::now().toSec();
      choice.task_choice=inputObj.GO_FORWARD;
      tasksPub.publish(choice);
      frontcamPub.publish(frontcamChoice);
      bottomcamPub.publish(bottomcamChoice);
      while(ros::Time::now().toSec()-task_start_time<inputObj.INTERVAL_MARKER_LOCKED);

      cout<<"MARKER DONE!!! GOING DOWN"<<endl;
      task_start_time=ros::Time::now().toSec();
      choice.task_choice=inputObj.GO_DOWN;
      tasksPub.publish(choice);
      frontcamPub.publish(frontcamChoice);
      bottomcamPub.publish(bottomcamChoice);
      cout<<"Killing Marker Process"<<endl;
      //use fork() exec()
      cout<<"Starting buoy process"<<endl;

      while(ros::Time::now().toSec()-task_start_time<inputObj.INTERVAL_GOING_DOWN);

      cout<<"BUOY TOUCHING"<<endl;
      task_start_time=ros::Time::now().toSec();
      choice.task_choice=inputObj.BUOY;
      tasksPub.publish(choice);
      frontcamPub.publish(frontcamChoice);
      bottomcamPub.publish(bottomcamChoice);
      while(controller_param[0]<inputObj.INTERVAL_BUOY_AREA);

      cout<<"HITTING BUOY"<<endl;
      task_start_time=ros::Time::now().toSec();
      choice.task_choice=inputObj.GO_FORWARD;
      tasksPub.publish(choice);
      frontcamPub.publish(frontcamChoice);
      bottomcamPub.publish(bottomcamChoice);
      while(ros::Time::now().toSec()-task_start_time<inputObj.INTERVAL_BUOY_ANGLE);

      cout<<"BUOY HIT RETREATING"<<endl;
      task_start_time=ros::Time::now().toSec();
      choice.task_choice=inputObj.GO_BACK;
      tasksPub.publish(choice);
      frontcamPub.publish(frontcamChoice);
      bottomcamPub.publish(bottomcamChoice);
      while(ros::Time::now().toSec()-task_start_time<inputObj.INTERVAL_BUOY_RETREATING);

      cout<<"BUOY DONE!!! COMMING UP"<<endl;
      task_start_time=ros::Time::now().toSec();
      choice.task_choice=inputObj.GO_UP;
      tasksPub.publish(choice);
      frontcamPub.publish(frontcamChoice);
      bottomcamPub.publish(bottomcamChoice);
      while(ros::Time::now().toSec()-task_start_time<inputObj.INTERVAL_RISE_UP);

      cout<<"ALL TASKS DONE HAULT!!!"<<endl;
      choice.task_choice=100;
      tasksPub.publish(choice);
      frontcamPub.publish(frontcamChoice);
      bottomcamPub.publish(bottomcamChoice);
      return 0;
}
