#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sys/types.h>
#include <unistd.h>
#include <iostream>
#include <task_planner_data/task_planner_data.h>
#include <task_planner_data/controller_data.h>
#include <task_planner/Tasks.h>

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
      ros::init(argc, argv,"task_planner");
      ros::NodeHandle n;
      ros::NodeHandle p;
      ros::Subscriber contrller_data_callback = n.subscribe<task_planner_data::controller_data>("controller_data",5,controllerDataCallback);
      ros::Publisher tasksPub = p.advertise<task_planner_data::task_planner_data>("task_planner_data",1);
      task_planner_data::task_planner_data choice;
      float task_start_time=0,task_end_time=0;

      cout<<"STARTTING VEHICLE"<<endl;
      task_start_time=ros::Time::now().toSec();
      choice.task_choice=START;
      tasksPub.publish(choice);
      while(ros::Time::now().toSec()-task_start_time<2);

      cout<<"FOLLOWING MARKER"<<endl;
      task_start_time=ros::Time::now().toSec();
      choice.task_choice=MARKER;
      tasksPub.publish(choice);
      while(abs(controller_param[0])>3);

      cout<<"MARKER LOCKED"<<endl;
      task_start_time=ros::Time::now().toSec();
      choice.task_choice=GO_FORWARD;
      tasksPub.publish(choice);
      while(ros::Time::now().toSec()-task_start_time<3);

      cout<<"MARKER DONE!!! GOING DOWN"<<endl;
      task_start_time=ros::Time::now().toSec();
      choice.task_choice=GO_DOWN;
      tasksPub.publish(choice);
      cout<<"Killing Marker Process"<<endl;
      //use fork() exec()
      cout<<"Starting buoy process"<<endl;

      while(ros::Time::now().toSec()-task_start_time<5);

      cout<<"BUOY TOUCHING"<<endl;
      task_start_time=ros::Time::now().toSec();
      choice.task_choice=BUOY;
      tasksPub.publish(choice);
      while(controller_param[0]<0.15);

      cout<<"HITTING BUOY"<<endl;
      task_start_time=ros::Time::now().toSec();
      choice.task_choice=GO_FORWARD;
      tasksPub.publish(choice);
      while(ros::Time::now().toSec()-task_start_time<5);

      cout<<"BUOY HIT RETREATING"<<endl;
      task_start_time=ros::Time::now().toSec();
      choice.task_choice=GO_BACK;
      tasksPub.publish(choice);
      while(ros::Time::now().toSec()-task_start_time<5);

      cout<<"BUOY DONE!!! COMMING UP"<<endl;
      task_start_time=ros::Time::now().toSec();
      choice.task_choice=GO_UP;
      tasksPub.publish(choice);
      while(ros::Time::now().toSec()-task_start_time<5);

      cout<<"ALL TASKS DONE HAULT!!!"<<endl;
      choice.task_choice=100;
      tasksPub.publish(choice);
      return 0;
}
