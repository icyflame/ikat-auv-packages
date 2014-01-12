#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sys/types.h>
#include <unistd.h>
#include <iostream>
#include <task_planner_data/task_planner_data.h>
#include <task_planner_data/controller_data.h>
#include <task_planner/TASKS_PARAM.h>

using namespace std;

float controller_param[3]={0};
int countT = 0;
task_planner_data::task_planner_data choice;

void controllerDataCallback(const task_planner_data::controller_dataConstPtr &msg)
{
    controller_param[0]=msg->param[0];
    controller_param[1]=msg->param[1];
    controller_param[2]=msg->param[2];
}

void taskTimerCallback(const ros::TimerEvent& e)
{
    countT++;
}

int main(int argc, char **argv)
{     
    TASKS_PARAM inputObj;
    ros::init(argc, argv,"task_planner");
    inputObj.readFromFile();
    inputObj.printVar();
    ros::NodeHandle n;
    ros::NodeHandle p;
    ros::Timer taskTimer = n.createTimer(ros::Duration(0.1), taskTimerCallback);
    ros::Subscriber contrller_data_callback = n.subscribe<task_planner_data::controller_data>("controller_data",5,controllerDataCallback);
    ros::Publisher tasksPub = p.advertise<task_planner_data::task_planner_data>("task_planner_data",5);
    //task_planner_data::task_planner_data choice;
    ros::Duration d = ros::Duration(SMALL_SLEEP,0);
    d.sleep();
    ros::Rate loopRate(10);
    int count_marker=0;

    for(int i=0;i<inputObj.task_schedule.size();i++)
    {
        switch(inputObj.task_schedule[i].second)
        {
                case START:
                    cout<<"STARTING VEHICLE"<<endl;
                    choice.task_choice = START;
                    tasksPub.publish(choice);
                    while(ros::ok())
                    {
                        ros::spinOnce();
                        if(countT >= inputObj.INTERVAL_VEHICLE_START * 10)
                        {
                            countT = 0;
                            break;
                        }
                        loopRate.sleep();
                    }
                    break;
                case MARKER:
                    cout<<"LOCKING MARKER"<<endl;
                    choice.task_choice = MARKER;
                    tasksPub.publish(choice);
                    d.sleep();
                    while(ros::ok())
                    {
                        ros::spinOnce();
                        cout<<"MARKER ANGLE"<<controller_param[0]<<endl;
                        if ((abs(controller_param[0])) < inputObj.MARKER_ANGLE_THRESHOLD && controller_param[0]!=0)
                        {
                            countT = 0;
                            count_marker++;
                            if (count_marker==1)
                                break;
                        }
                        loopRate.sleep();
                    }
                    break;

                case FOLLOW_MARKER:
                    cout<<"MARKER LOCKED!!! FOLLOWING YAW ANGLE"<<endl;
                    choice.task_choice = GO_HORIZONTAL;
                    choice.task_param  = 2;
                    tasksPub.publish(choice);
                    while(ros::ok())
                    {
                        ros::spinOnce();
                        if(countT >= inputObj.INTERVAL_MARKER_FOLLOWING * 10)
                        {
                            countT = 0;
                            break;
                        }
                        loopRate.sleep();
                    }
                    break;

                case GO_DOWN_MARKER:
                    cout<<"MARKER DONE!!! GOING DOWN"<<endl;
                    choice.task_choice = GO_DEPTH;
                    choice.task_param  = 0.4;
                    tasksPub.publish(choice);
                    while(ros::ok())
                    {
                        ros::spinOnce();
                        if(countT >= inputObj.INTERVAL_GO_DOWN_MARKER * 10)
                        {
                            countT = 0;
                            break;
                        }
                        loopRate.sleep();
                    }
                    break;
                case BUOY:
                    cout<<"BUOY TOUCHING"<<endl;
                    choice.task_choice = BUOY;
                    tasksPub.publish(choice);
                    d.sleep();
                    while(ros::ok())
                    {
                        ros::spinOnce();
                        cout<<"BUOY AREA:\t"<<controller_param[0]<<endl;
                        if(controller_param[0]>inputObj.BUOY_AREA_THRESHOLD)
                        {
                            countT = 0;
                            break;
                        }
                        loopRate.sleep();
                    }
                    break;

                case HIT_BUOY:
                    cout<<"HITTING BUOY"<<endl;
                    choice.task_choice = GO_HORIZONTAL;
                    choice.task_param = 2;
                    tasksPub.publish(choice);
                    while(ros::ok())
                    {
                        ros::spinOnce();
                        if(countT >= inputObj.INTERVAL_HIT_BUOY * 10)
                        {
                            countT = 0;
                            break;
                        }
                        loopRate.sleep();
                    }
                    break;

                case GO_BACK_BUOY:
                    cout<<"BUOY LOCKED RETREATING"<<endl;
                    choice.task_choice = GO_HORIZONTAL;
                    choice.task_param = -4;
                    tasksPub.publish(choice);
                    while(ros::ok())
                    {
                        ros::spinOnce();
                        if(countT >= inputObj.INTERVAL_GO_BACK_BUOY * 10)
                        {
                            countT = 0;
                            break;
                        }
                        loopRate.sleep();
                    }
                    break;

                 case GO_UP_BUOY:
                    cout<<"BUOY DONE!!! COMMING UP"<<endl;
                    choice.task_choice = GO_DEPTH;
                    choice.task_param  = 0.05;
                    tasksPub.publish(choice);
                    while(ros::ok())
                    {
                        ros::spinOnce();
                        if(countT >= inputObj.INTERVAL_GO_UP_BUOY * 10)
                        {
                            countT = 0;
                            break;
                        }
                        loopRate.sleep();
                    }
                    break;

                case GO_OVER_BUOY:
                    cout<<"GOING OVER BUOY"<<endl;
                    choice.task_choice = GO_HORIZONTAL;
                    choice.task_param  = 2;
                    tasksPub.publish(choice);
                    while(ros::ok())
                    {
                        ros::spinOnce();
                        if(countT >= inputObj.INTERVAL_GO_OVER_BUOY * 10)
                        {
                            countT = 0;
                            break;
                        }
                        loopRate.sleep();
                    }
                    break;
        }
    }


    cout<<"ALL TASKS DONE HAULT!!!"<<endl;
    choice.task_choice=100;
    tasksPub.publish(choice);
    return 0;
}
