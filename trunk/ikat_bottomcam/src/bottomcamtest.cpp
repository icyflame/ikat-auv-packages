#include <ros/ros.h>
#include <iostream>
#include <ikat_bottomcam/ikat_bottomcam.h>
#include <task_planner_data/task_planner_data.h>
#include <task_planner/Tasks.h>

using namespace std;

char currentTask;

void taskplannerCallback(const task_planner_data::task_planner_dataConstPtr &msg)
{
    currentTask = msg->task_choice;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "BottomCAM");
    if(argc!=2)
    {
        std::cout<<"Give Camera num"<<std::endl;
        exit(0);
    }
    ros::NodeHandle n;
    ros::Subscriber bcamSub = n.subscribe<task_planner_data::task_planner_data>("BottomCAMTask", 1, taskplannerCallback);
    BottomCam camera(atoi(argv[1]),"../marker_threshold.txt","../bin_threshold.txt");
    bool firstTime=true;
    currentTask = MARKER;
    while(ros::ok())
    {
        switch(currentTask)
        {
            case MARKER:
                    if (firstTime)
                    {
                        camera.wakeCam();
                        firstTime=false;
                    }
                    camera.markerDetect();
                    break;
            /*case BIN:
                    if (firstTime)
                    {
                        camera.wakeCam();
                        firstTime=false;
                    }
                    camera.binDetect();
                    break;*/
            default:
                    camera.sleepCam();
                    firstTime=true;
                    break;
        }

        imshow("Bottom CAM", camera.I);
        if(waitKey(200) == 27)
            break;
    }

}

