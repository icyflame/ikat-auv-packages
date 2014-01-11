#include <ros/ros.h>
#include <iostream>
#include <ikat_frontcam/ikat_frontcam.h>
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
    ros::init(argc, argv, "frontCAM");
    if(argc!=2)
    {
        std::cout<<"Give Camera num"<<std::endl;
        exit(0);
    }
    ros::NodeHandle n;
    ros::Subscriber fcamSub = n.subscribe<task_planner_data::task_planner_data>("task_planner_data", 1, taskplannerCallback);
    FrontCam camera(atoi(argv[1]),"/home/siddhartha/ros_workspace/ikat-auv-packages/ikat_frontcam/buoy_threshold.txt","../torpedo_threshold.txt","../obstacle_threshold.txt");
    bool firstTime=true;
    currentTask = BUOY;
    string video = "/home/siddhartha/ros_workspace/CameraVideo.avi";
    while(ros::ok())
    {
        switch(currentTask)
        {
            case BUOY:
                    if (firstTime)
                    {
                        if(!camera.wakeCam(video))
                            ros::shutdown();
                        firstTime=false;
                    }
                    camera.buoyDetect();
                    break;
            /*case OBSTACLE:
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
        if(camera.I.empty())
            cout << "Image is not there" << endl;
        else
        {
            imshow("front CAM", camera.I);
            if(waitKey(33) == 27)
                break;
        }
    }

}
