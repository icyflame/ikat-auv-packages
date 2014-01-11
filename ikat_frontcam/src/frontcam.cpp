#include <ros/ros.h>
#include <iostream>
#include <ikat_frontcam/ikat_frontcam.h>
#include <task_planner_data/task_planner_data.h>
#include <task_planner/TASKS_PARAM.h>

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
    string pwd = "/home/madhukar/ros_workspace/ikat-auv-packages-svn/ikat-auv-packages/ikat_frontcam/";

    FrontCam camera(atoi(argv[1]), pwd + "buoy_threshold.txt", pwd + "torpedo_threshold.txt", pwd + "obstacle_threshold.txt");
    bool firstTime=true;
    currentTask = VGATE;
    //camera.wakeCam();
    string video = "/home/madhukar/videolog/forward-wire-11_13_58.avi";
    while(ros::ok())
    {
        ros::spinOnce();
        switch(currentTask)
        {
            case BUOY:
                    if (firstTime)
                    {
                        if(!camera.wakeCam())
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
            case VGATE:
                    if (firstTime)
                    {
                        if(!camera.wakeCam(video))
                            ros::shutdown();
                        firstTime=false;
                    }
                    camera.validationGate();
                    break;
            default:
                    camera.sleepCam();
                    firstTime=true;
                    break;
        }
        if(!camera.I.empty())
            //cout << "Image is not there" << endl;
        //else
        {
            imshow("front CAM", camera.I);
        }

        if(waitKey(200) == 27)
            break;
    }

}
