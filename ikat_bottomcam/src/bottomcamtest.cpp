#include <ros/ros.h>
#include <iostream>
#include <ikat_bottomcam/ikat_bottomcam.h>
#include <task_planner_data/task_planner_data.h>
#include <task_planner/TASKS_PARAM.h>

using namespace std;

char currentTask;

void taskplannerCallback(const task_planner_data::task_planner_dataConstPtr &msg)
{
    currentTask = msg->task_choice;
    cout<<"callback bottom cam"<<currentTask<<endl;
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
    ros::Subscriber bcamSub = n.subscribe<task_planner_data::task_planner_data>("task_planner_data", 5, taskplannerCallback);
    BottomCam camera(atoi(argv[1]),"/home/ikat/ros_workspace/ikat-auv-packages/ikat_bottomcam/marker_threshold.txt","../bin_threshold.txt");
    bool firstTime=true;
    //currentTask = MARKER;
    string markervideo = "/home/ikat/ros_workspace/ikat-auv-packages/markerdetect_ipl/marker_1.avi";
    while(ros::ok())
    {
        ros::spinOnce();
        switch(currentTask)
        {
            case MARKER:
                    if(firstTime)
                    {
                        if(!camera.wakeCam())
                            ros::shutdown();
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
        if(!camera.I.empty())
            //cout << "Image is not there" << endl;
        //else
        {
            imshow("Bottom CAM", camera.I);
        }
        if(waitKey(200) == 27)
            break;
    }

}

