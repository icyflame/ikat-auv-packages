#include <stab.h>
#include <ros/ros.h>
#include <iptask/IpTask.h>

int main(int argc,char** argv)
{
    ros::init(argc,argv,"IPtask");
    iptask camera(0);
    //camera.showimage();
    camera.markerDetect();
    camera.~iptask();
}
