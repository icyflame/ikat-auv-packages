#include <stdio.h>
#include <ros/ros.h>
#include <iptask/IpTask.h>

int main(int argc,char** argv)
{
    ros::init(argc,argv,"IPtask");
    iptask camera(0);
    camera.showimage();
    CvScalar range1,range2;
    range1=cvScalar(30,0,0);
    range2=cvScalar(50,255,255);
    camera.threshold(range1,range2);
    //camera.markerDetect();
    camera.~iptask();
}
