#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argv, char ** argc)
{
    ros::init(argv,argc,"GrabVideo");
    VideoCapture camera(0);
    VideoWriter recorder;
    if(!camera.isOpened())
        cout << "The camera was not opened properly" << endl;
    int framewidth = camera.get(CV_CAP_PROP_FRAME_WIDTH);
    int frameheight = camera.get(CV_CAP_PROP_FRAME_HEIGHT);
    cout << "Camera Width : " << framewidth << "\t Height : " << frameheight << endl;
    int fps = 25;
    cout << "Video FPS : " << fps << endl;
    Mat frame;
    recorder.open("/home/ikat/ros_workspace/ikat-auv-packages/grabVideo/CameraVideo1.avi",CV_FOURCC('D','I','V','X'), fps,Size(framewidth, frameheight));
    if(!recorder.isOpened())
    {
        cout << "The recorder was not opened successfully" << endl;
        return 0;
    }
    while(ros::ok())
    {
        camera >> frame;
        recorder << frame;
        imshow("Input Video" , frame);
        if(waitKey(1) == 27)
            break;
    }
    return 0;
}
