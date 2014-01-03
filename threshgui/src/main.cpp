#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace cv;
using namespace std;

int lowerH=0;
int lowerS=0;
int lowerV=0;

int upperH=180;
int upperS=256;
int upperV=256;

int x_co;
int y_co;
int temp=0;

IplImage* GetThresholdedImage(IplImage* imgHSV)
{
    IplImage* imgThresh=cvCreateImage(cvGetSize(imgHSV),IPL_DEPTH_8U, 1);
    cvInRangeS(imgHSV, cvScalar(lowerH,lowerS,lowerV), cvScalar(upperH,upperS,upperV), imgThresh);
    return imgThresh;
}

void setwindowSettings()
{
    cvNamedWindow("Video");
    cvNamedWindow("Ball");
    cvCreateTrackbar("LowerH", "Ball", &lowerH, 180, NULL);
    cvCreateTrackbar("UpperH", "Ball", &upperH, 180, NULL);
    cvCreateTrackbar("LowerS", "Ball", &lowerS, 256, NULL);
    cvCreateTrackbar("UpperS", "Ball", &upperS, 256, NULL);
    cvCreateTrackbar("LowerV", "Ball", &lowerV, 256, NULL);
    cvCreateTrackbar("UpperV", "Ball", &upperV, 256, NULL);
}

void CallBackFunc(int evt, int x, int y, int flags, void* param)
{
    if(evt==CV_EVENT_LBUTTONDOWN )
    {
        x_co=x;
        y_co=y;
        temp=1;
    }
}

int main(int argc,char** argv)
{

    CvCapture* capture =0;
    if(argc!=2)
    {
        std::cout<<"Give the camera num"<<std::endl;
        exit(0);
    }
    capture = cvCaptureFromCAM(atoi(argv[1]));
    //capture = cvCaptureFromFile("/home/madhukar/ros_workspace/ikat-auv-packages/iptask/src/downward-pipe-15_56_17.avi");
    if(!capture){
        printf("Capture failure\n");
        return -1;
    }

    IplImage* frame=0;
    setwindowSettings();
    while(true)
    {

        frame = cvQueryFrame(capture);
        if(!frame)  break;
        frame=cvCloneImage(frame);
        IplImage* imgHSV = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
        cvCvtColor(frame, imgHSV, CV_BGR2HSV_FULL);

        IplImage* imgThresh = GetThresholdedImage(imgHSV);
        setMouseCallback("Video", CallBackFunc,0);
        if(temp==1)
        {
            //printf("Current Position: x= %d y= %d \n",x_co,y_co);
            CvScalar val = cvGet2D(imgHSV, y_co, x_co);
            //cout<<"H= "<<val.val[0]<<"  S= "<<val.val[1]<<"  V= "<<val.val[2]<<endl;
            cvSetTrackbarPos("LowerH", "Ball",val.val[0]-15);
            cvSetTrackbarPos("LowerS", "Ball",val.val[1]-30);
            cvSetTrackbarPos("LowerV", "Ball",val.val[2]-50);
            cvSetTrackbarPos("UpperH", "Ball",val.val[0]+15);
            cvSetTrackbarPos("UpperS", "Ball",val.val[1]+30);
            cvSetTrackbarPos("UpperV", "Ball",val.val[2]+50);
            temp=0;
        }

        cvShowImage("Ball", imgThresh);
        cvShowImage("Video", frame);
        cvReleaseImage(&imgHSV);
        cvReleaseImage(&imgThresh);
        cvReleaseImage(&frame);

        int c = cvWaitKey(80);
        if((char)c==27 )
        {
            cout << "Lower (" << lowerH << ", " << lowerS << ", " << lowerV <<")"<< endl;
            cout << "Upper (" << upperH << ", " << upperS << ", " << upperV <<")"<< endl;
            break;
        }
    }
    cvDestroyAllWindows();
    cvReleaseCapture(&capture);
    return 0;
}
