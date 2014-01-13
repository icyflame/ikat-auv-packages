#include <ros/ros.h>
#include <iostream>
#include <time.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <ikat_sensor_data/depth_sensor_data.h>
#include <blob/BlobResult.h>
#include <vector>
#include <fstream>
#include <ikat_ip_data/ip_buoy_data.h>

using namespace ros;
using namespace cv;
using namespace std;

float depthData, initDepth;
bool initcheck = false;

void depthCallback(const ikat_sensor_data::depth_sensor_data::ConstPtr& msg)
{
    depthData = msg->depth;
    if(!initcheck)
    {
        initcheck = true;
        initDepth = msg->depth;
    }
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "buoyMLgrab");
    if(argc!=2)
    {
        std::cout<<"Give Camera num"<<std::endl;
        ros::shutdown();
    }

    string pwd = "/home/madhukar/videolog/forward-buoys-11_13_09.avi";
    VideoCapture frontCamera(pwd);
    //VideoCapture frontCamera(atoi(argv[0]));
    if(!frontCamera.isOpened())
    {
        std::cout << "The CAMERA was not opened properly" << std::endl;
        ros::shutdown();
    }
    VideoWriter recordCamera;
    int timeWait = 200;
    if(!recordCamera.open("/home/madhukar/videolog/bouyvideo_ml.avi", CV_FOURCC('D','I','V','X'), 1000/timeWait, Size(frontCamera.get(CV_CAP_PROP_FRAME_WIDTH), frontCamera.get(CV_CAP_PROP_FRAME_HEIGHT))))
    {
        std::cout << "The Recorder did not opened properly" << std::endl;
        ros::shutdown();
    }
    ros::NodeHandle n;
    ros::Subscriber depth_callback = n.subscribe("current_depth", 10,depthCallback);
    ros::Publisher buoy_Data = n.advertise<ikat_ip_data::ip_buoy_data>("BuoyData", 1);
    Mat I, Ihsv, Ithresh;
    CBlobResult blobs;
    CBlob * currentBlob;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    bool contourdetected;
    bool first_time;
    int prev_error_x;
    int prev_error_y;
    int thresholdValBuoy[6];

    string threshPath = "/home/madhukar/ros_workspace/ikat-auv-packages/ikat_frontcam/buoy_threshold.txt";
    ifstream fileBT(threshPath.c_str());
    for(int i = 0; i < 6; i++)
    {
        fileBT >> thresholdValBuoy[i];
    }

    ofstream fileML("/home/madhukar/buoyml.txt");

    ikat_ip_data::ip_buoy_data data;
    int count = 0;

    float areaRatio = 0;

    while(ros::ok())
    {
        Point2f prev_center_final;
        Scalar threshmin(thresholdValBuoy[0],thresholdValBuoy[1],thresholdValBuoy[2]), threshmax(thresholdValBuoy[3],thresholdValBuoy[4],thresholdValBuoy[5]);
        vector<vector<Point> > contours_poly;
        vector<Point2f>center;
        vector<Point>approx_poly_temp;
        float radius_temp;
        Point2f center_temp,center_final;
        vector<Point> contours_poly_final;
        double circleArea;
        contourdetected = false;
        frontCamera >> I;
        cvtColor(I, Ihsv,CV_RGB2HSV_FULL);
        inRange(Ihsv, threshmin, threshmax, Ithresh);
        imshow("Thresholded image",Ithresh);
        GaussianBlur(Ithresh, Ithresh, Size(11,11), 0, 0);
        medianBlur(Ithresh,Ithresh,9);
        IplImage Ithreshipl = Ithresh;
        // Exclude small clutters
        blobs = CBlobResult(&Ithreshipl, NULL, 0);
        blobs.Filter(blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, 700);
        for (int i = 0; i < blobs.GetNumBlobs(); i++ )
        {
            currentBlob = blobs.GetBlob(i);
            currentBlob->FillBlob(&Ithreshipl,Scalar(255));
        }
        Mat Ifiltered(&Ithreshipl);
        imshow("filtered", Ifiltered);
        // Finding those contour which resemble cicle
        findContours(Ifiltered, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
        for(int i = 0; i< contours.size(); i++ )
        {
            if(contourArea(contours[i]) > 500)
            {
                approxPolyDP( Mat(contours[i]), approx_poly_temp, 3, true );
                minEnclosingCircle((Mat)approx_poly_temp, center_temp, radius_temp );
                circleArea = 3.14*radius_temp*radius_temp;
                if (data.Buoy_area > 0.15)
                {
                    center.push_back(center_temp);
                    contours_poly.push_back(approx_poly_temp);
                }
                else
                {
                    if(contourArea(contours[i]) > 0.4*circleArea)
                    {
                        center.push_back(center_temp);
                        contours_poly.push_back(approx_poly_temp);
                    }
                }

            }
        }
        if (center.size())
        {
            for(int i=0; i< center.size(); i++)
            {
                if ((center_final.y<=center[i].y) || ( abs(center[i].y-prev_center_final.y)<200 && abs(center[i].x-prev_center_final.x)<400))
                {
                    center_final=center[i];
                    //can be commented in final code
                    contours_poly_final= contours_poly[i];
                    contourdetected = true;
                    prev_center_final=center_final;
                }
            }
            first_time=true;
        }
        if(!contourdetected)
        {
            data.Buoy_Center_data[0] = prev_error_x;
            data.Buoy_Center_data[1] = prev_error_y;
            data.Buoy_area=0;
        }
        else
        {
            minEnclosingCircle((Mat)contours_poly_final, center_final, radius_temp );
            circle(I,center_final,radius_temp,Scalar(255, 255, 255),5);

            data.Buoy_Center_data[0] = -((float)center_final.x - I.rows/2);
            data.Buoy_Center_data[1] = -((float)center_final.y - I.cols/2);
            prev_error_x = data.Buoy_Center_data[0];
            prev_error_y = data.Buoy_Center_data[1];
            data.Buoy_area=contourArea(contours_poly_final)/(I.rows*I.cols);
            areaRatio = data.Buoy_area;
            fileML << count << "\t" << depthData - initDepth << "\t" <<data.Buoy_Center_data[1] << "\t" << areaRatio << "\n";
            count++;
        }
        imshow("Final Image", I);
        if(waitKey(200) == 27)
            break;
        ros::spinOnce();
        buoy_Data.publish(data);
    }
}
