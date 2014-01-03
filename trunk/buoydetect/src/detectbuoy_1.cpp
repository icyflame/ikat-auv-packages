
#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <iostream>
#include <blob/BlobResult.h>
#include <ikat_ip_data/ip_buoy_data.h>
#include <fstream>
using namespace std;
using namespace cv;

Point p;
bool mouseclick = false;

void mouseResponse(int event, int x, int y, int flags, void* param)
{
    if(event ==  CV_EVENT_LBUTTONDOWN)
    {
        p.x = x;
        p.y = y;
        mouseclick = true;
    }
}

void setThreshold(Scalar & threshmin, Scalar & threshmax, Vec3b data)
{
    threshmin.val[0] = data[0] - 20;
    if(threshmin.val[0] < 0)
        threshmin.val[0] = 0;
    threshmin.val[1] = 0;
    threshmin.val[2] = 0;
    threshmax.val[0] = data[0] + 20;
    if(threshmax.val[0] > 255)
        threshmax.val[0] = 255;
    threshmax.val[1] = 255;
    threshmax.val[2] = 255;
}


int main(int argc,char** argv)
{
    ros::init(argc,argv,"buoydetect");
    VideoCapture frontCam;
    //frontCam.open("/home/siddhartha/ros_workspace/CameraVideo1.avi");
    if(argc!=2)
    {
        std::cout<<"Give Camera num"<<std::endl;
        exit(0);
    }
    frontCam.open(atoi(argv[1]));
    if(!frontCam.isOpened())
    {
        cout << "The Camera is Not opened Properly" << endl;
        return 0;
    }
    bool contourdetected = false;
    ros::NodeHandle n;
    ros::Publisher buoy_Data = n.advertise<ikat_ip_data::ip_buoy_data>("BuoyData",1);
    ikat_ip_data::ip_buoy_data data;
    Mat I, Ihsv, Ithresh;
    std::ifstream file("/home/siddhartha/ros_workspace/ikat-auv-packages/buoydetect/th.txt");
    int a[6];
    file>>a[0];
    file>>a[1];
    file>>a[2];
    file>>a[3];
    file>>a[4];
    file>>a[5];
    Scalar threshmin(a[0],a[1],a[2]), threshmax(a[3],a[4],a[5]);
    CBlobResult blobs;
    CBlob * currentBlob;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    bool first_time=false;
    //namedWindow("Original Image");
    //setMouseCallback("Original Image", mouseResponse);
    int prev_error_x = 0;
    int prev_error_y = 0;
    Point2f prev_center_final;
    while(ros::ok())
    {

        vector<vector<Point> > contours_poly;
        vector<Point2f>center;
        vector<Point>approx_poly_temp;
        float radius_temp;
        Point2f center_temp,center_final;
        vector<Point> contours_poly_final;
        double circleArea;
        contourdetected = false;

        frontCam >> I;
        //imshow("Original Image", I);
        cvtColor(I, Ihsv,CV_RGB2HSV_FULL);
        // Threshold via mouse click Works better on IplImage
        //if(mouseclick == true)
        //{
        //    Vec3b data = Ihsv.at<uchar>(p);
        //    setThreshold(threshmin, threshmax,data);
        //    mouseclick = false;
        //}
        inRange(Ihsv, threshmin, threshmax, Ithresh);
        GaussianBlur(Ithresh, Ithresh, Size(11,11), 0, 0);
        medianBlur(Ithresh,Ithresh,9);
        imshow("thresholded image", Ithresh);
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
        // Finding those contour which resemble cicle
        findContours(Ifiltered, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
        for(int i = 0; i< contours.size(); i++ )
        {
            if(contourArea(contours[i]) > 2500)
            {
                approxPolyDP( Mat(contours[i]), approx_poly_temp, 3, true );
                minEnclosingCircle((Mat)approx_poly_temp, center_temp, radius_temp );
                circleArea = 3.14*radius_temp*radius_temp;
                if(contourArea(contours[i]) > 0.55*circleArea)
                {
                    center.push_back(center_temp);
                    contours_poly.push_back(approx_poly_temp);
                }
            }
        }
        //display all contours
        /*for(int i=0; i< contours.size(); i++)
        {

            minEnclosingCircle((Mat)contours[i], center_temp, radius_temp );
            circle(I,center_temp,radius_temp,Scalar(255, 255, 255),2);
            cout<<contourArea(contours[i])/3.14*radius_temp*radius_temp<<endl;
        }*/

        if (center.size())
        {
            //displaying all probable things
            /*for(int i=0; i< center.size(); i++)
            {

                minEnclosingCircle((Mat)contours_poly[i], center[i], radius_temp );
                circle(I,center[i],radius_temp,Scalar(255, 255, 255),2);
            }*/

            if (center.size()==1 && !first_time)
            {
                center_final= center[0];

                contours_poly_final= contours_poly[0];
                contourdetected = true;
                prev_center_final=center_final;
            }
            else
            {
                //finding the contour whose center is bottom most
                for(int i=0; i< center.size(); i++)
                {
                    if ((center_final.y<=center[i].y && !first_time) || (first_time && abs(center[i].y-prev_center_final.y)<200 && abs(center[i].x-prev_center_final.x)<400))
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


        }
        if(!contourdetected)
        {
            data.Buoy_Center_data[0] = prev_error_x;
            data.Buoy_Center_data[1] = prev_error_y;
            data.Buoy_area=0;
        }
        else
        {
            //can be commented in final code
            minEnclosingCircle((Mat)contours_poly_final, center_final, radius_temp );
            circle(I,center_final,radius_temp,Scalar(255, 255, 255),5);

            data.Buoy_Center_data[0] = -((float)center_final.x - I.size().width/2);
            data.Buoy_Center_data[1] = -((float)center_final.y - I.size().height/2);
            prev_error_x = data.Buoy_Center_data[0];
            prev_error_y = data.Buoy_Center_data[1];
            data.Buoy_area=contourArea(contours_poly_final)/(I.rows*I.cols);
        }

        buoy_Data.publish(data);

        imshow("Front Camera", I);
        char c = waitKey(200);
        if(c == 27)
            break;
    }
    return 0;
}
