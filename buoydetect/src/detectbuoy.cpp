#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <iostream>
#include <blob/BlobResult.h>
#include <ikat_ip_data/ip_buoy_data.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fstream>

using namespace std;
using namespace cv;

int posx, posy;
bool mouseclick = false;

void mouseResponse(int event, int x, int y, int flags, void* param)
{
    if(event ==  CV_EVENT_LBUTTONDOWN)
    {
        posx = x;
        posy = y;
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
    //frontCam.open("/home/ikat/ros_workspace/ikat-auv-packages/grabVideo/CameraVideo1.avi");
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
    else
    {
        cout<<"camera opened "<<frontCam.get(CV_CAP_PROP_MODE)<<endl;
        frontCam.set(CV_CAP_PROP_FRAME_WIDTH,640);
        frontCam.set(CV_CAP_PROP_FRAME_HEIGHT,480);
        frontCam.set(CV_CAP_PROP_FPS,5);
    }
    bool contourdetected = false;
    ros::NodeHandle n;
    ros::Publisher buoy_Data = n.advertise<ikat_ip_data::ip_buoy_data>("BuoyData",1);
    ikat_ip_data::ip_buoy_data data;
    Mat I, Ihsv, Ithresh;
    std::ifstream file("/home/ikat/ros_workspace/ikat-auv-packages/buoydetect/th.txt");
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
    //namedWindow("Original Image");
    //setMouseCallback("Original Image", mouseResponse);




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
        //imshow("thresholded image", Ithresh);
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
    frontCam.release();
    char c;
    std::cin>>c;
    frontCam.open(atoi(argv[1]));
    while(1)
    {
        frontCam>>I;
        imshow("Front Camera", I);
        c = waitKey(200);
        if(c == 27)
            break;
    }

    return 0;




    /*int prev_error_x = 0;
    int prev_error_y = 0;
    while(ros::ok())
    {
        frontCam >> I;
        //imshow("Original Image", I);
        cvtColor(I, Ihsv,CV_RGB2HSV_FULL);
        // Threshold via mouse click Works better on IplImage
        if(mouseclick == true)
        {
            Vec3b data = Ihsv.at<uchar>(posx,posy);
            setThreshold(threshmin, threshmax,data);
            //cout << "Inside" << endl;
            cout << "Hue : "<<threshmin.val[0] << "\t Saturation : "<< threshmin.val[1] << "\t Value : "<< threshmin.val[2]<< endl;
            cout << "Hue : "<<threshmax.val[0] << "\t Saturation : "<< threshmax.val[1] << "\t Value : "<< threshmax.val[2]<< endl;
            mouseclick = false;
        }
        inRange(Ihsv, threshmin, threshmax, Ithresh);
        //imshow("Medfilt", Ithresh);
        //imshow("Threshimage", Ihsv);
        GaussianBlur(Ithresh, Ithresh, Size(11,11), 0, 0);
        medianBlur(Ithresh,Ithresh,9);
        IplImage Ithreshipl = Ithresh;
        // Exlcude white blob having very high area
        /*blobs = CBlobResult(&Ihsvipl, NULL, 0);
        blobs.Filter(blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, 10000);
        for (int i = 0; i < blobs.GetNumBlobs(); i++ )
        {
            currentBlob = blobs.GetBlob(i);
            currentBlob->FillBlob(&Ihsvipl,Scalar(0));
        }
        blobs.ClearBlobs();
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

        vector<vector<Point> > contours_poly(contours.size());
        vector<Point2f>center(contours.size());
        vector<Rect> boundRect(contours.size());
        vector<float>radius(contours.size());

        for( int i = 0; i < contours.size(); i++ )
        {
            approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
            boundRect[i] = boundingRect(Mat(contours_poly[i]));
            minEnclosingCircle((Mat)contours_poly[i], center[i], radius[i] );
        }

        //Mat drawing = Mat::zeros(Ifiltered.size(), CV_8UC3 );
        double circleArea, polyArea, rectArea;
        contourdetected = false;
        for( int i = 0; i< contours.size(); i++ )
        {
            if(radius[i] > 30)
            {
                polyArea = contourArea(contours_poly[i]);
                circleArea = 3.14*radius[i]*radius[i];
                rectArea = boundRect[i].area();
                // One of the method to be followed to filter out the unwanted circle
                /*if(circleArea > 0.70*rectArea && circleArea < 1.30*rectArea)
                {
                    Scalar color = Scalar(0, 255, 255);
                    drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
                    circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
                }

                if(center[i].y > 200)
                {
                    if(polyArea > 0.50*circleArea && polyArea < 1.3*circleArea)
                    {
                        Scalar color = Scalar(0, 255, 255);
                        drawContours( I, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
                        circle( I, center[i], (int)radius[i], color, 2, 8, 0 );
                    }
                    data.Buoy_Center_data[0] = -((float)center[i].x - I.size().width/2);
                    data.Buoy_Center_data[1] = -((float)center[i].y - I.size().height/2);
                    prev_error_x = data.Buoy_Center_data[0];
                    prev_error_y = data.Buoy_Center_data[1];
                    buoy_Data.publish(data);
                    contourdetected = true;
                }
            }
        }
        if(!contourdetected)
        {
            data.Buoy_Center_data[0] = prev_error_x;
            data.Buoy_Center_data[1] = prev_error_y;
            buoy_Data.publish(data);
        }
        imshow("Front Camera", I);
        char c = waitKey(200);
        if(c == 27)
            break;
    }
    return 0;*/
}
