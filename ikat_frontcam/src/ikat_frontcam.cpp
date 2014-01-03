#include <iostream>
#include <ros/ros.h>
#include <fstream>
#include <string>
#include <ikat_frontcam/ikat_frontcam.h>

using namespace std;

FrontCam::FrontCam(int deviceIdno, const string &buoyThresh, const string &torpedoThresh, const string &vgateThresh)
{
    if(frontcamera.open(deviceIdno))
    {
        cout << "The camera is opened Successfully" << endl;
        deviceId = deviceIdno;
        frontcamera.set(CV_CAP_PROP_FRAME_WIDTH,640);
        frontcamera.set(CV_CAP_PROP_FRAME_HEIGHT,480);
        frontcamera.set(CV_CAP_PROP_FPS,5);
    }
    else
    {
        cout << "The camera did not started successfully" << endl;
        ros::shutdown();
    }
    ifstream fileBT(buoyThresh.c_str());
    ifstream fileMT(torpedoThresh.c_str());
    ifstream fileVT(vgateThresh.c_str());
    for(int i = 0; i < 6; i++)
    {
        fileBT >> thresholdValBuoy[i];
        fileMT >> thresholdValVgate[i];
        fileVT >> thresholdValTshoot[i];
    }
    buoy_Data =  n.advertise<ikat_ip_data::ip_buoy_data>("BuoyData",1);
    //valid_data
    //torpid_data
    contourdetected = false;
    // Buoy Specific
    first_time = false;
    prev_error_x = 0;
    prev_error_y = 0;    
}

void FrontCam::sleepCam()
{
    frontcamera.release();
}

void FrontCam::wakeCam()
{
    if(!frontcamera.open(deviceId))
    {
        cout << "The camera was not opened successfully" << endl;
    }
}

void FrontCam::buoyDetect()
{
    ikat_ip_data::ip_buoy_data data;
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
    frontcamera >> I;
    cvtColor(I, Ihsv,CV_RGB2HSV_FULL);
    inRange(Ihsv, threshmin, threshmax, Ithresh);
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
    if (center.size())
    {
        //displaying all probable things
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
    imshow("Front Camera", I);
    waitKey(200);
    buoy_Data.publish(data);
}
