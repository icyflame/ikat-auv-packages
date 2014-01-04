#include <ikat_bottomcam/ikat_bottomcam.h>

BottomCam::BottomCam(int deviceIdno, const string &markerThresh, const string &binThresh)
{
    if(bottomcamera.open(deviceIdno))
    {
        cout << "The camera is opened Successfully" << endl;
        deviceId = deviceIdno;
        bottomcamera.set(CV_CAP_PROP_FRAME_WIDTH,640);
        bottomcamera.set(CV_CAP_PROP_FRAME_HEIGHT,480);
        bottomcamera.set(CV_CAP_PROP_FPS,5);
    }
    else
    {
        cout << "The camera did not started successfully" << endl;
        ros::shutdown();
    }
    /*ifstream fileMT(markerThresh.c_str());
    ifstream fileBT(binThresh.c_str());
    for(int i = 0; i < 6; i++)
    {
        fileMT >> thresholdValMarker[i];
        fileBT >> thresholdValBin[i];
    }*/
    markerData = n.advertise<ikat_ip_data::ip_marker_data>("MarkerData", 1);
    //binData = n.advertise<>
    elem = getStructuringElement(MORPH_RECT, Size(5,5));
    data = new markerDataStruct;
    data = NULL;
    bottomcamera >> I;
}

void BottomCam::sleepCam()
{
    bottomcamera.release();
}

bool BottomCam::wakeCam()
{
    return bottomcamera.open(deviceId);
}

void BottomCam::markerDetect()
{
    bottomcamera >> I;
    contours.clear();
    approxpoly.clear();
    Scalar threshmin(thresholdValMarker[0],thresholdValMarker[1],thresholdValMarker[3]), threshmax(thresholdValMarker[3],thresholdValMarker[4],thresholdValMarker[5]);
    cvtColor(I, I_hsv, CV_RGB2HSV_FULL);
    inRange(I_hsv, threshmin, threshmax, I_thresh);
    GaussianBlur(I_thresh, I_thresh, Size(11, 11), 0, 0);
    erode(I_thresh, I_thresh, elem);
    IplImage I_blob = I_thresh;
    blobs = CBlobResult(&I_blob, NULL, 0);
    cout << "Inside" << endl;
    blobs.Filter(blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, 1000);
    for(int i = 0; i < blobs.GetNumBlobs(); i++)
    {
        currentblob = blobs.GetBlob(i);
        currentblob->FillBlob(&I_blob, Scalar(255));
    }
    Mat I_proc(&I_blob,false);
    findContours(I_proc, contours,hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    if(!contours.empty())
    {
        if(data == NULL){}
        else
        {
            msg.Marker_data[0] = data->prev_center.x;
            msg.Marker_data[1] = data->prev_center.y;
            if (data->angle<-45)
                 msg.Marker_data[2]= 90 + data->prev_angle;
            else
                 msg.Marker_data[2] = data->prev_angle;
            msg.Marker_data[2]=-msg.Marker_data[2];
            markerData.publish(msg);
        }
    }
    else
    {
        for(int i = 0; i < contours.size(); i++)
        {
            if(contourArea(Mat(contours[i])) > 2000)
                approxPolyDP(contours[i], approxpoly[i], 5, true);
        }
        RotatedRect rectcontour;
        int maxlength = 10;
        for(int i = 0; i < approxpoly.size(); i++)
        {
            rectcontour = minAreaRect(approxpoly[i]);
            if(rectcontour.size.height > maxlength || rectcontour.size.width > maxlength)
            {
                data->center.x = rectcontour.center.x - bottomcamera.get(CV_CAP_PROP_FRAME_WIDTH)/2.0;
                data->center.y = rectcontour.center.y - bottomcamera.get(CV_CAP_PROP_FRAME_WIDTH)/2.0;
                data->size.x = rectcontour.size.width;
                data->size.y = rectcontour.size.height;
                data->angle = rectcontour.angle;
                //maxarea = rectcontour.size.height*rectcontour.size.width;
                msg.Marker_data[0] = data->center.x - bottomcamera.get(CV_CAP_PROP_FRAME_WIDTH)/2.0;
                msg.Marker_data[1] = data->center.y - bottomcamera.get(CV_CAP_PROP_FRAME_WIDTH)/2.0;
                if (data->angle<-45)
                      msg.Marker_data[2]= 90 + data->angle;
                else
                      msg.Marker_data[2]= data->angle;
                msg.Marker_data[2]=-msg.Marker_data[2];
                data->prev_center = data->center;
                data->prev_angle = data->angle;
            }
            Point2f rect_points[4];
            rectcontour.points(rect_points);
            for( int j = 0; j < 4; j++ )
                line(I, rect_points[j], rect_points[(j+1)%4], Scalar(255), 1, 8 );

        }
        markerData.publish(msg);
    }
}

void BottomCam::binDetect()
{

}

BottomCam::~BottomCam()
{
    bottomcamera.release();
}
