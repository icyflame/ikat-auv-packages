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
    ifstream fileMT(markerThresh.c_str());
    ifstream fileBT(binThresh.c_str());
    for(int i = 0; i < 6; i++)
    {
        fileMT >> thresholdValMarker[i];
        fileBT >> thresholdValBin[i];
    }   
    markerData = n.advertise<ikat_ip_data::ip_marker_data>("MarkerData", 1);
    //binData = n.advertise<>
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
    Scalar threshmin(thresholdValMarker[0],thresholdValMarker[1],thresholdValMarker[3]), threshmax(thresholdValMarker[3],thresholdValMarker[4],thresholdValMarker[5]);
    
}
