#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <ikat_ip_data/ip_marker_data.h>
#include <blob/BlobResult.h>

using namespace cv;
using namespace ros;

class BottomCam
{
private:
    VideoCapture bottomcamera;
    int deviceId;
    int thresholdValMarker[6];
    int thresholdValBin[6];
    NodeHandle n;
    Publisher markerData, binData;
    Mat I, I_hsv, I_thresh;
    MemStorage * storage;
public:
    BottomCam(int, const string &, const string &);
    void markerDetect();
    void binDetect();
    void sleepCam();
    bool wakeCam();
    ~BottomCam();
};
