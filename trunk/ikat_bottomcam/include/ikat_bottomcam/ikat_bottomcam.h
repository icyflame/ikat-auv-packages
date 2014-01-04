#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <ikat_ip_data/ip_marker_data.h>
#include <blob/BlobResult.h>
#include <vector>
#include <fstream>

using namespace cv;
using namespace std;
using namespace ros;

struct markerDataStruct
{
    CvPoint2D32f size,center,prev_center;
    float angle,prev_angle;
};


class BottomCam
{
private:
    VideoCapture bottomcamera;
    int deviceId;
    int thresholdValMarker[6];
    int thresholdValBin[6];
    NodeHandle n;
    Publisher markerData, binData;
    Mat I_hsv, I_thresh, elem;
    MemStorage * storage;
    CBlobResult blobs;
    CBlob * currentblob;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    vector<vector<Point> > approxpoly;
    markerDataStruct * data;
    ikat_ip_data::ip_marker_data msg;
public:
    Mat I;
    BottomCam(int, const string &, const string &);
    void markerDetect();
    void binDetect();
    void sleepCam();
    void wakeCam();
    void wakeCam(const string &);
    ~BottomCam();
};
