#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <ikat_ip_data/ip_buoy_data.h>
#include <blob/BlobResult.h>
#include <vector>

using namespace cv;
using namespace std;
using namespace ros;

class FrontCam
{
private:
    VideoCapture frontcamera;
    int deviceId;
    int thresholdValBuoy[6];
    int thresholdValVgate[6];
    int thresholdValTshoot[6];
    int state;
    NodeHandle n;
    Publisher buoy_Data, valid_data, torpid_data;
    // Common Variables used by all the functions 
    Mat Ihsv, Ithresh;
    CBlobResult blobs;
    CBlob * currentBlob;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    bool contourdetected;
    // Buoy Specific
    bool first_time;
    int prev_error_x;
    int prev_error_y;
    //Validation Gate Specific
    vector<Vec4i> lines;
public:
    Mat I;
    FrontCam(int deviceId, const string &, const string &, const string &);
    void sleepCam();
    void buoyDetect();
    void validationGate();
    void torpedoShoot();
    bool wakeCam();
    bool wakeCam(const string &);
    ~FrontCam();
};
