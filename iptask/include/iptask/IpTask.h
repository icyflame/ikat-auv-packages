#include <ros/ros.h>
#include <std_msgs/String.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <blob/blob.h>
#include <blob/BlobResult.h>
#include <blob/BlobContour.h>
#include <blob/BlobLibraryConfiguration.h>
#include <blob/BlobOperators.h>
#include <blob/ComponentLabeling.h>
#include <blob/BlobProperties.h>


typedef struct
{
    CvPoint2D32f size,center;
    float angle;
}marker_data;

class iptask
{
private:
    CvCapture *img;
public:
    iptask(int);
    int task_manager(int);
    void markerDetect(void);
    void showimage(void);
    void threshold(CvScalar,CvScalar);
    void validationGate(void);
    ~iptask();
};
