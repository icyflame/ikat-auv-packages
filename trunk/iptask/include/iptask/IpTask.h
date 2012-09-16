#include <ros/ros.h>
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
    float angle;
    int x_cor,y_cor;
}marker_data;

class iptask
{
private:
    CvCapture *img ;
    marker_data data;
    CBlobResult blob;
    CBlob* currentBlob;

public:
    iptask(int);
    int task_manager(int);
    void markerDetect(void);
    void showimage(void);
    void threshold(CvScalar,CvScalar);
    ~iptask();
};
