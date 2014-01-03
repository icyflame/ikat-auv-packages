#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <iostream>
#include <blob/blob.h>
#include <blob/BlobResult.h>
#include <blob/BlobContour.h>
#include <blob/BlobLibraryConfiguration.h>
#include <blob/BlobOperators.h>
#include <blob/ComponentLabeling.h>
#include <blob/BlobProperties.h>
#include <ikat_ip_data/ip_marker_data.h>
#include <math.h>
#include <malloc.h>
#include <fstream>

using namespace std;
using namespace cv;

struct marker_Data
{
    CvPoint2D32f size,center,prev_center;
    float angle,prev_angle;
};


int main(int argc, char ** argv)
{
     ros::init(argc,argv,"markeripl");//vdeocaptute >>
     VideoCapture frontCam;
     //frontCam.open("/home/ikat/ros_workspace/ikat-auv-packages/markerdetect_ipl/marker_1.avi");
     //CvCapture * img = cvCaptureFromCAM(1);
     //cvCaptureFromFile("/home/ikat/ros_workspace/ikat-auv-packages/markerdetect_ipl/src/downward-pipe-15_56_17.avi");
     //if(img)

     if(argc!=2)
     {
         std::cout<<"Give Camera num"<<std::endl;
         exit(0);
     }
     frontCam.open(atoi(argv[1]));

     if(!frontCam.isOpened())
     {
         cout<<"camera not opened properly"<<endl;
     }
     else
     {
         cout<<"camera opened "<<frontCam.get(CV_CAP_PROP_MODE)<<endl;
         frontCam.set(CV_CAP_PROP_FRAME_WIDTH,640);
         frontCam.set(CV_CAP_PROP_FRAME_HEIGHT,480);
         frontCam.set(CV_CAP_PROP_FPS,5);
     }
     IplImage * frame,*img_hsv,*img_proc,* new1;
     CvMemStorage * storage = cvCreateMemStorage(0);
     ros::NodeHandle n;
     ros::Publisher marker = n.advertise<ikat_ip_data::ip_marker_data>("marker_data",3);
     //ros::Rate looprate(5);
     int count = 0;
     CvSeq * contours,*final_contour;
     int total_con;
     double maxarea,maxlength;
     marker_Data * Data =(marker_Data *)malloc(sizeof(marker_Data));
     CBlobResult blobs;
     CBlob * currentblob;
     CvPoint2D32f vertices[4];
     //CvCapture * img_video=cvCaptureFromAVI("downward-pipe-15_56_17.avi");
     Mat I;
     frontCam >> I;
     IplImage copy = I;
     frame=&copy;
     //frame=cvQueryFrame(img);

     //cvNamedWindow("Image Actual");
     cvNamedWindow("final Image");
     img_hsv=cvCreateImage(cvGetSize(frame),8,3);
     img_proc=cvCreateImage(cvGetSize(frame),8,1);
     new1=cvCreateImage(cvGetSize(frame),8,1);

     std::ifstream file("/home/ikat/ros_workspace/ikat-auv-packages/markerdetect_ipl/th.txt");
     int a[6];
     file>>a[0];
     file>>a[1];
     file>>a[2];
     file>>a[3];
     file>>a[4];
     file>>a[5];
     cout<<a[3]<<endl;
     while(ros::ok())
     {
         ikat_ip_data::ip_marker_data msg;
         IplImage * img_con=cvCreateImage(cvGetSize(frame),8,1);
         frontCam >> I;
         copy=I;
         frame=&copy;//cvQueryFrame(img);

         if(!frame)
                 break;
         //cvShowImage("Image Actual",frame);
         cvCvtColor(frame,img_hsv,CV_RGB2HSV_FULL);
         cvInRangeS(img_hsv,cvScalar(a[0],a[1],a[2]),cvScalar(a[3],a[4],a[5]),img_proc);
         cvSmooth(img_proc,img_proc,CV_GAUSSIAN,11,11);
         cvErode(img_proc,img_proc);
         //cvShowImage("thresholded Image",img_proc);
         blobs=CBlobResult(img_proc,NULL,0);
         //if (blobs!=NULL)
         {
         blobs.Filter(blobs,B_EXCLUDE,CBlobGetArea(),B_LESS,1000);
         for (int i = 0; i < blobs.GetNumBlobs(); i++ )
         {
                 currentblob = blobs.GetBlob(i);
                 currentblob->FillBlob(img_proc,cvScalar(255));
         }
         }
         cvCanny(img_proc,img_proc,10,200);
         total_con=cvFindContours(img_proc,storage,&contours,sizeof(CvContour),CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
         //cout<<contours->total<<endl;
         if(!contours)
         {


             msg.Marker_data[0]=Data->prev_center.x;
             msg.Marker_data[1]=Data->prev_center.y;
             if (Data->angle<-45)
                     msg.Marker_data[2]=90+Data->prev_angle;
             else
                     msg.Marker_data[2]=Data->prev_angle;
             msg.Marker_data[2]=-msg.Marker_data[2];
             marker.publish(msg);
             cvShowImage("final Image",frame);
             char c = waitKey(200);
             if(c == 27)
             {break;}
             continue;
         }

         final_contour=cvApproxPoly(contours,sizeof(CvContour),storage,CV_POLY_APPROX_DP,1,1);
         maxlength=10;
         maxarea=0;
         cvZero(img_con);
         CvBox2D rect;
         while(final_contour)
         {
              rect=cvMinAreaRect2(final_contour, storage);
              if(rect.size.height>maxlength || rect.size.width>maxlength)//rect.size.height*rect.size.width>maxarea)
              {
                  Data->center.x=rect.center.x-CV_CAP_PROP_FRAME_WIDTH/2.0;
                  Data->center.y=rect.center.y-CV_CAP_PROP_FRAME_HEIGHT/2.0;
                  Data->size.x=rect.size.width;
                  Data->size.y=rect.size.height;
                  Data->angle=rect.angle;
                  maxarea=rect.size.height*rect.size.width;
                  msg.Marker_data[0]=Data->center.x-frame->width/2.0;
                  msg.Marker_data[1]=Data->center.y-frame->height/2.0;
                  if (Data->angle<-45)
                          msg.Marker_data[2]=90+Data->angle;
                  else
                          msg.Marker_data[2]=Data->angle;
                  msg.Marker_data[2]=-msg.Marker_data[2];
                  //msg.Marker_data[2]=Data->angle;
                  Data->prev_center=Data->center;
                  Data->prev_angle=Data->angle;

              }
              final_contour=final_contour->h_next;
         }
         cvBoxPoints(rect,vertices);
         cvLine(frame,cvPointFrom32f(vertices[0]),cvPointFrom32f(vertices[1]),cvScalarAll(255),2);
         cvLine(frame,cvPointFrom32f(vertices[1]),cvPointFrom32f(vertices[2]),cvScalarAll(255),2);
         cvLine(frame,cvPointFrom32f(vertices[2]),cvPointFrom32f(vertices[3]),cvScalarAll(255),2);
         cvLine(frame,cvPointFrom32f(vertices[3]),cvPointFrom32f(vertices[0]),cvScalarAll(255),2);
         //ROS_INFO("center x :[%f]",msg.Marker_data[0]);
         //ROS_INFO("center y :[%f]",msg.Marker_data[1]);
         //ROS_INFO("angle : [%f]",msg.Marker_data[2]);
         marker.publish(msg);
         cvShowImage("final Image",frame);
         //ros::spinOnce();
         char c = waitKey(200);
         if(c == 27)
             break;
         //++count;
         //looprate.sleep();
     }
     //cvDestroyWindow("Image Actual");
     cvDestroyWindow("final Image");
     return 0;
}
