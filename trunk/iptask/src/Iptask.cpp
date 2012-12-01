#include <iptask/IpTask.h>
#include <ikat_ip_data/ip_marker_data.h>
#include <ikat_ip_data/ip_validation_data.h>
#include <iostream>

using namespace std;

iptask::iptask(int camerainput)
{
    img = cvCaptureFromCAM(camerainput);
}

void iptask::showimage()
{
    char c;
    cvNamedWindow("My image",CV_WINDOW_AUTOSIZE);
    IplImage * frame = cvQueryFrame(img);
    while(1)
    {
        frame=cvQueryFrame(img);
        if(!frame) break;
        cvShowImage("My image",frame);
        c=cvWaitKey(33);
        if(c==27) break;
    }
    cvDestroyWindow("My image");
}

void iptask::threshold(CvScalar range1,CvScalar range2)
{
    IplImage * frame=cvQueryFrame(img);
    IplImage * threshold;
    threshold = cvCreateImage(cvGetSize(frame),frame->depth,1);
    while(1)
    {
        frame=cvQueryFrame(img);
        cvCvtColor(frame,frame,CV_BGR2HSV_FULL);
        cvInRangeS(frame,range1,range2,threshold);
        cvShowImage("Thresholded",threshold);
        char c=cvWaitKey(33);
        if(c==27) break;
    }
    cvDestroyWindow("Thresholded");
    cvReleaseImage(&threshold);
}

void iptask::markerDetect(void)
{
     IplImage * frame,*img_hsv,*img_proc,* new1;
     CvMemStorage * storage = cvCreateMemStorage(0);
     ros::NodeHandle n;
     ros::Publisher marker = n.advertise<ikat_ip_data::ip_marker_data>("marker_data",3);
     ros::Rate looprate(5);
     int count = 0;
     CvSeq * contours,*final_contour;
     int total_con;
     double maxarea;
     marker_data * Data =(marker_data *)malloc(sizeof(marker_data));
     CBlobResult blobs;
     CBlob * currentblob;
     CvPoint2D32f vertices[4];
     //CvCapture * img_video=cvCaptureFromAVI("downward-pipe-15_56_17.avi");
     frame=cvQueryFrame(img);
     cvNamedWindow("Image Actual");
     cvNamedWindow("final Image");
     img_hsv=cvCreateImage(cvGetSize(frame),8,3);
     img_proc=cvCreateImage(cvGetSize(frame),8,1);
     new1=cvCreateImage(cvGetSize(frame),8,1);
     while(ros::ok())
     {
         ikat_ip_data::ip_marker_data msg;
         IplImage * img_con=cvCreateImage(cvGetSize(frame),8,1);
         frame=cvQueryFrame(img);
         if(!frame)
                 break;
         cvShowImage("Image Actual",frame);
         cvCvtColor(frame,img_hsv,CV_RGB2HSV);
         cvInRangeS(img_hsv,cvScalar(100,100,100),cvScalar(120,170,255),img_proc);
         cvSmooth(img_proc,img_proc,CV_GAUSSIAN,11,11);
         cvErode(img_proc,img_proc);
         blobs=CBlobResult(img_proc,NULL,0);
         blobs.Filter(blobs,B_EXCLUDE,CBlobGetArea(),B_LESS,75);
         for (int i = 0; i < blobs.GetNumBlobs(); i++ )
         {
                 currentblob = blobs.GetBlob(i);
                 currentblob->FillBlob(img_proc,cvScalar(255));
         }
         cvCanny(img_proc,img_proc,10,200);
         total_con=cvFindContours(img_proc,storage,&contours,sizeof(CvContour),CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
         final_contour=cvApproxPoly(contours,sizeof(CvContour),storage,CV_POLY_APPROX_DP,1,1);
         maxarea=0;
         cvZero(img_con);
         CvBox2D rect;
         while(final_contour)
         {
              rect=cvMinAreaRect2(final_contour, storage);
              if(rect.size.height*rect.size.width>maxarea)
              {
                  Data->center.x=rect.center.x;
                  Data->center.y=rect.center.y;
                  Data->size.x=rect.size.width;
                  Data->size.y=rect.size.height;
                  Data->angle=rect.angle;
                  maxarea=rect.size.height*rect.size.width;
                  msg.Marker_data[0]=Data->center.x;
                  msg.Marker_data[1]=Data->center.y;
                  msg.Marker_data[2]=Data->angle;
              }
              final_contour=final_contour->h_next;
         }
         cvBoxPoints(rect,vertices);
         cvLine(frame,cvPointFrom32f(vertices[0]),cvPointFrom32f(vertices[1]),cvScalarAll(255),2);
         cvLine(frame,cvPointFrom32f(vertices[1]),cvPointFrom32f(vertices[2]),cvScalarAll(255),2);
         cvLine(frame,cvPointFrom32f(vertices[2]),cvPointFrom32f(vertices[3]),cvScalarAll(255),2);
         cvLine(frame,cvPointFrom32f(vertices[3]),cvPointFrom32f(vertices[0]),cvScalarAll(255),2);
         ROS_INFO("center x :[%f]",msg.Marker_data[0]);
         ROS_INFO("center y :[%f]",msg.Marker_data[1]);
         ROS_INFO("angle : [%f]",msg.Marker_data[2]);
         marker.publish(msg);
         cvShowImage("final Image",frame);
         char c=cvWaitKey(33);
         if  (c==27)
         break;
         ros::spinOnce();
         ++count;
         looprate.sleep();
     }
     free(Data);
}

void iptask::validationGate()
{
    IplImage * img_RGB,*img_hsv,*final;
    img_RGB=cvQueryFrame(img);
    int depth,channels,coordinate[8]={0};
    ikat_ip_data::ip_validation_data error;
    depth=img_RGB->depth;
    channels=img_RGB->nChannels;
    img_hsv=cvCreateImage(cvGetSize(img_RGB),IPL_DEPTH_8U,channels);
    final=cvCreateImage(cvGetSize(img_RGB),depth,1);
    cvCvtColor(img_RGB,img_hsv,CV_RGB2HSV);
    CvSeq * contours;
    CvPoint rodA,rodB,rodC,center;
    CvMemStorage * storage=cvCreateMemStorage(0);
    center.x=img_RGB->width/2;
    center.y=img_RGB->height/2;
    ros::NodeHandle n;
    ros::Publisher validation_gate = n.advertise<ikat_ip_data::ip_validation_data>("validation_data",5);
    ros::Rate looprate(5);
    int count = 0;
    while(ros::ok())
    {
            img_RGB=cvQueryFrame(img);
            if(!img_RGB)
                 break;
            cvShowImage("Camera Input",img_RGB);
            cvCvtColor(img_RGB,img_hsv,CV_RGB2HSV);
            cvInRangeS(img_hsv,cvScalar(60,50,0),cvScalar(95,255,255),final);
            cvSmooth(final,final,CV_MEDIAN,7,7);
            contours = cvHoughLines2(final, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180,120, 10);
            for( int i = 0; i < contours->total; i++ )
            {
                    CvPoint* line = (CvPoint*)cvGetSeqElem(contours,i);
                    if(i==0)
                    {
                            coordinate[0]=line[0].x;			//taking min x
                            coordinate[3]=line[0].y;			//taking min y
                            coordinate[4]=line[1].x;			//taking max x
                            coordinate[7]=line[1].y;			//taking max y
                    }
                    if(line[0].x < coordinate[0])
                    {
                            coordinate[0]=line[0].x;
                            coordinate[1]=line[0].y;
                    }
                    if(line[1].x > coordinate[4])
                    {
                            coordinate[4]=line[1].x;
                            coordinate[5]=line[1].y;
                    }
                    if(line[0].x > coordinate[4])
                    {
                            coordinate[4]=line[0].x;
                            coordinate[5]=line[0].y;
                    }
                    if(line[1].x < coordinate[0])
                    {
                            coordinate[0]=line[1].x;
                            coordinate[1]=line[1].y;
                    }
                    if(line[0].y < coordinate[3])
                    {
                            coordinate[2]=line[0].x;
                            coordinate[3]=line[0].y;
                    }
                    if(line[1].y > coordinate[7])
                    {
                            coordinate[6]=line[1].x;
                            coordinate[7]=line[1].y;
                    }
                    if(line[0].y > coordinate[7])
                    {
                            coordinate[6]=line[0].x;
                            coordinate[7]=line[0].y;
                    }
                    if(line[1].y < coordinate[3])
                    {
                            coordinate[2]=line[1].x;
                            coordinate[3]=line[1].y;
                    }
            }
            rodA.x=coordinate[0];
            rodA.y=coordinate[1];
            rodC.x=coordinate[2];
            rodC.y=coordinate[3];
            cvLine(img_RGB,rodA,rodC,cvScalar(255,0,0),3,8);
            rodB.x=(rodA.x+rodC.x)/2;
            rodB.y=(rodA.y+rodC.y)/2;
            std::cout<<rodB.x<<"\t"<<rodB.y<<"\t"<<center.x<<"\t"<<center.y<<"\t"<<std::endl;
            error.errorx=center.x-rodB.x;
            error.errory=center.y-rodB.y;
            validation_gate.publish(error);
            cvShowImage("Final",img_RGB);
            char c=cvWaitKey(33);
            if(c==27)
                    break;
            count++;
            looprate.sleep();

    }
}

iptask::~iptask()
{
    cvReleaseCapture(&img);
}

