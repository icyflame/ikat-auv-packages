#include <iptask/IpTask.h>

iptask::iptask(int camerainput)//:blob(), currentBlob()
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
        if(!frame)
            break;
        cvShowImage("My image",frame);
        c=cvWaitKey(33);
        if(c==27) break;
    }
    cvReleaseImage(&frame);
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
    cvReleaseImage(&frame);
    cvReleaseImage(&threshold);
}

void iptask::markerDetect(void)
{
    cvNamedWindow( "wallpaper",CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "wall",CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "wall1",CV_WINDOW_AUTOSIZE );
    marker_data data;
    IplImage * frame=cvQueryFrame(img);
    IplImage *imgBW,*imgHSV,*imgH,*imgS,*imgV;
    CBlobResult blobs;
    CBlob* currentBlob;
    CvSeq* contours;
    CvSeq* result;
    CvBox2D box;
    CvPoint2D32f * p = (CvPoint2D32f *)malloc(4*sizeof(CvPoint2D32f));
    CvMemStorage* storage = cvCreateMemStorage(0);
    CvPoint *p1,*p2,*p3,*p4;
    double l1=0,l2=0;
    float a1=0,maxArea=0;
    char c;
    imgBW= cvCreateImage(cvGetSize(frame),IPL_DEPTH_8U,1);
    imgH=cvCreateImage(cvGetSize(frame),frame->depth,1);
    imgS=cvCreateImage(cvGetSize(frame),frame->depth,1);
    imgV=cvCreateImage(cvGetSize(frame),frame->depth,1);
    imgHSV=cvCreateImage(cvGetSize(frame),IPL_DEPTH_8U,3);
    while (1)
    {
            frame=cvQueryFrame(img);
            if (!frame)
                break;
            maxArea=0,a1=0,l1=0,l2=0,data.angle=0,c=NULL,data.x_cor=0.0,data.y_cor=0.0;
            result=NULL,currentBlob=NULL,p1=NULL,p2=NULL,p3=NULL,p4=NULL;
            cvCvtColor(frame,imgHSV,CV_BGR2HSV_FULL);
            cvShowImage("original",imgHSV);
            cvInRangeS(imgHSV, cvScalar(5,0,0), cvScalar(50, 255, 255), imgBW);
            cvShowImage("wallpaper",imgBW);
            cvSmooth( imgBW, imgBW, CV_GAUSSIAN, 11, 11 );
            cvDilate(imgBW,imgBW);
            cvErode(imgBW,imgBW);
            blob = CBlobResult( imgBW, NULL,0);
            blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_LESS,100 );
            for (int i = 0; i < blobs.GetNumBlobs(); i++ )
            {
                    currentBlob = blobs.GetBlob(i);
                    currentBlob->FillBlob( imgBW,cvScalar(255));
            }
            cvShowImage("wall1",imgBW);
            cvCanny( imgBW, imgBW, 10, 100, 3 );
            cvFindContours( imgBW, storage,&contours, sizeof(CvContour),CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
            /*while(contours)
            {
                    result = cvApproxPoly( contours, sizeof(CvContour), storage,CV_POLY_APPROX_DP, cvContourPerimeter(contours)*0.02, 0 );
                    if( fabs(cvContourPerimeter(result))>500)
                    {

                            box = cvMinAreaRect2(result,NULL);
                            cvBoxPoints(box,p);
                            p1->x=p[0].x;p1->y=p[0].y;
                            p2->x=p[1].x;p2->y=p[1].y;
                            p3->x=p[2].x;p3->y=p[2].y;
                            p4->x=p[3].x;p3->y=p[3].y;
                            cvLine(frame, *p1, *p2, cvScalar(255,0,0),3,8);
                            cvLine(frame, *p2, *p3, cvScalar(255),3);
                            cvLine(frame, *p3, *p4, cvScalar(255),3);
                            cvLine(frame, *p4, *p1, cvScalar(255),3);
                            l1=(double)(p1->x-p2->x)*(p1->x-p2->x)+(p1->y-p2->y)*(p1->y-p2->y);
                            l2=(double)(p2->x-p3->x)*(p2->x-p3->x)+(p2->y-p3->y)*(p2->y-p3->y);
                            if (l1>l2)
                            {
                                    data.angle=atan(((p1->x-p2->x)*1.0)/(p1->y-p2->y));
                            }
                            else
                            {
                                    data.angle=atan( ((p2->x-p3->x)*1.0)/(p2->y-p3->y) );
                            }
                            data.angle*=-180/3.141;
                            data.x_cor=box.center.x;data.y_cor=box.center.y;
                            printf("Center= ( %f , %f ) Angle=%f\n",data.x_cor,data.y_cor,data.angle );

                    }
                    contours=contours->h_next;

            }*/
            c=cvWaitKey(33);
            if(c==27) break;
    }
    cvReleaseMemStorage(&storage);
    cvDestroyWindow( "wallpaper" );
    free(p);
}

iptask::~iptask()
{
    cvReleaseCapture(&img);
}

