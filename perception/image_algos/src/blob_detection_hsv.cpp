 #include "iostream"  
 #include "stdlib.h"  
 #include "stdio.h"  
 #include "cv.h"  
 #include "highgui.h"  
 #include "cstring"
#include "ros/ros.h"

 /*This program is not done in CVBlob library*/  
 /*Purpose of the program to track the blob */  
using namespace std;  



/*Note:The detection algorithm is the same as the one you can find in one of my previous posts 
 The same algorithm is copied from that post and pasted in this 
 post only the values of the "sthreshold" and the "hupper" and the "hlower" are different....*/  

int main(int argc, char *argv[])    
{
  /*You may change the values of the sthreshold and hupper to get different results....
    For red blob for instance this value is to be between 175 and 185, that is 175 and 5
    in openCV notation.*/  
  int sthreshold;  
  double hupper, hlower;  

  if (argc != 5)
    {
      ROS_INFO("Usage %d <file_name>, <saturation threshold>, <lower hue value>, <upper hue value>", argv[0]);
    exit(0);
    }
  else
    {
      string image(argv[1]);
      sthreshold=atoi(argv[2]);
      hlower=atoi(argv[3]);
      hupper=atoi(argv[4]);
    }

  string image(argv[1]);
  //for iterations  
  int i,j,k;
  int height,width,step,channels;
  int stepmono,channelsmono;
  uchar *data,*datamono;  
  i=j=k=0;
  IplImage *frame=cvLoadImage(image.c_str(),1);
  //resultant image
  IplImage *result = cvCreateImage(cvGetSize(frame), 8, 1);
  //hue channel image
  IplImage* hue = cvCreateImage( cvGetSize(frame), 8, 1 );

  //allocations
  height = frame->height;  
  width = frame->width;  
  step = frame->widthStep;  
  channels = frame->nChannels;    
  datamono = (uchar *) result->imageData;
  stepmono = result->widthStep;
  channelsmono = result->nChannels;
 
  /* create  windows */  
 cvNamedWindow("monoimage", CV_WINDOW_AUTOSIZE);  
 cvNamedWindow("hsvimage", CV_WINDOW_AUTOSIZE);  
 //cvNamedWindow("original frame", CV_WINDOW_AUTOSIZE);  
 //cvNamedWindow("hue", CV_WINDOW_AUTOSIZE);
  
 IplImage *imageHSV = cvCloneImage(frame);
 /*Converting the color space of the image....*/  
 cvCvtColor(frame,imageHSV,CV_BGR2HSV);
 cvSplit(imageHSV, hue, 0, 0, 0);

 cvZero(result);
 data = (uchar *)imageHSV->imageData;  

 for (i = 0; i < height; i++) {
   for (j = 0; j < width; j++) {
     if( (data[(i) * step + j * channels] <= hupper) &&
         (data[(i) * step + j * channels] >= hlower) &&
         (data[(i) * step + j * (channels) + 1] > sthreshold) ) {
       datamono[(i) * stepmono + j * channelsmono] = 255;
     }
   }
 }

 //see: http://www.dca.fee.unicamp.br/dipcourse/html-dip/c9/s4/front-page.html
 cvErode(result,result,0,01);
 cvDilate( result,result,0,01); 

 //cvShowImage("original frame", frame);  
 cvShowImage("hsvimage", imageHSV);  
 cvShowImage("monoimage", result);  
 //cvShowImage("hue", hue);  

 while(true)
   {
     if( cvWaitKey(10) == 27 ) 
       {
	 /* free memory */  
	 //cvDestroyWindow("original frame");  
	 cvDestroyWindow("monoimage");  
	 cvDestroyWindow("hsvimage");  
	 //cvDestroyWindow("hue");  
	 cvReleaseImage(&hue);
	 cvReleaseImage(&imageHSV);
	 cvReleaseImage(&frame);
	 cvReleaseImage(&result);
	 break;
       }
   }
 return 0;
}
