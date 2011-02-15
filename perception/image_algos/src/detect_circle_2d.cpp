#include <ros/ros.h>
#include <ros/node_handle.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <string.h>
#include "math.h"
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <string.h>

using namespace std;

class DetectCircle2D 
{
public:
  int c_;
  int px_[0], py_[0], radius_;
  //for CvThreshold
  double threshold_, max_value_;
  //for CvSmooth
  int aperture_width_, aperture_height_;
  // for cvHoughCircles
  int accumulator_, canny_high_threshold_, accumulator_center_, min_radius_, max_radius_;
  int max_circles_;
  double sleep_;
  std::string image_topic_;
  CvMemStorage* cstorage_;
  DetectCircle2D(ros::NodeHandle &n) :
    n_(n), it_(n_)
  {
    px_[0] = 0; py_[0] = 0; radius_ = 0;
    cvNamedWindow("src",1);
    cvNamedWindow("gray",1);
    n_.param("threshold", threshold_, 100.00);
    n_.param("max_value", max_value_, 255.00);
    n_.param("aperture_height_", aperture_height_, 9);
    n_.param("aperture_width_", aperture_width_, 9);
    n_.param("accumulator", accumulator_, 2);
    n_.param("canny_high_threshold", canny_high_threshold_, 5);
    n_.param("accumulator_center", accumulator_center_, 35);
    n_.param("min_radius", min_radius_, 80);
    n_.param("max_radius", max_radius_, 150);
    n_.param("max_circles", max_circles_, 2);
    n_.param("sleep", sleep_, 0.1);
    n_.param("image_topic", image_topic_, std::string("/narrow_stereo/right/image_rect"));
    image_sub_ = it_.subscribe(
                               image_topic_, 1, &DetectCircle2D::imageCallback, this);
    ROS_INFO("DetectCircle2D Ready!!!");
    cstorage_ = cvCreateMemStorage(0);
  }

  ~DetectCircle2D()
  {
    //release all windows
    cvDestroyAllWindows();
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
    {
      
      IplImage *cv_image = NULL;
      try
      {
        cv_image = bridge_.imgMsgToCv(msg_ptr, "passthrough");
      }
      catch (sensor_msgs::CvBridgeException error)
      {
        ROS_ERROR("error");
      }

      IplImage *gray_ = NULL;
      gray_=cvCloneImage(cv_image);
      //color threshold
      cvThreshold(gray_, gray_, threshold_, max_value_, CV_THRESH_BINARY);
      //smooth the image to reduce unneccesary results
      cvSmooth( gray_, gray_, CV_GAUSSIAN, aperture_width_, aperture_height_);
      // cerr <<  accumulator_ << " " << gray_->height/5 << " " << canny_high_threshold_ << " " << accumulator_center_ << " " << min_radius_ << " " << max_radius_ << endl;
      //get circles
      CvSeq* circles =  cvHoughCircles( gray_, cstorage_, CV_HOUGH_GRADIENT, accumulator_, gray_->height/5, 
                                        canny_high_threshold_, accumulator_center_, min_radius_, max_radius_);
      //output all the circle detected
      ROS_INFO_STREAM("circles total: " << circles->total);
      //start drawing all the circles
      int i;
      for( i = 0; circles->total>=max_circles_?i<max_circles_:i < circles->total; i++ )
      { 
        //just make a filter to limit only <=2 circles to draw
        float* p = (float*)cvGetSeqElem( circles, i );
        cvCircle( cv_image, cvPoint(cvRound(p[0]),cvRound(p[1])), 3, CV_RGB(255,0,0), -1, 8, 0 );
        cvCircle( cv_image, cvPoint(cvRound(p[0]),cvRound(p[1])), cvRound(p[2]), CV_RGB(200,0,0), 1, 8, 0 );
        px_[i]=cvRound(p[0]); py_[i]=cvRound(p[1]); radius_ = cvRound(p[2]);
      }
      
      //output two circles' center position
      ROS_INFO_STREAM("center x: "<<px_[0]<<"  center y: "<<py_[0] << " radius: " << radius_);
      cvSetImageROI(gray_, cvRect(0,0,cv_image->width,cv_image->height));
      cvShowImage("gray",gray_);
      cvShowImage("src",cv_image);
      //release memory
      cvClearSeq(circles);
      cvReleaseImage(&gray_);
      cvClearMemStorage( cstorage_ );
      //ready to exit loop

      c_=cvWaitKey(10);
      if(c_==27)
        exit(2);
      sleep(sleep_);
    }
  
protected:
  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  sensor_msgs::CvBridge bridge_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detect_circle2D");
  ros::NodeHandle nh("~");
  DetectCircle2D dc(nh);
  ros::spin();
  return 0;
}
