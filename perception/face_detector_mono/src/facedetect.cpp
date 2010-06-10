/*
 * Copyright (C) 2010 by Dejan Pangercic <dejan.pangercic@cs.tum.edu>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

 
#define CV_NO_BACKWARD_COMPATIBILITY
//opencv
#include "cv.h"
#include "highgui.h"

#include <iostream>
#include <cstdio>
//ros
#include <ros/ros.h>
#include <ros/node_handle.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"

//yarp
//#define YARP
#ifdef YARP
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
//namespaces
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace yarp::sig::file;
#endif

#ifdef _EiC
#define WIN32
#endif

using namespace std;
using namespace cv;

class FaceDetect {

public:
  String cascade_name_;
  String nested_cascade_name_;
  Mat  image_;
  String input_image_topic_, port_name_;
  CascadeClassifier cascade_, nested_cascade_;
  double scale_;
  bool display_, yarp_image_;
#ifdef YARP
  Network yarp_;
  // Make a port for reading and writing images
  BufferedPort<Bottle> port_;
#endif
  int hsizes_[2];
  CvHistogram *start_hist_;
  IplImage* hist_image_; 
  ros::Time ts_;

  FaceDetect(ros::NodeHandle &n) :
    n_(n), it_(n_)
  {
    n_.param("cascade_name", cascade_name_, std::string(""));
    n_.param("nested_cascade_name", nested_cascade_name_, std::string(""));
    n_.param("scale", scale_, 1.5);
    n_.param("input_image_topic", input_image_topic_, std::string("/yarp_to_ros_image/yarp_to_ros_image"));
    n_.param("display", display_, false);
    n_.param("yarp_image", yarp_image_, false);
    n_.param("port_name", port_name_, std::string("/icub_look_at_head"));
    ts_ = ros::Time::now ();

    image_sub_ = it_.subscribe(input_image_topic_, 1, &FaceDetect::image_callback, this);

    if (cascade_name_ == "" || nested_cascade_name_ == "")
      ROS_ERROR("Classification files missing!");
    hsizes_[0] = 128, hsizes_[1] = 128;
    start_hist_ = cvCreateHist(2, hsizes_, CV_HIST_ARRAY);
    hist_image_ = cvCreateImage(cvSize(30,30), IPL_DEPTH_8U, 1 );
    
    if(display_)
    {
      cvNamedWindow( "result", 1 );
      cvNamedWindow( "smallImg", 1 );
    }

#ifdef YARP    
    if(yarp_image_)
      {
	ConstString yarp_port_name_(port_name_.c_str());
	yarp_.init();
	port_.open(yarp_port_name_);
      }
#endif
  }

  ~FaceDetect()
    {
      if(display_)
      {
        cvDestroyWindow("result");
        cvDestroyWindow("smallImg");
      }
      cvReleaseImage(&hist_image_);
      cvReleaseHist(&start_hist_);
    }

  void image_callback(const sensor_msgs::ImageConstPtr& msg_ptr)
  {
    try
    {
      image_ = bridge_.imgMsgToCv(msg_ptr, "bgr8");
    }
    catch (sensor_msgs::CvBridgeException error)
    {
      ROS_ERROR("error");
    }
    
    if( !nested_cascade_.load( nested_cascade_name_ ) )
      cerr << "WARNING: Could not load classifier cascade for nested objects" << endl;
    
    if( !cascade_.load( cascade_name_ ) )
    {
      cerr << "ERROR: Could not load classifier cascade" << endl;
      cerr << "Usage: facedetect [--cascade=\"<cascade_path>\"]\n"
        "   [--nested-cascade[=\"nested_cascade_path\"]]\n"
        "   [--scale[=<image scale>\n"
        "   [filename|camera_index]\n" ;
      //return -1;
    }
    
    if( !image_.empty() )
    {
      detect_and_draw( image_, cascade_, nested_cascade_, scale_ );
      waitKey(3);
    }   
    
    // return 0;
  }

  void detect_and_draw( Mat& img,
                      CascadeClassifier& cascade, CascadeClassifier& nestedCascade,
                      double scale)
  {
    int i = 0;
    double t = 0;
    vector<Rect> faces;
    const static Scalar colors[] =  { CV_RGB(0,0,255),
                                      CV_RGB(0,128,255),
                                      CV_RGB(0,255,255),
                                      CV_RGB(0,255,0),
                                      CV_RGB(255,128,0),
                                      CV_RGB(255,255,0),
                                      CV_RGB(255,0,0),
                                      CV_RGB(255,0,255)} ;
    Mat gray, smallImg( cvRound (img.rows/scale), cvRound(img.cols/scale), CV_8UC1 );
    
    cvtColor( img, gray, CV_BGR2GRAY );
    resize( gray, smallImg, smallImg.size(), 0, 0, INTER_LINEAR );
    equalizeHist( smallImg, smallImg );

    t = (double)cvGetTickCount();
    cascade.detectMultiScale( smallImg, faces,
                              1.1, 5, 0
                              //|CV_HAAR_FIND_BIGGEST_OBJECT
                              //|CV_HAAR_DO_ROUGH_SEARCH
                              |CV_HAAR_SCALE_IMAGE
                              ,
                              Size(30, 30) );
    t = (double)cvGetTickCount() - t;
    ROS_INFO( "detection time = %g ms", t/((double)cvGetTickFrequency()*1000.) );
    for( vector<Rect>::const_iterator r = faces.begin(); r != faces.end(); r++, i++ )
    {
      Mat smallImgROI;
      vector<Rect> nestedObjects;
      Point center;
      Scalar color = colors[i%8];
      int radius;
      center.x = cvRound((r->x + r->width*0.5)*scale);
      center.y = cvRound((r->y + r->height*0.5)*scale);
#ifdef YARP 
      //send center coordinates to the icub head
      if(yarp_image_)
	send_face_center_to_yarp_port(center.x, center.y);
#else
      //send center coordinates with <geometry_msg::PointStamped> ????
#endif      
      radius = cvRound((r->width + r->height)*0.25*scale);
      circle( img, center, radius, color, 3, 8, 0 );
      
      //cvSetImageROI(smallImg, *r); 
      //cvCopy(smallImg, hist_image_);
      //cvResetImageROI(smallImg);
      smallImgROI = smallImg(*r);
      //cvCalcHist(smallImgROI, start_hist_);
      cv::imshow( "smallImg", smallImgROI );    
      
      if( nestedCascade.empty() )
        continue;
      smallImgROI = smallImg(*r);
      nestedCascade.detectMultiScale( smallImgROI, nestedObjects,
                                      1.1, 2, 0
                                      |CV_HAAR_FIND_BIGGEST_OBJECT
                                      //|CV_HAAR_DO_ROUGH_SEARCH
                                      //|CV_HAAR_DO_CANNY_PRUNING
                                      //|CV_HAAR_SCALE_IMAGE
                                      ,
                                      Size(30, 30) );
      for( vector<Rect>::const_iterator nr = nestedObjects.begin(); nr != nestedObjects.end(); nr++ )
      {
        center.x = cvRound((r->x + nr->x + nr->width*0.5)*scale);
        center.y = cvRound((r->y + nr->y + nr->height*0.5)*scale);
        
        radius = cvRound((nr->width + nr->height)*0.25*scale);
        circle( img, center, radius, color, 3, 8, 0 );
      }
    }  
    if(display_)
      cv::imshow( "result", img );    
  }
#ifdef YARP
  void send_face_center_to_yarp_port(double x, double y)
  {
    Bottle& output = port_.prepare();
    output.clear();
    output.addDouble(x);
    output.addDouble(y);
    ROS_INFO("Writting center coordinates %s to port %s. Time elapsed since last sending: %g seconds", 
	     output.toString().c_str(), port_name_.c_str(), (ros::Time::now () - ts_).toSec ());
    ts_ = ros::Time::now();
    port_.write();
  }
#endif

protected:
  
  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  sensor_msgs::CvBridge bridge_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "facedetect");
  ros::NodeHandle n("~");
  FaceDetect f(n);
  ros::spin();
  return 0;
}
