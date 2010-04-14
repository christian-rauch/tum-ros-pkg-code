/* 
 * Copyright (c) 2009, Dejan Pangercic <pangercic@cs.tum.edu>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "cv_bridge/CvBridge.h" // CvBridge converts ros image_msgs to OpenCV images.
#include "opencv/cxcore.h" // OpenCV
#include "opencv/cv.h" // OpenCV
#include "opencv/highgui.h" // OpenCV's gui functions
#include "ros/node_handle.h"
//#include "topic_synchronizer/topic_synchronizer.h" // Synchronizes multiple messages so they arrive at the same time.
//#include <boost/thread.hpp>

class DisparityViewer
{
private:
  // Images in ROS
  sensor_msgs::ImageConstPtr left_image_ptr_;
  sensor_msgs::ImageConstPtr disparity_image_ptr_;
  
  // Converter from ROS images to OpenCV images
  sensor_msgs::CvBridge left_bridge_;
  sensor_msgs::CvBridge disparity_bridge_;
  
  // Node handle and subscribers
  ros::NodeHandle nh_;
  ros::Subscriber left_image_sub_;
  ros::Subscriber disparity_image_sub_;
  //TopicSynchronizer sync_; // Synchronizes the images
  
  // Image mutices
  //boost::mutex left_image_mutex_, disparity_image_mutex_;
  
public:
  DisparityViewer(const ros::NodeHandle& node_handle)
    : nh_(node_handle)
  {
    // Create OpenCV gui windows for the images.
    cvNamedWindow("Left");
    cvNamedWindow("Disparity");
    
    // Initialize the image synchronization
    //sync_(&DisparityViewer::imageCallback, this);
    // Subscribe to the images
    left_image_sub_ = nh_.subscribe("/stereo/left/image", 1, &DisparityViewer::leftImageCallback,this );
    disparity_image_sub_ = nh_.subscribe("/stereo/disparity", 1, &DisparityViewer::disparityImageCallback,this );
  }
  
  ~DisparityViewer()
  {
    // Destroy all of the OpenCV windows
    cvDestroyWindow("Left");
    cvDestroyWindow("Disparity");
  }
  
  // Left image callback. Keep a pointer to the left image.
  void leftImageCallback(const sensor_msgs::ImageConstPtr& image_ptr)
  {
    IplImage *left_iplimage;
    //boost::mutex::scoped_lock lock(left_image_mutex_);
    left_image_ptr_ =  image_ptr;
    if (left_bridge_.fromImage(*left_image_ptr_)) {
      ROS_INFO("left image\n");
      left_iplimage = left_bridge_.toIpl();
    }
    else {
      ROS_INFO("no left image\n");
      return;
    }

    // Use OpenCV's HighGUI to display the images.
    cvShowImage("Left", left_iplimage);

    cvWaitKey(2);
  }
  
  // Disparity image callback. Keep a pointer to the disparity image.
  void disparityImageCallback(const sensor_msgs::ImageConstPtr& image_ptr)
  {
    IplImage *disparity_iplimage;
    //boost::mutex::scoped_lock lock(disparity_image_mutex_);
    disparity_image_ptr_ = image_ptr;

    if (disparity_bridge_.fromImage(*disparity_image_ptr_)) {
      ROS_INFO("disp image\n");
      disparity_iplimage = disparity_bridge_.toIpl();
    }
    else {
      ROS_INFO("no disp image\n");
      return;
    }
    
    // Hack: convert the 16-bit disparity image to an 8-bit image so OpenCV can display it.
    IplImage *disparity_out = cvCreateImage(cvGetSize(disparity_iplimage), IPL_DEPTH_8U, 1);
    cvCvtScale(disparity_iplimage, disparity_out, 4.0/16.0);
    cvShowImage("Disparity", disparity_out);
    //cvShowImage("Disparity", disparity_iplimage);
    //
    cvReleaseImage(&disparity_out);

    cvWaitKey(4);
  }
  
  // Synchronized image callback. Displays the images.
  void imageCallback()
  {
    IplImage *left_iplimage, *disparity_iplimage;

    //boost::mutex::scoped_lock lock(left_image_mutex_);
    //boost::mutex::scoped_lock lock(disparity_image_mutex_);
    
    // Use CvBridge to get OpenCV versions of the images
    if (left_bridge_.fromImage(*left_image_ptr_,"bgr") && disparity_bridge_.fromImage(*disparity_image_ptr_)) {
      left_iplimage = left_bridge_.toIpl();
      disparity_iplimage = disparity_bridge_.toIpl();
    }
    else {
      return;
    }
    
    // Use OpenCV's HighGUI to display the images.
    cvShowImage("Left", left_iplimage);
    
    // Hack: convert the 16-bit disparity image to an 8-bit image so OpenCV can display it.
    IplImage *disparity_out = cvCreateImage(cvGetSize(disparity_iplimage), IPL_DEPTH_8U, 1);
    cvCvtScale(disparity_iplimage, disparity_out, 4.0/16.0);
    cvShowImage("Disparity", disparity_iplimage);
    
    cvWaitKey(2);
    
    cvReleaseImage(&disparity_out);
  }
  
};  // end class

int main(int argc, char **argv)
{
  //sleep(10000);

  ros::init(argc, argv, "disparity_viewer");
  ros::NodeHandle n;
  DisparityViewer viewer(n);
  ros::spin();
  return 0;
}


