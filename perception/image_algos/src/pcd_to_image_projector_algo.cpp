/* 
 * Copyright (c) 2010, Dejan Pangercic <dejan.pangercic@cs.tum.edu>, 
 Zoltan-Csaba Marton <marton@cs.tum.edu>, Nico Blodow <blodow@cs.tum.edu>
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


#include <image_algos/pcd_to_image_projector_algo.h>
#include "highgui.h"  

using namespace image_algos;

void  PCDToImageProjector::init (ros::NodeHandle& nh)
{
  nh_ = nh;
  nh_.param("origin", origin_, std::string("/RightEyeCalcOff"));
  nh_.param("child", child_, std::string("/map"));
  //nh_.param("input_image_topic", input_image_topic_, std::string("/cop/left/camera"));
  //nh_.param("input_cloud_topic", input_cloud_topic_, std::string("/swissranger_test/cloud_sr"));
  //nh_.param("output_cluster_topic", output_cluster_topic_, std::string("output_cluster"));
  //nh_.param("output_image_topic", output_image_topic_, std::string("image_with_projected_cluster"));
  nh_.param("focal_length", focal_length_, 0.00641331974023884);
  nh_.param("proj_center_x", proj_center_x_, 833.248646581603);
  nh_.param("proj_center_y", proj_center_y_, 661.107370424523);
  nh_.param("pix_size_x", pix_size_x_, 7.43100103980579e-06);
  nh_.param("pix_size_y", pix_size_y_, 7.4e-06);
  //image_sub_ = it_.subscribe(input_image_topic_, 1, &PointCloudToImageProjector::image_cb, this);
  //cloud_sub_= nh_.subscribe (input_cloud_topic_, 10, &PointCloudToImageProjector::cloud_cb, this);
  //image_pub_ = it_.advertise(output_image_topic_, 1);
}


void  PCDToImageProjector::pre  ()
{
  //  subimage_ = boost::shared_ptr <OutputType>(new OutputType ());
}


void  PCDToImageProjector::post ()
{
  cvReleaseImage(&subimage_);
  //Nothing TODO
}


std::vector<std::string>  PCDToImageProjector::requires ()
{
  return std::vector<std::string>();
}


std::vector<std::string>  PCDToImageProjector::provides ()
{
  return std::vector<std::string>();
}

void PCDToImageProjector::project_3D_point(const double &x, const double &y, const double &z, int &row, int &column)
{
  double temp1,temp2;
  if (focal_length_ > 0.0)
  {
    temp1 = focal_length_ * (x / z );
    temp2 = focal_length_ * (y / z );
  }
  else
  {
    temp1 = x;
    temp2 = y;
  }
  column = ( temp1 / pix_size_x_ ) + proj_center_x_;
  row = ( temp2 / pix_size_y_ ) + proj_center_y_;
}


std::string PCDToImageProjector::process (sensor_msgs::PointCloud &cloud_in, IplImage *image)
{
  try
  {
    tf_listener_.transformPointCloud(origin_, cloud_in, cloud_in);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("[PCDToImageProjector:] %s",ex.what());
  }
  ROS_INFO("[PCDToImageProjector:] cloud transformed to %s", origin_.c_str());
  //create a sequence storage for projected points 
  CvMemStorage* stor = cvCreateMemStorage (0);
  CvSeq* seq = cvCreateSeq (CV_SEQ_ELTYPE_POINT, sizeof (CvSeq), sizeof (CvPoint), stor);
  if (cloud_in.points.size() == 0)
  {
    ROS_WARN("[PCDToImageProjector:] Point cloud points with size 0!!! Returning.");
    return std::string("[PCDToImageProjector:] Point cloud points with size 0!!! Returning.");
  }
  for (unsigned long i = 0; i < cloud_in.points.size(); i++)
  {
    cv::Point3d pt_cv(cloud_in.points[i].x, cloud_in.points[i].y, cloud_in.points[i].z);
    CvPoint uv;
    project_3D_point(pt_cv.x, pt_cv.y, pt_cv.z, uv.y, uv.x);
    if (uv.x < 0 || uv.x > image->width)
    {
      ROS_WARN("[PCDToImageProjector:] width out of bounds");
      continue;
    }
    if (uv.y < 0 || uv.y > image->height)
    {
      ROS_WARN("[PCDToImageProjector:] height out of bounds");
      continue;
    }
    //static const int RADIUS = 3;
    //cvCircle(image, uv, RADIUS, CV_RGB(255,0,0), -1);
    cvSeqPush( seq, &uv );
  }
  
  CvRect rect = cvBoundingRect(seq);
  if (rect.width == 0 || rect.height == 0)
  {
    ROS_WARN("[PCDToImageProjector:] Rectangle's width = height = 0. This is impossible. Skipping ROI extraction.");
    return std::string("[PCDToImageProjector:] Rectangle's width = height = 0. This is impossible. Skipping ROI extraction.");
  }
  CvPoint pt1, pt2;
  pt1.x = rect.x;
  //check boundary, increase ROI for 10%
  if ((rect.x + (rect.width + 0.1*rect.width)) > image->width)
  {
    rect.width =  image->width - rect.x;
  }
  else 
    rect.width = rect.width + 0.1 * rect.width;
  
  pt2.x = rect.x + rect.width;
  pt1.y = rect.y;
  
  //check boundary, increase ROI for 10%
  if ((rect.y + (rect.height + 0.1*rect.height)) > image->height)
  {
    rect.height =  image->height - rect.y;
  }
  else
    rect.height = rect.height + 0.1 * rect.height;
  pt2.y = rect.y+rect.height;
  
  //draw rectangle around the points
  //cvRectangle(image, pt1, pt2, CV_RGB(255,0,0), 3, 8, 0 );
  cvClearMemStorage( stor );
  
  //get subimage_, aka region of interest
  
  cvSetImageROI(image,rect);
  // sub-image
  subimage_ = cvCreateImage( cvSize(rect.width, rect.height), image->depth, image->nChannels );
  cvCopy(image, subimage_);
  cvResetImageROI(image); // release image ROI
  //roi_ = bridge_.cvToImgMsg(subimage_)
  //ROS_INFO("[PCDToImageProjector:] image published to %s", output_image_topic_.c_str());
  return std::string ("ok");
}

std::string PCDToImageProjector::process (geometry_msgs::Polygon &cloud_in, IplImage *image)
{
  //create a sequence storage for projected points 
  CvMemStorage* stor = cvCreateMemStorage (0);
  CvSeq* seq = cvCreateSeq (CV_SEQ_ELTYPE_POINT, sizeof (CvSeq), sizeof (CvPoint), stor);
  if (cloud_in.points.size() == 0)
  {
    ROS_WARN("[PCDToImageProjector:] Point cloud points with size 0!!! Returning.");
    return std::string("[PCDToImageProjector:] Point cloud points with size 0!!! Returning.");
  }
  for (unsigned long i = 0; i < cloud_in.points.size(); i++)
  {
    //transform polygon to right camera frame
    geometry_msgs::PointStamped Pstamped_local;
    Pstamped_local.point.x = cloud_in.points[i].x;
    Pstamped_local.point.y = cloud_in.points[i].y;
    Pstamped_local.point.z = cloud_in.points[i].z;
    Pstamped_local.header.frame_id = child_;
    geometry_msgs::PointStamped Pstamped_global;
    try
    {
      tf_listener_.transformPoint(origin_, Pstamped_local, Pstamped_global);
      cloud_in.points[i].x = Pstamped_global.point.x;
      cloud_in.points[i].y = Pstamped_global.point.y;
      cloud_in.points[i].z = Pstamped_global.point.z;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("[PCDToImageProjector:] %s",ex.what());
      return ("PCDToImageProjector: failed polygon transform");
    }

    cv::Point3d pt_cv(cloud_in.points[i].x, cloud_in.points[i].y, cloud_in.points[i].z);
    CvPoint uv;
    project_3D_point(pt_cv.x, pt_cv.y, pt_cv.z, uv.y, uv.x);
    if (uv.x < 0 || uv.x > image->width)
    {
      ROS_WARN("[PCDToImageProjector:] width out of bounds");
      continue;
    }
    if (uv.y < 0 || uv.y > image->height)
    {
      ROS_WARN("[PCDToImageProjector:] height out of bounds");
      continue;
    }
    //static const int RADIUS = 3;
    //cvCircle(image, uv, RADIUS, CV_RGB(255,0,0), -1);
    cvSeqPush( seq, &uv );
  }
  CvRect rect = cvBoundingRect(seq);
  if (rect.width == 0 || rect.height == 0)
  {
    ROS_WARN("[PCDToImageProjector:] Rectangle's width = height = 0. This is impossible. Skipping ROI extraction.");
    return std::string("[PCDToImageProjector:] Rectangle's width = height = 0. This is impossible. Skipping ROI extraction.");
  }
  CvPoint pt1, pt2;
  pt1.x = rect.x;
  //check boundary, increase ROI for 10%
  if ((rect.x + (rect.width + 0.1*rect.width)) > image->width)
  {
    rect.width =  image->width - rect.x;
  }
  else 
    rect.width = rect.width + 0.1 * rect.width;
    
  pt2.x = rect.x + rect.width;
  pt1.y = rect.y;
    
  //check boundary, increase ROI for 10%
  if ((rect.y + (rect.height + 0.1*rect.height)) > image->height)
  {
    rect.height =  image->height - rect.y;
  }
  else
    rect.height = rect.height + 0.1 * rect.height;
  pt2.y = rect.y+rect.height;
    
  //draw rectangle around the points
  //cvRectangle(image, pt1, pt2, CV_RGB(255,0,0), 3, 8, 0 );
  cvClearMemStorage( stor );
    
  //get subimage_, aka region of interest
    
  cvSetImageROI(image,rect);
  // sub-image
  subimage_ = cvCreateImage( cvSize(rect.width, rect.height), image->depth, image->nChannels );
  cvCopy(image, subimage_);
  cvResetImageROI(image); // release image ROI
  //roi_ = bridge_.cvToImgMsg(subimage_)
  //ROS_INFO("[PCDToImageProjector:] image published to %s", output_image_topic_.c_str());
  return std::string ("ok");
}


PCDToImageProjector::OutputType PCDToImageProjector::output () 
{
  return subimage_;
};


#ifdef CREATE_NODE
int main (int argc, char* argv[])
{
  return standalone_node < PCDToImageProjector> (argc, argv);
}
#endif

