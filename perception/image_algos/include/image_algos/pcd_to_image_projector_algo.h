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

#ifndef IMAGE_ALGOS_PCD_TO_IMAGE_PROJECTOR_H
#define IMAGE_ALGOS_PCD_TO_IMAGE_PROJECTOR_H
#include <fstream>
#include <image_algos/image_algos.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Polygon.h>
//boost
#include <boost/thread/mutex.hpp>

namespace image_algos
{

class PCDToImageProjector : public ImageAlgo
{
public:
  PCDToImageProjector () 
  { 
  };
  typedef sensor_msgs::Image InputType;
  typedef sensor_msgs::PointCloud InputTypeII;

  typedef IplImage* OutputType;

  static std::string default_input_topic ()
    {return std::string ("image");}

  static std::string default_output_topic ()
    {return std::string ("roi");};

  static std::string default_node_name () 
    {return std::string ("pcd_to_image_projector_node");};

  void init (ros::NodeHandle&);
  void pre  ();
  void post ();
  std::vector<std::string> requires ();
  std::vector<std::string> provides ();
  //std::string process (const boost::shared_ptr<const InputType>);
  void project_3D_point(const double &x, const double &y, const double &z, int &row, int &column);
  std::string process (sensor_msgs::PointCloud &cloud_in, IplImage *image);
  std::string process (geometry_msgs::Polygon &cloud_in, IplImage *image);
  OutputType output ();

  //protected:
  //subscribers/publishers
  ros::NodeHandle nh_;
  image_transport::Publisher image_pub_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber cloud_sub_;
  
  tf::TransformListener tf_listener_;
  //opencv bridge
  //image_transport::ImageTransport it_;
  sensor_msgs::CvBridge bridge_;
  //sensor_msgs::Image roi_;
  OutputType subimage_;
  std::string input_image_topic_, input_cloud_topic_, output_cluster_topic_, output_image_topic_;
  //calibration parameters for svistec cameras
  double focal_length_, proj_center_x_, proj_center_y_, pix_size_x_, pix_size_y_;
  //ROS msgs
  //sensor_msgs::PointCloud cloud_in_;
  //IplImage* image_;
  std::string origin_, interim_, child_;
  boost::mutex  cloud_lock_;
  std::vector <sensor_msgs::PointCloud> cloud_queue_;
};

}//namespace image_algos
#endif


