/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: transform_pointcloud.cpp 30719 2010-07-09 20:28:41Z rusu $
 *
 */

/**

\author Dejan Pangercic

@b pointcloud_minmax_3d computes the MinMax3D point of the pointcloud
and publishes them along with the cloud's centroid.

**/

// ROS core
#include <ros/ros.h>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/common/common.h"

//for transformPointCloud
#include <pcl_tf/transforms.h>

#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <ias_table_msgs/TableCluster.h>

using namespace std;

class PointcloudMinMax3DNode
{
protected:
  ros::NodeHandle nh_;
  
public:
  string output_cluster_topic_, input_cloud_topic_;
  
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Publisher vis_pub_, max_pub_, min_pub_;

  ias_table_msgs::TableCluster output_cluster_;
  pcl::PointCloud<pcl::PointXYZ> cloud_in_;
  pcl::PointXYZ point_min_;
  pcl::PointXYZ point_max_;
  pcl::PointXYZ point_center_;
  visualization_msgs::Marker marker_center_;
  visualization_msgs::Marker marker_min_;
  visualization_msgs::Marker marker_max_;

  bool visualize_;
  ////////////////////////////////////////////////////////////////////////////////
  PointcloudMinMax3DNode  (ros::NodeHandle &n) : nh_(n)
  {
    // Maximum number of outgoing messages to be queued for delivery to subscribers = 1
    nh_.param("input_cloud_topic", input_cloud_topic_, std::string("cloud_pcd"));
    nh_.param("output_cluster_topic", output_cluster_topic_, std::string("cluster"));
    nh_.param("visualize", visualize_, false);
    sub_ = nh_.subscribe (input_cloud_topic_, 1,  &PointcloudMinMax3DNode::cloud_cb, this);
    ROS_INFO ("[PointcloudMinMax3DNode:] Listening for incoming data on topic %s", nh_.resolveName (input_cloud_topic_).c_str ());
    pub_ = nh_.advertise<ias_table_msgs::TableCluster>(output_cluster_topic_, 1);
    ROS_INFO ("[PointcloudMinMax3DNode:] Will be publishing data on topic %s.", nh_.resolveName (output_cluster_topic_).c_str ());
    vis_pub_ = nh_.advertise<visualization_msgs::Marker>( "center_marker", 0 );
    min_pub_ = nh_.advertise<visualization_msgs::Marker>( "min_marker", 0 );
    max_pub_ = nh_.advertise<visualization_msgs::Marker>( "max_marker", 0 );
  }

  ////////////////////////////////////////////////////////////////////////////////
  // cloud_cb (!)
  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& pc)
  {
    pcl::fromROSMsg(*pc, cloud_in_);
    getMinMax3D (cloud_in_, point_min_, point_max_);
    //Calculate the centroid of the hull
    output_cluster_.header.stamp = ros::Time::now();
    output_cluster_.header.frame_id = pc->header.frame_id;
    
    output_cluster_.center.x = (point_max_.x + point_min_.x)/2;
    output_cluster_.center.y = (point_max_.y + point_min_.y)/2;
    output_cluster_.center.z = (point_max_.z + point_min_.z)/2;
    
    output_cluster_.min_bound.x = point_min_.x;
    output_cluster_.min_bound.y = point_min_.y;
    output_cluster_.min_bound.z = point_min_.z;

    output_cluster_.max_bound.x = point_max_.x;
    output_cluster_.max_bound.y = point_max_.y;
    output_cluster_.max_bound.z = point_max_.z;
    

    ROS_INFO("[PointcloudMinMax3DNode:] Published cloud to topic %s", output_cluster_topic_.c_str());
    pub_.publish (output_cluster_);
    
    if (visualize_)
    {
      compute_marker(marker_center_, output_cluster_.center);
      vis_pub_.publish( marker_center_ );
      compute_marker(marker_min_, output_cluster_.min_bound);
      min_pub_.publish( marker_min_ );
      compute_marker(marker_max_, output_cluster_.max_bound);
      max_pub_.publish( marker_max_ );
    }
  }


  void compute_marker(visualization_msgs::Marker &marker,  geometry_msgs::Point32 &point)
  {
    marker.header.frame_id = output_cluster_.header.frame_id;
    marker.header.stamp =  output_cluster_.header.stamp;
    marker.ns = "object_cluster";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = point.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

  }
};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "pointcloud_minmax_3d");
  ros::NodeHandle n("~");
  PointcloudMinMax3DNode mm(n);
  ros::spin ();
  return (0);
}
