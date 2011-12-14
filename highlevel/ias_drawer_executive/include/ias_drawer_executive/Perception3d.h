

/*
 * Copyright (c) 2011, Thomas Ruehr <ruehr@cs.tum.edu>
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


#ifndef __Perception3d_H__
#define __Perception3d_H__

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "pcl/common/common.h"

#include <boost/thread.hpp>
#include <tf/transform_listener.h>

#define ROSCPP_DEPRECATED

#include <ias_table_msgs/TableCluster.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

class Perception3d{

    public:

  static boost::mutex handle_cloud_mutex;
  static boost::mutex handle_mutex;
  static boost::mutex plane_mutex;
  static PointCloudT lastCloud;
  static btVector3 handleHint;
  static btVector3 handleResult;
  static ros::Time cloud_time;
  static ros::Time query_time;
  static double handleMinDist;
  static std::vector<tf::Stamped<tf::Pose> *> handlePoses;
  static std::vector<tf::Stamped<tf::Pose> *> planePoses;

  //get handles by pose
  static void handleCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  static tf::Stamped<tf::Pose> getHandlePoseFromLaser(int pos);

  //get inslied cloud of handles
  static void handleCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

  static tf::Stamped<tf::Pose> getHandlePoseFromLaser(tf::Stamped<tf::Pose> hint, double wait = -1);

  //get plane center inside fridge
  static void fridgePlaneCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

  static tf::Stamped<tf::Pose> getFridgePlaneCenterPose();

  // get bottle
  static void bottleCallback(const ias_table_msgs::TableCluster::ConstPtr& msg);

  static tf::Stamped<tf::Pose> getBottlePose();

};



#endif
