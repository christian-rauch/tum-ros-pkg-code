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
 * $Id: table_pose.cpp 30899 2010-07-16 04:56:51Z rusu $
 *
 */

/**
\author Radu Bogdan Rusu
 **/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_ann.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>

#include <table_pose/GetPose.h>

using namespace pcl;
      
ros::Publisher pub;

bool
  getTablePose (table_pose::GetPose::Request &req, table_pose::GetPose::Response &res)
{
  PointCloud<PointXYZ> cloud;
  pcl::fromROSMsg (req.data, cloud);

  // Use an ANN kd tree representation
  KdTreeANN<PointXYZ>::Ptr tree = boost::make_shared<KdTreeANN<PointXYZ> > ();

  // ---[ Create the voxel grid
  VoxelGrid<PointXYZ> grid;
  grid.setInputCloud (cloud.makeShared ());
  grid.setLeafSize (0.01, 0.01, 0.01);
  grid.setFilterFieldName ("z");
  grid.setFilterLimits (0.5, 1.2);

  PointCloud<PointXYZ> cloud_filtered;
  grid.filter (cloud_filtered);

  // ---[ Estimate the point normals
  NormalEstimation<PointXYZ, Normal> n3d;
  n3d.setInputCloud (cloud_filtered.makeShared ());
  n3d.setSearchMethod (tree);
  n3d.setKSearch (0.01);
  
  PointCloud<Normal> cloud_normals;
  n3d.compute (cloud_normals);

  // ---[ Perform segmentation
  SACSegmentationFromNormals<PointXYZ, Normal> seg;
  seg.setInputCloud (cloud_filtered.makeShared ());
  seg.setInputNormals (cloud_normals.makeShared ());
  seg.setNormalDistanceWeight (0.1);
  seg.setModelType (SACMODEL_NORMAL_PLANE);
  seg.setMethodType (SAC_RANSAC);
  seg.setDistanceThreshold (0.1);
  seg.setMaxIterations (10000);

  PointIndices table_inliers; 
  ModelCoefficients table_coefficients;
  seg.segment (table_inliers, table_coefficients);

  // ---[ Extract the table
  PointCloud<PointXYZ> table_projected;
  ProjectInliers<PointXYZ> proj;
  proj.setModelType (SACMODEL_NORMAL_PLANE);
  proj.setInputCloud (cloud_filtered.makeShared ());
  proj.setIndices (boost::make_shared<PointIndices> (table_inliers));
  proj.setModelCoefficients (boost::make_shared<ModelCoefficients> (table_coefficients));
  proj.filter (table_projected);

  // Publish (not needed) - to be removed
  PointCloud<PointXYZ> cloud_table;
  copyPointCloud (cloud_filtered, table_inliers, cloud_table);
  sensor_msgs::PointCloud2 cloud_out;
  pcl::toROSMsg (cloud_table, cloud_out);
  pub.publish (cloud_out);

  return (true);
}

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "table_pose");

  ros::NodeHandle nh;
  
  pub = nh.advertise<sensor_msgs::PointCloud2> ("table", 1);

  ros::ServiceServer service = nh.advertiseService ("get_table_pose", getTablePose);
  ros::spin ();

  return (0);
}
/* ]--- */

