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
 * $Id: table_objects.cpp 30899 2010-07-16 04:56:51Z rusu $
 *
 */

/**
\author Zoltan-Csaba Marton
 **/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <mapping_msgs/CollisionObject.h>

#include <pcl/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/features/feature.h>
#include <pcl/segmentation/extract_clusters.h>

#include <table_objects/GetObjects.h>

// Eigen
#include <Eigen3/Core>
#include <Eigen3/Geometry>

using namespace pcl;

bool
  getTableObjects (table_objects::GetObjects::Request &req, table_objects::GetObjects::Response &res)
{
  // ---[ Get the input
  PointCloud<PointXYZ> cloud;
  pcl::fromROSMsg (req.data, cloud);

  // ---[ Split the objects into Euclidean clusters
  std::vector<pcl::PointIndices> clusters;
  pcl::EuclideanClusterExtraction<PointXYZ> cluster;
  cluster.setInputCloud (cloud.makeShared ());
  cluster.setClusterTolerance (0.03);
  cluster.setMinClusterSize (20);
  cluster.extract (clusters);
  ROS_INFO ("[getTableObjects] Number of clusters found matching the given constraints: %d.", (int)clusters.size ());

  // ---[ Convert clusters to collision objects
  for (size_t i = 0; i < clusters.size (); ++i)
  {
    pcl::PointCloud<PointXYZ> cloud_object_cluster;
    pcl::copyPointCloud (cloud, clusters[i], cloud_object_cluster);
    Eigen3::Vector4f centroid;
    pcl::compute3DCentroid (cloud_object_cluster, centroid);
    
    // project to 2D
    pcl::PointCloud<PointXYZ> cloud_object_cluster_2D = cloud_object_cluster;
    for (size_t cp = 0; cp < cloud_object_cluster_2D.points.size (); cp ++)
      cloud_object_cluster_2D.points[cp].z /= 1000; // can't set it to 0 directly because of a bug in eigen33
      
    // get the rough oriented bounding box
    Eigen3::Vector4f centroid2D;
    pcl::compute3DCentroid (cloud_object_cluster_2D, centroid2D);
    Eigen3::Matrix3f covariance_matrix;
    computeCovarianceMatrix (cloud_object_cluster_2D, centroid2D, covariance_matrix);
    Eigen3::Matrix3f evecs;
    Eigen3::Vector3f evals;
    pcl::eigen33 (covariance_matrix, evecs, evals);
    Eigen3::Vector3f up = evecs.col (0);
    evecs.col (0) = evecs.col (1);
    evecs.col (1) = evecs.col (2);
    evecs.col (2) = up;
    cerr << "Eigenvalues followed by re-ordered eigenvectors: " << evals.transpose () << endl;
    cerr << evecs << endl;
    //cerr << "norms: " << evecs.col (0).norm () << " " << evecs.col (1).norm () << " " << evecs.col (2).norm () << endl;
    Eigen3::Matrix3f rotation = evecs.transpose ();
    Eigen3::Quaternion<float> qt (evecs);
    //cerr << "Quaternions: " << qt.x () << " " << qt.y () << " " << qt.z () << " " << qt.w () << endl;
    
    //pcl::copyPointCloud (cloud, clusters[i], cloud_object_cluster);
    Eigen3::Array3f min_point (+FLT_MAX, +FLT_MAX, +FLT_MAX);
    Eigen3::Array3f max_point (-FLT_MAX, -FLT_MAX, -FLT_MAX);
    for (size_t cp = 0; cp < cloud_object_cluster.points.size (); cp ++)
    {
      Eigen3::Map<Eigen3::Vector3f> point (&cloud_object_cluster.points[cp].x);
      Eigen3::Array3f transformed = rotation * (point - centroid.head<3> ());
      //cerr << point[2] << "/" << (point - centroid2D.head<3> ())[2] << "/" << transformed[0] << " ";
      min_point = min_point.min (transformed);
      max_point = max_point.max (transformed);
    }
    cerr << "Minimum corner: " << min_point.transpose () << endl;
    cerr << "Maximum corner: " << max_point.transpose () << endl;
    
    // create the collison object
    std::stringstream ss;
    ss << "cluster_" << i;
    mapping_msgs::CollisionObject collision_object;
    collision_object.header = cloud.header;
    collision_object.id = ss.str ();
    collision_object.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
    collision_object.shapes.resize (1);
    collision_object.shapes[0].type = geometric_shapes_msgs::Shape::BOX;
    collision_object.shapes[0].dimensions.resize (3);
    collision_object.shapes[0].dimensions[0] = std::max (0.01f, max_point[0] - min_point[0]);
    collision_object.shapes[0].dimensions[1] = std::max (0.01f, max_point[1] - min_point[1]);
    collision_object.shapes[0].dimensions[2] = std::max (0.01f, max_point[2] - min_point[2]);
    collision_object.poses.resize (1);
    collision_object.poses[0].position.x = centroid[0];
    collision_object.poses[0].position.y = centroid[1];
    collision_object.poses[0].position.z = centroid[2];
    collision_object.poses[0].orientation.x = qt.x ();
    collision_object.poses[0].orientation.y = qt.y ();
    collision_object.poses[0].orientation.z = qt.z ();
    collision_object.poses[0].orientation.w = qt.w ();
    res.table_objects.push_back (collision_object);
  }

  return (true);
}

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "table_objects");

  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService ("get_table_objects", getTableObjects);
  ros::spin ();

  return (0);
}
