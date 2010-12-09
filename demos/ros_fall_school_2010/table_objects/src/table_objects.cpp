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
#include <table_objects/GetLastObjects.h>

#include <table_objects/table_objects.h>

// Eigen
#include <Eigen3/Core>
#include <Eigen3/Geometry>

using namespace pcl;

bool classify;
flann::Index< flann::L2<float> > *flann_index;

/*
#define MAX_QUEUE 20
std::vector<mapping_msgs::CollisionObject> last_table_objects;

bool
  getLastObjects (table_objects::GetLastObjects::Request &req, table_objects::GetLastObjects::Response &res)
{
  std::vector<mapping_msgs::CollisionObject>::iterator oldest = last_table_objects.end ();
  if (req.number < last_table_objects.size ())
    oldest = last_table_objects.begin () + req.number;
  res.table_objects.insert (res.table_objects.end (), last_table_objects.begin (), oldest);
  return (true);
}*/

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
  res.table_objects.reserve (clusters.size ());
  for (size_t i = 0; i < clusters.size (); ++i)
  {
    pcl::PointCloud<PointXYZ> cloud_object_cluster;
    pcl::copyPointCloud (cloud, clusters[i], cloud_object_cluster);
    std::stringstream ss;
    ss << "cluster_" << i;
    std::string label = ss.str ();
    
    // run VFH classification
    if (classify)
    {
      // compute normals
      pcl::PointCloud<pcl::Normal>::Ptr normals = boost::make_shared<pcl::PointCloud<pcl::Normal> >();
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n3d;
      n3d.setKSearch (30); // have to use the same parameters as during training
      pcl::KdTree<pcl::PointXYZ>::Ptr tree = boost::make_shared<pcl::KdTreeANN<pcl::PointXYZ> > ();
      n3d.setSearchMethod (tree);
      n3d.setInputCloud(cloud_object_cluster.makeShared ());
      n3d.compute(*normals);
      
      // compute vfh
      pcl::PointCloud<pcl::VFHSignature308> vfh_signature;
      pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
      vfh.setSearchMethod (tree);
      vfh.setInputCloud(cloud_object_cluster.makeShared ());
      vfh.setInputNormals(normals);
      vfh.compute(vfh_signature);
      
      // search for nearest neighor
      int k = 1;
      flann::Matrix<float> p = flann::Matrix<float>(vfh_signature.points.at (0).histogram, 1, 308);
      flann::Matrix<int> k_indices = flann::Matrix<int>(new int[k], 1, k);
      flann::Matrix<float> k_distances = flann::Matrix<float>(new float[k], 1, k);
      flann_index->knnSearch (p, k_indices, k_distances, k, flann::SearchParams (512));

      // extracting the label name
      std::string vfh_model = models.at (k_indices[0][0]).first.c_str ();
      size_t pos1 = vfh_model.find_last_of ("/\\"); // just in case this will ever run on windows :P
      std::string vfh_label = vfh_model;
      if (pos1 != std::string::npos)
      {
        size_t pos2 = vfh_model.find ('.', pos1+1);
        vfh_label = vfh_model.substr (pos1+1, pos2-pos1-1);
      }
      while (vfh_label.find (".bag") != std::string::npos || vfh_label.find (".pcd") != std::string::npos)
        vfh_label = vfh_label.substr (0, vfh_label.size () - 4);
      
      // hard-code some nice label names for knowledge processing
      if (vfh_label.find ("cereal") != std::string::npos)
        label = "BreakfastCereal";
      else if (vfh_label.find ("milk") != std::string::npos)
        label = "CowsMilk-Product";
      else if (vfh_label.find ("lego") != std::string::npos)
        label = "Bowl-Eating"; // last minute change, as someone said bowls can not be grasped
      ROS_INFO ("VFH label for cluster %zu: %s", i, label.c_str ());
    }
    
    // compute 3D centroid
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
    //cerr << "Eigenvalues followed by eigenvectors: " << evals.transpose () << endl;
    //cerr << evecs << endl;
    Eigen3::Matrix3f rotation;
    rotation.row (2) = evecs.col (0);
    rotation.row (1) = evecs.col (1);
    rotation.row (0) = rotation.row (1).cross (rotation.row (2));
    //rotation.transposeInPlace ();
    cerr << "Rotation matrix: " << endl;
    cerr << rotation << endl;
    //cerr << "norms: " << rotation.row (0).norm () << " " << rotation.row (1).norm () << " " << rotation.row (2).norm () << endl;
    //cerr << "Quaternions: " << qt.x () << " " << qt.y () << " " << qt.z () << " " << qt.w () << endl;
    
    //pcl::copyPointCloud (cloud, clusters[i], cloud_object_cluster);
    Eigen3::Array3f min_point_projected (+FLT_MAX, +FLT_MAX, +FLT_MAX);
    Eigen3::Array3f max_point_projected (-FLT_MAX, -FLT_MAX, -FLT_MAX);
    Eigen3::Array3f min_point_base (+FLT_MAX, +FLT_MAX, +FLT_MAX);
    Eigen3::Array3f max_point_base (-FLT_MAX, -FLT_MAX, -FLT_MAX);
    for (size_t cp = 0; cp < cloud_object_cluster.points.size (); cp ++)
    {
      Eigen3::Map<Eigen3::Vector3f> point (&cloud_object_cluster.points[cp].x);
      Eigen3::Array3f transformed = rotation * (point - centroid.head<3> ());
      //cerr << point[2] << "/" << (point - centroid2D.head<3> ())[2] << "/" << transformed[0] << " ";
      
      min_point_base = min_point_base.min (point.array());
      max_point_base = max_point_base.max (point.array());

      min_point_projected = min_point_projected.min (transformed);
      max_point_projected = max_point_projected.max (transformed);
    }
    //cerr << "Minimum corner: " << min_point.transpose () << endl;
    //cerr << "Maximum corner: " << max_point.transpose () << endl;
    Eigen3::Array3f center_offset = min_point_base + (max_point_base - min_point_base)/2;

    
    rotation.transposeInPlace ();
    Eigen3::Quaternion<float> qt (rotation);
    qt.normalize ();

    // create the collison object
    mapping_msgs::CollisionObject collision_object;
    collision_object.header = cloud.header;
    collision_object.id = label;
    collision_object.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
    collision_object.shapes.resize (1);
    collision_object.shapes[0].type = geometric_shapes_msgs::Shape::BOX;
    collision_object.shapes[0].dimensions.resize (3);
    collision_object.shapes[0].dimensions[0] = std::max (0.01f, max_point_projected[0] - min_point_projected[0]);
    collision_object.shapes[0].dimensions[1] = std::max (0.01f, max_point_projected[1] - min_point_projected[1]);
    collision_object.shapes[0].dimensions[2] = std::max (0.01f, max_point_projected[2] - min_point_projected[2]);
    collision_object.poses.resize (1);
    collision_object.poses[0].position.x = center_offset[0];
    collision_object.poses[0].position.y = center_offset[1];
    collision_object.poses[0].position.z = center_offset[2];
    collision_object.poses[0].orientation.x = qt.x ();
    collision_object.poses[0].orientation.y = qt.y ();
    collision_object.poses[0].orientation.z = qt.z ();
    collision_object.poses[0].orientation.w = qt.w ();
    res.table_objects.push_back (collision_object);
    
    /*// save the last MAX_QUEUE objects
    last_table_objects.insert (last_table_objects.begin (), collision_object);
    if (last_table_objects.size () > MAX_QUEUE)
      last_table_objects.resize (MAX_QUEUE);*/
  }

  return (true);
}

/* ---[ */
int
  main (int argc, char** argv)
{
  // never the case, only to have the parameter descriptions in the code
  if (argc < 1)
  {
    print_error ("Syntax is: %s [options] {kdtree.idx} {training_data.h5} {training_data.list}\n", argv[0]);
    print_info  ("    where options are:  -metric = metric/distance type:  1 = Euclidean, 2 = Manhattan, 3 = Minkowski, 4 = Max, 5 = HIK, 6 = JM, 7 = Chi-Square (default: "); print_value ("%d", metric); print_info (")\n\n");
    //
    print_info  ("      * note: the metric_type has to match the metric that was used when the tree was created.\n");
    print_info  ("              the last three parameters are optional and represent: the kdtree index file (default: "); print_value ("kdtree.idx"); print_info (")\n"
                 "                                                                    the training data used to create the tree (default: "); print_value ("training_data.h5"); print_info (")\n"
                 "                                                                    the list of models used in the training data (default: "); print_value ("training_data.list"); print_info (")\n");
    return (-1);
  }

  // Check if classification can be done and repare everything for it
  classify = false;
  if (getParameters (argc, argv) == 1)
  {
    classify = true;
    // Load trainin data into FLANN
    loadFileList (models, training_data_list_file_name);
    flann::Matrix<float> data;
    flann::load_from_file (data, training_data_h5_file_name, "training_data");
    print_highlight ("Training data found. Loaded %d VFH models from %s/%s.\n", (int)data.rows, training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
    // Initialize FLAN indices
    flann_index = new flann::Index< flann::L2<float> > (data, flann::SavedIndexParams (kdtree_idx_file_name));
    flann_index->buildIndex ();
  }
  else
    ROS_WARN ("VFH classification is disabled!");
  
  //last_table_objects.reserve (MAX_QUEUE);
  
  ros::init (argc, argv, "table_objects");

  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService ("get_table_objects", getTableObjects);
  //ros::ServiceServer service2 = nh.advertiseService ("get_last_objects", getLastObjects);
  
  ros::spin ();

  return (0);
}
/* ]--- */
