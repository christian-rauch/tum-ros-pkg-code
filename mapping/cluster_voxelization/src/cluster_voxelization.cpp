/*
 * Copyright (c) 2009 Technische Universitaet Muenchen
 *
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
 *
 * $Id: cluster_voxelization.cpp 17089 2009-06-15 18:52:12Z veedee $
 *
 */

/**
@mainpage

@htmlinclude manifest.html

 **/

// ROS core
#include <ros/node.h>
#include <ros/node_handle.h>
// ROS messages
#include <robot_msgs/PointCloud.h>
#include <robot_msgs/PolygonalMap.h>

// Sample Consensus
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/msac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>

#include <angles/angles.h>

// Kd Tree
#include <point_cloud_mapping/kdtree/kdtree_ann.h>

// Cloud geometry
#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/geometry/areas.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/distances.h>
#include <point_cloud_mapping/geometry/nearest.h>
#include <point_cloud_mapping/geometry/transforms.h>
#include <point_cloud_mapping/geometry/statistics.h>

#include <sys/time.h>

#include <perception_srvs/ClustersVoxels.h>

using namespace std;
using namespace ros;
using namespace std_msgs;
using namespace robot_msgs;
using namespace perception_srvs;

// Comparison operator for a vector of vectors
bool
  compareRegions (const std::vector<int> &a, const std::vector<int> &b)
{
  return (a.size () < b.size ());
}

class ClusterVoxelization
{
  protected:
    ros::NodeHandle nh_;
  public:

    // ROS messages
    PointCloudConstPtr cloud_in_;

    Point32 axis_;
    PointCloud cloud_down_;
    bool need_cloud_data_;

    // Parameters
    Point leaf_width_;
    string input_cloud_topic_;
    int k_;
    int clusters_min_pts_;

    int object_cluster_min_pts_;
    double object_cluster_tolerance_;
    double sac_distance_threshold_, eps_angle_;

    // Min/max bounds for tables
    double table_min_height_, table_max_height_;
    double table_clusters_growing_tolerance_, table_region_angle_threshold_;
    int table_clusters_min_pts_;

    Subscriber cloud_sub_;
    int downsample_factor_;
    ServiceServer clusters_service_;
//     Publisher cloud_table_pub_, cloud_clusters_pub_, pmap_pub_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ClusterVoxelization ()
    {
      axis_.x = 0; axis_.y = 0; axis_.z = 1;
      nh_.param ("~downsample_leaf_width_x", leaf_width_.x, 0.03);          // 3cm radius by default
      nh_.param ("~downsample_leaf_width_y", leaf_width_.y, 0.03);          // 3cm radius by default
      nh_.param ("~downsample_leaf_width_z", leaf_width_.z, 0.03);          // 3cm radius by default
      nh_.param ("~search_k_closest", k_, 10);                 // 10 k-neighbors by default
      nh_.param ("~normal_eps_angle", eps_angle_, 15.0);       // 15 degrees
      eps_angle_ = angles::from_degrees (eps_angle_);          // convert to radians

      {
        nh_.param ("~table_min_height", table_min_height_, 0.5);              // minimum height of a table : 0.5m
        nh_.param ("~table_max_height", table_max_height_, 1.5);              // maximum height of a table : 1.5m
        nh_.param ("~table_clusters_growing_tolerance", table_clusters_growing_tolerance_, 0.5);   // 0.5 m
        nh_.param ("~table_clusters_min_pts", table_clusters_min_pts_, 50);                       // 50 points
        nh_.param ("~table_region_angle_threshold", table_region_angle_threshold_, 30.0);         // Difference between normals in degrees for cluster/region growing
        table_region_angle_threshold_ = angles::from_degrees (table_region_angle_threshold_);     // convert to radians
      }

      nh_.param ("~object_cluster_tolerance", object_cluster_tolerance_, 0.04);   // 4cm between two objects
      nh_.param ("~object_cluster_min_pts", object_cluster_min_pts_, 30);         // 30 points per object cluster

      nh_.param ("~input_cloud_topic", input_cloud_topic_, string ("/cloud_pcd"));
      clusters_service_ = nh_.advertiseService ("clusters_service", &ClusterVoxelization::cluster_voxelization_service, this);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   void
      updateParametersFromServer ()
    {
      nh_.getParam ("~input_cloud_topic", input_cloud_topic_);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool
      cluster_voxelization_service (ClustersVoxels::Request &req, ClustersVoxels::Response &resp)
    {
      ROS_INFO ("Service request initiated.");
      updateParametersFromServer ();

      // Subscribe to a point cloud topic
      need_cloud_data_ = true;
      cloud_sub_ = nh_.subscribe (input_cloud_topic_, 1, &ClusterVoxelization::cloud_cb, this);

      // Wait until the scan is ready, sleep for 100ms
      ros::Duration tictoc (0, 10000000);
      while (need_cloud_data_)
      {
        //tictoc.sleep ();
        ros::spinOnce ();
      }

      detectTable (*cloud_in_, resp);

      ROS_INFO ("Service request terminated.");
      return (true);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PointCloud message callback
    void
      cloud_cb (const PointCloudConstPtr& cloud)
    {
      if (!need_cloud_data_)
        return;

      ROS_INFO ("PointCloud message received on %s with %d points.", input_cloud_topic_.c_str (), (int)cloud->pts.size ());
      cloud_in_ = cloud;
      need_cloud_data_ = false;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      detectTable (const PointCloud &cloud, ClustersVoxels::Response &resp)
    {
      ros::Time ts = ros::Time::now ();

      // We have a pointcloud, estimate the true point bounds - assume Z is point up!
      vector<int> indices_in_bounds (cloud.pts.size ());
      int nr_p = 0;
      for (unsigned int i = 0; i < cloud.pts.size (); i++)
      {
        if (cloud.pts[i].z >= table_min_height_ && cloud.pts[i].z <= table_max_height_)
        {
          indices_in_bounds[nr_p] = i;
          nr_p++;
        }
      }
      indices_in_bounds.resize (nr_p);
      ROS_INFO ("%d of %d points are within the table height bounds of [%.2lf,%.2lf]", nr_p, (int)cloud.pts.size (), table_min_height_, table_max_height_);

      // Downsample the cloud in the bounding box for faster processing
      // NOTE: <leaves_> gets allocated internally in downsamplePointCloud() and is not deallocated on exit
      vector<cloud_geometry::Leaf> leaves;
      try
      {
        cloud_geometry::downsamplePointCloud (cloud, indices_in_bounds, cloud_down_, leaf_width_, leaves, -1);
      }
      catch (std::bad_alloc)
      {
        // downsamplePointCloud should issue a ROS_ERROR on screen, so we simply exit here
        return;
      }
      ROS_INFO ("Number of points after downsampling with a leaf of size [%f,%f,%f]: %d.", leaf_width_.x, leaf_width_.y, leaf_width_.z, (int)cloud_down_.pts.size ());

      // Compute point cloud normals
      PointStamped viewpoint;
      int vx_idx = cloud_geometry::getChannelIndex (cloud, "vx");
      if (vx_idx != -1)       // If viewpoints are present in the dataset
      {
        viewpoint.point.x = cloud.chan[vx_idx + 0].vals[0];
        viewpoint.point.y = cloud.chan[vx_idx + 1].vals[0];
        viewpoint.point.z = cloud.chan[vx_idx + 2].vals[0];
      }
      else
        viewpoint.point.x = viewpoint.point.y = viewpoint.point.z = 0.0;
      cloud_geometry::nearest::computePointCloudNormals (cloud_down_, cloud, k_, viewpoint);

      // ---[ Select points whose normals are perpendicular to the Z-axis
      vector<int> indices_z;
      cloud_geometry::getPointIndicesAxisParallelNormals (cloud_down_, 0, 1, 2, eps_angle_, axis_, indices_z);
      ROS_INFO ("Number of points with normals parallel to Z: %d.", (int)indices_z.size ());

      vector<vector<int> > clusters;
      // Split the Z-parallel points into clusters
      int nx_idx = cloud_geometry::getChannelIndex (cloud, "nx");
      cloud_geometry::nearest::extractEuclideanClusters (cloud_down_, indices_z,
          table_clusters_growing_tolerance_, clusters, nx_idx + 0, nx_idx + 1, nx_idx + 2,
          table_region_angle_threshold_, table_clusters_min_pts_);

      sort (clusters.begin (), clusters.end (), compareRegions);

      return;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool
      fitSACPlane (PointCloud *points, vector<int> &indices, vector<int> &inliers, vector<double> &coeff,
                   const Point32 &viewpoint_cloud, double dist_thresh)
    {
      if ((int)indices.size () < clusters_min_pts_)
      {
        inliers.resize (0);
        coeff.resize (0);
        return (false);
      }

      // Create and initialize the SAC model
      sample_consensus::SACModelPlane *model = new sample_consensus::SACModelPlane ();
      sample_consensus::SAC *sac             = new sample_consensus::MSAC (model, sac_distance_threshold_);
      sac->setMaxIterations (200);
      sac->setProbability (0.99);
      model->setDataSet (points, indices);

      // Search for the best plane
      if (sac->computeModel ())
      {
        if ((int)sac->getInliers ().size () < clusters_min_pts_)
        {
          //ROS_ERROR ("fitSACPlane: Inliers.size (%d) < sac_min_points_per_model (%d)!", sac->getInliers ().size (), sac_min_points_per_model_);
          inliers.resize (0);
          coeff.resize (0);
          return (false);
        }

        sac->computeCoefficients (coeff);     // Compute the model coefficients
        sac->refineCoefficients (coeff);      // Refine them using least-squares
        model->selectWithinDistance (coeff, dist_thresh, inliers);

        cloud_geometry::angles::flipNormalTowardsViewpoint (coeff, points->pts.at (inliers[0]), viewpoint_cloud);

        // Project the inliers onto the model
        model->projectPointsInPlace (inliers, coeff);
      }
      return (true);
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "cluster_voxelization");

  ClusterVoxelization p;
  ros::spin ();

  return (0);
}
/* ]--- */
