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
    double table_min_height_, table_max_height_, table_clusters_growing_tolerance_, table_region_angle_threshold_;
    int table_clusters_min_pts_;

    double object_delta_z_, object_min_dist_from_table_;

    Subscriber cloud_sub_;
    int downsample_factor_;
    ServiceServer clusters_service_;
    Publisher cloud_table_pub_, cloud_clusters_pub_, pmap_pub_;

    // Spend CPU cycles to publish the results for visualization purposes ?
    bool publish_debug_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ClusterVoxelization () : publish_debug_ (true)
    {
      axis_.x = 0; axis_.y = 0; axis_.z = 1;
      {
        nh_.param ("~downsample_leaf_width_x", leaf_width_.x, 0.03);          // 3cm radius by default
        nh_.param ("~downsample_leaf_width_y", leaf_width_.y, 0.03);          // 3cm radius by default
        nh_.param ("~downsample_leaf_width_z", leaf_width_.z, 0.03);          // 3cm radius by default
        nh_.param ("~search_k_closest", k_, 10);                 // 10 k-neighbors by default
      }

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

      {
        nh_.param ("~sac_distance_threshold", sac_distance_threshold_, 0.03);     // 3 cm
      }

      {
        nh_.param ("~object_table_delta_z", obect_delta_z_, 0.03);                        // consider objects starting at 3cm from the table
        nh_.param ("~object_min_distance_from_table", object_min_dist_from_table_, 0.08); // objects which have their support more 8cm from the table will not be considered
      }

      nh_.param ("~object_cluster_tolerance", object_cluster_tolerance_, 0.04);   // 4cm between two objects
      nh_.param ("~object_cluster_min_pts", object_cluster_min_pts_, 30);         // 30 points per object cluster

      {
        cloud_table_pub_    = nh_.advertise<PointCloud> ("table_cloud", 1);
        cloud_clusters_pub_ = nh_.advertise<PointCloud> ("clusters_cloud", 1);
        pmap_pub_           = nh_.advertise<PolygonalMap> ("table_polygon", 1);
      }

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

      // First detect the table plane
      vector<int> table_inliers;
      vector<double> table_plane_coeff;
      detectTable (*cloud_in_, resp, table_inliers, table_plane_coeff);

      // Then get the object clusters supported by the table
      Point32 min_p, max_p;
      cloud_geometry::statistics::getMinMax (*cloud_in_, table_inliers, min_p, max_p);
      vector<int> object_inliers;
      extractObjectClusters (*cloud_in_, table_plane_coeff, axis_, min_p, max_p, object_inliers);

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
    bool
      detectTable (const PointCloud &cloud, ClustersVoxels::Response &resp, vector<int> &table_inliers, vector<double> &table_plane_coeff)
    {
      ros::Time ts = ros::Time::now ();

      // We have a pointcloud, get the maximum table bounds - assume Z is point up!
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
        return (false);
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
      cloud_geometry::nearest::extractEuclideanClusters (cloud_down_, indices_z,
          table_clusters_growing_tolerance_, clusters, 0, 1, 2, table_region_angle_threshold_, table_clusters_min_pts_);

      sort (clusters.begin (), clusters.end (), compareRegions);
      ROS_INFO ("Number of clusters found: %d, largest cluster: %d.", (int)clusters.size (), (int)clusters[clusters.size () - 1].size ());

      int c_good = -1;
      double eps_angle_deg = angles::to_degrees (eps_angle_);
      for (int i = clusters.size () - 1; i >= 0; i--)
      {
        // Find the best plane in this cluster
        fitSACPlane (&cloud_down_, clusters[i], table_inliers, table_plane_coeff, viewpoint, sac_distance_threshold_);
        // One extra check to see if the plane is indeed perpendicular to Z
        double angle = angles::to_degrees (cloud_geometry::angles::getAngleBetweenPlanes (table_plane_coeff, axis_));
        if ( fabs (angle) < eps_angle_deg || fabs (180.0 - angle) < eps_angle_deg )
        {
          c_good = i;
          break;
        }
      }

      if (c_good == -1)
      {
        ROS_WARN ("No supporting plane / table found under the given constraints! Exiting...");
        return (false);
      }

      // Obtain the bounding 2D polygon of the table
      Polygon3D table;
      cloud_geometry::areas::convexHull2D (cloud_down_, table_inliers, table_plane_coeff, table);

      if (publish_debug_)
      {
        // Create a 2D polygon
        PolygonalMap pmap;
        pmap.header = cloud.header;
        pmap.polygons.resize (1);
        pmap.polygons[0] = table;
        pmap_pub_.publish (pmap);

        // Create a point cloud
        PointCloud cloud_annotated;
        cloud_geometry::getPointCloud (cloud_down_, inliers, cloud_annotated);   // downsampled version
        cloud_table_pub_.publish (cloud_annotated);
      }
      return (true);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      extractObjectClusters (const PointCloud &cloud, const vector<double> &coeff, const Polygon3D &table,
                             const Point32 &axis, const Point32 &min_p, const Point32 &max_p, vector<int> &object_indices)
    {
      int nr_p = 0;
      Point32 pt;
      object_indices.resize (cloud.pts.size ());

      // Iterate over the entire cloud to extract the object clusters
      for (unsigned int i = 0; i < cloud.pts.size (); i++)
      {
        // Select all the points in the given bounds - check all axes
        if ( axis.x == 1 && ( cloud.pts.at (i).y < min_p.y || cloud.pts.at (i).y > max_p.y || cloud.pts.at (i).z < min_p.z || cloud.pts.at (i).z > max_p.z ) )
          continue;

        else if ( axis.y == 1 && ( cloud.pts.at (i).x < min_p.x || cloud.pts.at (i).x > max_p.x || cloud.pts.at (i).z < min_p.z || cloud.pts.at (i).z > max_p.z ) )
          continue;

        else if ( axis.z == 1 && ( cloud.pts.at (i).x < min_p.x || cloud.pts.at (i).x > max_p.x || cloud.pts.at (i).y < min_p.y || cloud.pts.at (i).y > max_p.y ) )
          continue;

        // Calculate the distance from the point to the plane
        double dist_to_plane = coeff.at (0) * cloud.pts.at (i).x +
                               coeff.at (1) * cloud.pts.at (i).y +
                               coeff.at (2) * cloud.pts.at (i).z +
                               coeff.at (3) * 1;
        // Calculate the projection of the point on the plane
        pt.x = cloud.pts.at (i).x - dist_to_plane * coeff.at (0);
        pt.y = cloud.pts.at (i).y - dist_to_plane * coeff.at (1);
        pt.z = cloud.pts.at (i).z - dist_to_plane * coeff.at (2);

        if (dist_to_plane > object_delta_z_ && cloud_geometry::areas::isPointIn2DPolygon (pt, table))
        {
          object_indices[nr_p] = i;
          nr_p++;
        }
      }
      object_indices.resize (nr_p);

      // Find the clusters
      nr_p = 0;
      vector<vector<int> > object_clusters;
      cloud_geometry::nearest::extractEuclideanClusters (cloud, object_indices, object_cluster_tolerance_,
                                                         object_clusters, -1, -1, -1, -1, object_cluster_min_pts_);

#ifdef DEBUG
        int total_nr_pts = 0;
        for (unsigned int i = 0; i < object_clusters.size (); i++)
          total_nr_pts += object_clusters[i].size ();

        cloud_annotated_.header = cloud.header;
        cloud_annotated_.pts.resize (total_nr_pts);
        cloud_annotated_.chan.resize (1);
        cloud_annotated_.chan[0].name = "rgb";
        cloud_annotated_.chan[0].vals.resize (total_nr_pts);
        ROS_INFO ("Number of clusters found: %d", (int)object_clusters.size ());
#endif

      robot_msgs::Point32 min_p_cluster, max_p_cluster;
      
//      resp.oclusters.resize (object_clusters.size ());
      for (unsigned int i = 0; i < object_clusters.size (); i++)
      {
#ifdef DEBUG
        float rgb = getRGB (rand () / (RAND_MAX + 1.0), rand () / (RAND_MAX + 1.0), rand () / (RAND_MAX + 1.0));
#endif
        vector<int> object_idx = object_clusters.at (i);

        // Check whether this object cluster is supported by the table or just flying through thin air
        cloud_geometry::statistics::getMinMax (cloud, object_idx, min_p_cluster, max_p_cluster);
        // Select all the points in the given bounds - check all axes
        if ( axis.x == 1 && ( min_p_cluster.x > max_p.x + object_min_dist_from_table_ ) )
          continue;        
        if ( axis.y == 1 && ( min_p_cluster.y > max_p.y + object_min_dist_from_table_ ) )
          continue;        
        if ( axis.z == 1 && ( min_p_cluster.z > max_p.z + object_min_dist_from_table_ ) )
          continue;        

        // Process this cluster and extract the centroid and the bounds
        for (unsigned int j = 0; j < object_idx.size (); j++)
        {
          object_indices[nr_p] = object_idx.at (j);
#ifdef DEBUG          
            cloud_annotated_.pts[nr_p] = cloud.pts.at (object_idx.at (j));
            cloud_annotated_.chan[0].vals[nr_p] = rgb;
#endif
          nr_p++;
        }
//        cloud_geometry::statistics::getMinMax (cloud, object_idx, resp.oclusters[i].min_bound, resp.oclusters[i].max_bound);
//        cloud_geometry::nearest::computeCentroid (cloud, object_idx, resp.oclusters[i].center);
      }
      object_indices.resize (nr_p);
#ifdef DEBUG
        cloud_annotated_.pts.resize (nr_p);
        cloud_annotated_.chan[0].vals.resize (nr_p);
#endif
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool
      fitSACPlane (PointCloud *points, vector<int> &indices, vector<int> &inliers, vector<double> &coeff,
                   const PointStamped &viewpoint_cloud, double dist_thresh)
    {
      if ((int)indices.size () < clusters_min_pts_)
      {
        inliers.resize (0);
        coeff.resize (0);
        return (false);
      }

      // Create and initialize the SAC model
      sample_consensus::SACModelPlane *model = new sample_consensus::SACModelPlane ();
      sample_consensus::SAC *sac             = new sample_consensus::MSAC (model, dist_thresh);
      sac->setMaxIterations (200);
      sac->setProbability (0.99);
      model->setDataSet (points, indices);

      // Search for the best plane
      if (sac->computeModel ())
      {
        if ((int)sac->getInliers ().size () < clusters_min_pts_)
        {
          ROS_WARN ("fitSACPlane: Inliers.size (%d) < sac_min_points_per_model (%d)!", sac->getInliers ().size (), clusters_min_pts_);
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
