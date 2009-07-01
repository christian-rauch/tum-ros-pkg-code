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

#include <mapping_msgs/CollisionMap.h>
#include <perception_msgs/Voxel.h>
#include <perception_msgs/VoxelList.h>
#include <perception_srvs/ClustersVoxels.h>

using namespace std;
using namespace ros;
using namespace std_msgs;
using namespace robot_msgs;
using namespace mapping_msgs;
using namespace perception_srvs;
using namespace perception_msgs;

// Comparison operator for a vector of vectors
bool
  compareRegions (const std::vector<int> &a, const std::vector<int> &b)
{
  return (a.size () < b.size ());
}

// Comparison operator for a vector of vectors
bool
  compareVoxels (const Voxel &a, const Voxel &b)
{
  if (a.i < b.i)
    return (true);
  else if (a.i > b.i)
    return (false);
  else if (a.j < b.j)
    return (true);
  else if (a.j > b.j)
    return (false);
  else if (a.k < b.k)
    return (true);
  else
    return (false);
}

// Comparison operator for a vector of vectors
bool
  equalVoxels (const Voxel &a, const Voxel &b)
{
  return (a.i == b.i && a.j == b.j && a.k == b.k);
}

class ClusterVoxelization
{
  protected:
    ros::NodeHandle nh_;
  public:

    // ROS messages
    PointCloudConstPtr cloud_in_;
    CollisionMap c_map_;

    Point32 axis_;
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

    double max_table_voxels_z_;

    Subscriber cloud_sub_;
    int downsample_factor_;
    ServiceServer clusters_service_;
    Publisher cloud_table_pub_, cloud_clusters_pub_, pmap_pub_, cmap_pub_;

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
        nh_.param ("~object_table_delta_z", object_delta_z_, 0.02);                       // consider objects starting at 2cm from the table
        nh_.param ("~object_min_distance_from_table", object_min_dist_from_table_, 0.10); // objects which have their support more 10cm from the table will not be considered
      }

      {
        nh_.param ("~object_cluster_tolerance", object_cluster_tolerance_, 0.03);   // 3cm between two objects
        nh_.param ("~object_cluster_min_pts", object_cluster_min_pts_, 30);         // 30 points per object cluster
      }

      {
        nh_.param ("~max_table_voxels_z", max_table_voxels_z_, 0.3);                // .5 meters from the table: that's all we care about
      }

      if (publish_debug_)
      {
        cloud_table_pub_    = nh_.advertise<PointCloud> ("table_cloud", 1);
        cloud_clusters_pub_ = nh_.advertise<PointCloud> ("clusters_cloud", 1);
        pmap_pub_           = nh_.advertise<PolygonalMap> ("table_polygon", 1);
        cmap_pub_           = nh_.advertise<CollisionMap> ("collision_map", 1);
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

      // ---[ First detect the table plane
      vector<int> table_inliers;
      vector<double> table_plane_coeff;
      Polygon3D table_polygon;
      PointCloud cloud_out;
      ros::WallTime ts = ros::WallTime::now ();
      detectTable (*cloud_in_, cloud_out, table_inliers, table_plane_coeff, table_polygon);
      ROS_INFO ("Table detected in %g seconds.", (ros::WallTime::now () - ts).toSec ());

      // Get the minimum/maximum bounds of the table
      Point32 min_p, max_p;
      cloud_geometry::statistics::getMinMax (cloud_out, table_inliers, min_p, max_p);

      // ---[ Then get the object clusters supported by the table
      vector<vector<int> > object_clusters;
      ts = ros::WallTime::now ();
      extractObjectClusters (*cloud_in_, table_plane_coeff, table_polygon, axis_, min_p, max_p, object_clusters);
      ROS_INFO ("Object clusters extracted in %g seconds.", (ros::WallTime::now () - ts).toSec ());

      // Assemble the complete list of inliers for the table and the objects on top of it
      vector<int> table_object_inliers (cloud_in_->pts.size ());
      int j = 0;
      for (unsigned int i = 0; i < cloud_in_->pts.size (); i++)
      {
        // Refine table plane
        double dist_to_plane = cloud_geometry::distances::pointToPlaneDistance (cloud_in_->pts[i], table_plane_coeff);
        // Get all points from the original point cloud which could belong to the table plane, even if it's not axis-aligned
        // (note: if we do need axis aligned, check against their 2D projections in the bounding polygon).
        if (dist_to_plane < sac_distance_threshold_ &&
            cloud_in_->pts[i].x >= min_p.x && cloud_in_->pts[i].x <= max_p.x &&
            cloud_in_->pts[i].y >= min_p.y && cloud_in_->pts[i].y <= max_p.y)
          table_object_inliers[j++] = i;
      }
      for (unsigned int k = 0; k < object_clusters.size (); k++)
      {
        if (object_clusters[k].size () == 0)
          continue;
        for (unsigned int l = 0; l < object_clusters[k].size (); l++)
          table_object_inliers[j++] = object_clusters[k][l];
      }
      table_object_inliers.resize (j);

      // ---[ Then obtain the 3D bounds of the space around the table
      VoxelList voxels;
      ts = ros::WallTime::now ();
      computeOcclusionMap (*cloud_in_, table_object_inliers, min_p, max_p, req.leaf_width, voxels);
      ROS_INFO ("Occlusion map estimated in %g seconds.", (ros::WallTime::now () - ts).toSec ());

      ROS_INFO ("Service request terminated.");

      if (publish_debug_)
      {
        // Create a 2D polygon for the table
        PolygonalMap pmap;
        pmap.header = cloud_in_->header;
        pmap.polygons.resize (1);
        pmap.polygons[0] = table_polygon;
        pmap_pub_.publish (pmap);

        // Create a point cloud for the table
        PointCloud cloud_annotated;
        cloud_geometry::getPointCloud (cloud_out, table_inliers, cloud_annotated);   // downsampled version
        cloud_table_pub_.publish (cloud_annotated);

        // Count the number of points that we need to allocate
        int total_nr_pts = 0;
        for (unsigned int i = 0; i < object_clusters.size (); i++)
          total_nr_pts += object_clusters[i].size ();

        cloud_annotated.pts.resize (total_nr_pts);
        cloud_annotated.chan.resize (1);
        cloud_annotated.chan[0].name = "rgb";
        cloud_annotated.chan[0].vals.resize (total_nr_pts);

        total_nr_pts = 0;
        for (unsigned int i = 0; i < object_clusters.size (); i++)
        {
          if (object_clusters[i].size () == 0)
            continue;

          // Get a different color
          float rgb = getRGB (rand () / (RAND_MAX + 1.0), rand () / (RAND_MAX + 1.0), rand () / (RAND_MAX + 1.0));

          // Process this cluster and extract the centroid and the bounds
          for (unsigned int j = 0; j < object_clusters[i].size (); j++)
          {
            cloud_annotated.pts[total_nr_pts] = cloud_in_->pts.at (object_clusters[i].at (j));
            cloud_annotated.chan[0].vals[total_nr_pts] = rgb;
            total_nr_pts++;
          }
        }
//        cloud_geometry::statistics::getMinMax (cloud, object_idx, resp.oclusters[i].min_bound, resp.oclusters[i].max_bound);
//        cloud_geometry::nearest::computeCentroid (cloud, object_idx, resp.oclusters[i].center);
        cloud_clusters_pub_.publish (cloud_annotated);
      }

      // Prepare for service reply
      resp.vlist = voxels;
      resp.clusters.resize (object_clusters.size ());
      int nr_c = 0;
      for (unsigned int i = 0; i < object_clusters.size (); i++)
      {
        // If object cluster is empty, just continue
        if (object_clusters[i].size () == 0)
          continue;

        // Copy the data
        resp.clusters[nr_c].pts.resize (object_clusters[i].size ());
        resp.clusters[nr_c].chan.resize (cloud_in_->chan.size ());
        for (unsigned int d = 0; d < cloud_in_->chan.size (); d++)
        {
          resp.clusters[nr_c].chan[d].name = cloud_in_->chan[d].name;
          resp.clusters[nr_c].chan[d].vals.resize (object_clusters[i].size ());
        }

        // For every object cluster
        for (unsigned int j = 0; j < object_clusters[i].size (); j++)
        {
          resp.clusters[nr_c].pts[j] = cloud_in_->pts.at (object_clusters[i].at (j));

          for (unsigned int d = 0; d < cloud_in_->chan.size (); d++)
            resp.clusters[nr_c].chan[d].vals[j] = cloud_in_->chan[d].vals.at (object_clusters[i].at (j));
        }
        nr_c++;
      }

      return (true);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      computeOcclusionMap (const PointCloud& cloud, const vector<int> &indices, const Point32 &min_p, const Point32 &max_p,
                           const Point32& leaf_width, VoxelList &vlist)
    {
      Point32 viewpoint;
      int vx_idx = cloud_geometry::getChannelIndex (cloud, "vx");
      if (vx_idx == -1)       // If viewpoints are not present in the dataset
      {
        viewpoint.x = viewpoint.y = viewpoint.z = 0.0;
        ROS_WARN ("No viewpoint information present in the input data! Assuming acquisition viewpoint to be <0, 0, 0>!");
      }

      Point32 min_b, max_b, div_b;
      // Compute the minimum and maximum bounding box values
      min_b.x = min_p.x; max_b.x = max_p.x;
      min_b.y = min_p.y; max_b.y = max_p.y;

      min_b.z = min_p.z;
      // We want to go higher on Z
      max_b.z = max_table_voxels_z_ + max_p.z;

      // Compute the number of divisions needed along all axis
      div_b.x = (int)((max_b.x - min_b.x) / leaf_width.x);
      div_b.y = (int)((max_b.y - min_b.y) / leaf_width.y);
      div_b.z = (int)((max_b.z - min_b.z) / leaf_width.z);

      // Allocate the space needed
      try
      {
        vlist.voxels.reserve (div_b.x * div_b.y * div_b.z);             // fallback to x*y*z from 2*x*y*z due to memory problems
      }
      catch (std::bad_alloc)
      {
        ROS_ERROR ("Failed while attempting to allocate a vector of %f (%g x %g x %g) leaf elements (%f bytes total)", div_b.x * div_b.y * div_b.z,
                  div_b.x, div_b.y, div_b.z, div_b.x * div_b.y * div_b.z * sizeof (Voxel));
      }

      // Go over all points and insert them into the right leaf
      double curpoint[3], dir[3];
      int idx_cur[3], idx_goal[3];
      for (unsigned int cp = 0; cp < indices.size (); cp++)
      {

        // Create a line from this point to its viewpoint
        for (int d = 0; d < 3; d++)
          curpoint[d] = cloud.chan[vx_idx + d].vals[indices.at (cp)];
        dir[0] = cloud.pts[indices.at (cp)].x - cloud.chan[vx_idx + 0].vals[indices.at (cp)];
        dir[1] = cloud.pts[indices.at (cp)].y - cloud.chan[vx_idx + 1].vals[indices.at (cp)];
        dir[2] = cloud.pts[indices.at (cp)].z - cloud.chan[vx_idx + 2].vals[indices.at (cp)];

        // Get the index for the current point and the viewpoint
        getVoxelIndex (curpoint, leaf_width, min_b, idx_cur);
        getVoxelIndex (cloud.pts[indices.at (cp)], leaf_width, min_b, idx_goal);

        // Go over the line and select the appropiate i/j/k + idx
        bool seen_point = false;
        double c[3];
        while (seen_point == false || (curpoint[0] < max_b.x && curpoint[0] > min_b.x && curpoint[1] < max_b.y && curpoint[1] > min_b.y && curpoint[2] < max_b.z && curpoint[2] > min_b.z))
        {
          if (seen_point)
            break;
          else
          {
            if (idx_cur[0] == idx_goal[0] && idx_cur[1] == idx_goal[1] && idx_cur[2] == idx_goal[2] )
            {
              seen_point = true;
              break;
            }
            else
            {
//              int idx = ( (idx_cur[2] - min_b.z) * div_b.y * div_b.x ) + ( (idx_cur[1] - min_b.y) * div_b.x ) + (idx_cur[0] - min_b.x);
              if (idx_cur[0] > 0 && idx_cur[1] > 0 && idx_cur[2] > 0 && idx_cur[0] < div_b.x && idx_cur[1] < div_b.y && idx_cur[2] < div_b.z)
              {
                Voxel v;
                v.i = idx_cur[0]; v.j = idx_cur[1]; v.k = idx_cur[2];
                vlist.voxels.push_back (v);
              }
            }
          }

          c[0] = min_b.x + (idx_cur[0] + (dir[0] >= 0 ? 1 : 0)) * leaf_width.x - curpoint[0];
          c[1] = min_b.y + (idx_cur[1] + (dir[1] >= 0 ? 1 : 0)) * leaf_width.y - curpoint[1];
          c[2] = min_b.z + (idx_cur[2] + (dir[2] >= 0 ? 1 : 0)) * leaf_width.z - curpoint[2];

          double dir_n[3], r[3];
          double n_norm = sqrt (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
          for (int d = 0; d < 3; d++)
          {
            dir_n[d] = dir[d] / n_norm;
            r[d] = c[d] / dir_n[d];
          }

          double dist = 0.0;
          int go[3] = {0.0, 0.0, 0.0};
#define SGN(x) (((x)==0.0)?0.0:((x)>0.0?1.0:-1.0))
          for (int d = 0; d < 3; d++)
          {
            if (fabs (r[d]) <= fabs (r[(d + 1) % 3]))
              if (fabs (r[d]) <= fabs (r[(d + 2) % 3]))
              {
                go[d] = SGN (dir[d]);
                dist = r[d];
              }
          }
          if (dist == 0.0)
          {
            cerr << "crap!" << endl;
            break;
          }
          for (int d = 0; d < 3; d++)
          {
            curpoint[d] = curpoint[d] + dist * dir_n[d];
            idx_cur[d] += go[d];
          }
        }
      }

      ROS_INFO ("Total number of checked voxels: %d.", (int)vlist.voxels.size ());
      ros::WallTime ts = ros::WallTime::now ();
      sort (vlist.voxels.begin (), vlist.voxels.end (), compareVoxels);
      vlist.voxels.erase (unique (vlist.voxels.begin (), vlist.voxels.end (), equalVoxels), vlist.voxels.end ());
      ROS_INFO ("Remaining number of voxels (time spent: %f): %d.", (ros::WallTime::now () - ts).toSec (), (int)vlist.voxels.size ());

      // Fill in the remaining dataset
      vlist.min = min_b;
      vlist.leaf_width = leaf_width;
      vlist.ndivs = div_b;
      if (publish_debug_)
      {
        // Assemble the collision map from the list of voxels
        c_map_.header = cloud_in_->header;
        c_map_.boxes.resize (vlist.voxels.size ());
        for (unsigned int cl = 0; cl < vlist.voxels.size (); cl++)
        {
          c_map_.boxes[cl].extents.x = leaf_width.x / 2.0;
          c_map_.boxes[cl].extents.y = leaf_width.y / 2.0;
          c_map_.boxes[cl].extents.z = leaf_width.z / 2.0;
          c_map_.boxes[cl].center.x = (vlist.voxels[cl].i + 1) * leaf_width.x - c_map_.boxes[cl].extents.x + min_b.x;
          c_map_.boxes[cl].center.y = (vlist.voxels[cl].j + 1) * leaf_width.y - c_map_.boxes[cl].extents.y + min_b.y;
          c_map_.boxes[cl].center.z = (vlist.voxels[cl].k + 1) * leaf_width.z - c_map_.boxes[cl].extents.z + min_b.z;
          c_map_.boxes[cl].axis.x = c_map_.boxes[cl].axis.y = c_map_.boxes[cl].axis.z = 0.0;
          c_map_.boxes[cl].angle = 0.0;
        }
        ROS_INFO ("Number of voxels in the collision map: %d.", (int)c_map_.boxes.size ());

        cmap_pub_.publish (c_map_);
      }
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    inline void
      getVoxelIndex (double *point, const Point32 &leaf_width, const Point32 &min_b, int *idx)
    {
      idx[0] = (int)(floor ((point[0] - min_b.x) / leaf_width.x));
      idx[1] = (int)(floor ((point[1] - min_b.y) / leaf_width.y));
      idx[2] = (int)(floor ((point[2] - min_b.z) / leaf_width.z));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    inline void
      getVoxelIndex (const Point32& point, const Point32 &leaf_width, const Point32 &min_b, int *idx)
    {
      idx[0] = (int)(floor ((point.x - min_b.x) / leaf_width.x));
      idx[1] = (int)(floor ((point.y - min_b.y) / leaf_width.y));
      idx[2] = (int)(floor ((point.z - min_b.z) / leaf_width.z));
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
      detectTable (const PointCloud &cloud,
                   PointCloud &cloud_out, vector<int> &table_inliers, vector<double> &table_plane_coeff, Polygon3D &table_polygon)
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
        cloud_geometry::downsamplePointCloud (cloud, indices_in_bounds, cloud_out, leaf_width_, leaves, -1);
      }
      catch (std::bad_alloc)
      {
        // downsamplePointCloud should issue a ROS_ERROR on screen, so we simply exit here
        return (false);
      }
      ROS_INFO ("Number of points after downsampling with a leaf of size [%f,%f,%f]: %d.", leaf_width_.x, leaf_width_.y, leaf_width_.z, (int)cloud_out.pts.size ());

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
      cloud_geometry::nearest::computePointCloudNormals (cloud_out, cloud, k_, viewpoint);

      // ---[ Select points whose normals are perpendicular to the Z-axis
      vector<int> indices_z;
      cloud_geometry::getPointIndicesAxisParallelNormals (cloud_out, 0, 1, 2, eps_angle_, axis_, indices_z);
      ROS_INFO ("Number of points with normals parallel to Z: %d.", (int)indices_z.size ());

      vector<vector<int> > clusters;
      // Split the Z-parallel points into clusters
      cloud_geometry::nearest::extractEuclideanClusters (cloud_out, indices_z,
          table_clusters_growing_tolerance_, clusters, 0, 1, 2, table_region_angle_threshold_, table_clusters_min_pts_);

      sort (clusters.begin (), clusters.end (), compareRegions);
      ROS_INFO ("Number of clusters found: %d, largest cluster: %d.", (int)clusters.size (), (int)clusters[clusters.size () - 1].size ());

      int c_good = -1;
      double eps_angle_deg = angles::to_degrees (eps_angle_);
      for (int i = clusters.size () - 1; i >= 0; i--)
      {
        // Find the best plane in this cluster
        fitSACPlane (&cloud_out, clusters[i], table_inliers, table_plane_coeff, viewpoint, sac_distance_threshold_);
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
      cloud_geometry::areas::convexHull2D (cloud_out, table_inliers, table_plane_coeff, table_polygon);
      return (true);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      extractObjectClusters (const PointCloud &cloud, const vector<double> &coeff, const Polygon3D &table,
                             const Point32 &axis, const Point32 &min_p, const Point32 &max_p,
                             vector<vector<int> > &object_clusters)
    {
      int nr_p = 0;
      Point32 pt;
      vector<int> object_indices (cloud.pts.size ());

      // Iterate over the entire cloud to extract the object clusters
      for (unsigned int i = 0; i < cloud.pts.size (); i++)
      {
        if ( cloud.pts.at (i).x < min_p.x || cloud.pts.at (i).x > max_p.x || cloud.pts.at (i).y < min_p.y || cloud.pts.at (i).y > max_p.y )
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
      cloud_geometry::nearest::extractEuclideanClusters (cloud, object_indices, object_cluster_tolerance_,
                                                         object_clusters, -1, -1, -1, -1, object_cluster_min_pts_);

      Point32 min_p_cluster, max_p_cluster;

      int nr_clusters = 0;
      for (unsigned int i = 0; i < object_clusters.size (); i++)
      {
        // Check whether this object cluster is supported by the table or just flying through thin air
        cloud_geometry::statistics::getMinMax (cloud, object_clusters.at (i), min_p_cluster, max_p_cluster);

        // If this cluster does not satisfy our constraints, remove it from the list
        if ( min_p_cluster.z > max_p.z + object_min_dist_from_table_ )
        {
          object_clusters[i].resize (0);
          continue;
        }
        nr_clusters++;
      }
      ROS_INFO ("Number of object clusters found: %d", nr_clusters);
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
          ROS_WARN ("fitSACPlane: Inliers.size (%d) < sac_min_points_per_model (%d)!", (int)sac->getInliers ().size (), clusters_min_pts_);
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

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Obtain a 24-bit RGB coded value from 3 independent <r, g, b> channel values
      * \param r the red channel value
      * \param g the green channel value
      * \param b the blue channel value
      */
    inline double
      getRGB (float r, float g, float b)
    {
      int res = (int(r * 255) << 16) | (int(g*255) << 8) | int(b*255);
      double rgb = *(float*)(&res);
      return (rgb);
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "cluster_voxelization");

  ClusterVoxelization p;
  p.publish_debug_ = false;
  ros::spin ();

  return (0);
}
/* ]--- */
