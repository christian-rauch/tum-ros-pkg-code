/*
 * Copyright (c) 2008 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
 * Dejan Pangercic <pangercic@cs.tum.edu>
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
 *
 */
/** 
@file

@brief wall_filter detects vertically oriented planes in a PointCloud, 
removes them from the original PointCloud and publishes the wall-less
PointCloud

@par Advertises
- \b input_cloud_topic with PointCloud message

@par Subscribes
- \b output_cloud_topic with PointCloud message (wall-less PointCloud)


@par Parameters
-    string input_cloud_topic;
-    string output_cloud_topic;
-    int k;
-    double clusters_growing_tolerance;
-    int clusters_min_pts;
-    double sac_distance_threshold, eps_angle, region_angle_threshold;
*/

// ROS core
#include <ros/ros.h>
// ROS messages
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Polygon.h>
#include <mapping_msgs/PolygonalMap.h>

// Sample Consensus
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/msac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>

#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
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

using namespace std;
using namespace std_msgs;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace mapping_msgs;

// Comparison operator for a vector of vectors
/**
 * \brief Comparison operator for a vector of vectors
 * \param a first vector
 * \param b second vector
 */
bool
  compareRegions (const std::vector<int> &a, const std::vector<int> &b)
{
  return (a.size () < b.size ());
}

class WallFilter
{
  protected:
    ros::NodeHandle& node_;
  public:

    // ROS messages
    sensor_msgs::PointCloud cloud_in_, cloud_down_;
    geometry_msgs::Point leaf_width_;
    sensor_msgs::PointCloud cloud_annotated_;
    geometry_msgs::Point32 z_axis_;
    PolygonalMap pmap_;

    // Parameters
    string input_cloud_topic_;
    string output_cloud_topic_;
    int k_;
    double clusters_growing_tolerance_;
    int clusters_min_pts_;
    double sac_distance_threshold_, eps_angle_, region_angle_threshold_;

    //Subscribers/Publishers
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
    * \brief Constructor initializes parameters
    */
    WallFilter (ros::NodeHandle& anode) : node_ (anode)
    {
      // Use downsampling internally to estimate solutions faster.
      {
        node_.param ("downsample_leaf_width_x", leaf_width_.x, 0.06);          // 3cm radius by default
        node_.param ("downsample_leaf_width_y", leaf_width_.y, 0.06);          // 3cm radius by default
        node_.param ("downsample_leaf_width_z", leaf_width_.z, 0.06);          // 3cm radius by default
      }
      node_.param ("search_k_closest", k_, 10);                              // 10 k-neighbors by default

      z_axis_.x = 0; z_axis_.y = 0; z_axis_.z = 1;
      node_.param ("normal_eps_angle", eps_angle_, 15.0);                   // 15 degrees
      eps_angle_ = angles::from_degrees (eps_angle_);                        // convert to radians

      node_.param ("region_angle_threshold", region_angle_threshold_, 30.0);   // Difference between normals in degrees for cluster/region growing
      region_angle_threshold_ = angles::from_degrees (region_angle_threshold_); // convert to radians

      node_.param ("clusters_growing_tolerance", clusters_growing_tolerance_, 0.5);   // 0.5 m
      node_.param ("clusters_min_pts", clusters_min_pts_, 10);                        // 10 points

      node_.param ("input_cloud_topic", input_cloud_topic_, string ("/cloud_pcd"));
      node_.param ("output_cloud_topic", output_cloud_topic_, string ("/cloud_without_walls"));

      // This should be set to whatever the leaf_width factor is in the downsampler
      node_.param ("sac_distance_threshold", sac_distance_threshold_, 0.03);     // 5 cm

      cloud_pub_ = node_.advertise<sensor_msgs::PointCloud> (output_cloud_topic_, 1);
      cloud_sub_ = node_.subscribe (input_cloud_topic_, 1, &WallFilter::cloud_cb, this);
      ROS_INFO ("callback registered");
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
    * \brief updates parameters from server
    */
    void
      updateParametersFromServer ()
    {
      if (node_.hasParam ("input_cloud_topic"))
        node_.getParam ("input_cloud_topic", input_cloud_topic_);
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
      int res = (int (r * 255) << 16) | (int (g*255) << 8) | int (b*255);
      double rgb = *(float*)(&res);
      return (rgb);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** 
    * \brief point cloud  callback, every other function gets called from here
    * \param pc input point cloud
    */
    void cloud_cb (const sensor_msgs::PointCloudConstPtr& pc)
    {
      ROS_INFO ("+ callback");
      cloud_in_ = *pc;
      
      ros::Time ts = ros::Time::now ();

      // Downsample the cloud in the bounding box for faster processing
      // NOTE: <leaves_> gets allocated internally in downsamplePointCloud() and is not deallocated on exit
      vector<cloud_geometry::Leaf> leaves;
      try
      {
        cloud_geometry::downsamplePointCloud (cloud_in_, cloud_down_, leaf_width_, leaves, -1);
      }
      catch (std::bad_alloc)
      {
        // downsamplePointCloud should issue a ROS_ERROR on screen, so we simply exit here
        return;
      }

      ROS_DEBUG ("Number of points after downsampling with a leaf of size [%f,%f,%f]: %d.", 
                 leaf_width_.x, leaf_width_.y, leaf_width_.z, (int)cloud_down_.points.size ());

      // Reserve space for 3 channels: nx, ny, nz
      cloud_down_.channels.resize (3);
      cloud_down_.channels[0].name = "nx";
      cloud_down_.channels[1].name = "ny";
      cloud_down_.channels[2].name = "nz";
      for (unsigned int d = 0; d < cloud_down_.channels.size (); d++)
        cloud_down_.channels[d].values.resize (cloud_down_.points.size ());

      // Create Kd-Tree
      estimatePointNormals (cloud_down_);

      // ---[ Select points whose normals are perpendicular to the Z-axis
      vector<int> indices_z;
      cloud_geometry::getPointIndicesAxisPerpendicularNormals (cloud_down_, 0, 1, 2, eps_angle_, z_axis_, indices_z);
      ROS_DEBUG ("Number of points with normals perpendicular to Z: %d.", (int)indices_z.size ());

      vector<vector<int> > clusters;
      // Split the Z-parallel points into clusters
      cloud_geometry::nearest::extractEuclideanClusters (cloud_down_, indices_z, clusters_growing_tolerance_, clusters, 0, 1, 2, region_angle_threshold_, clusters_min_pts_);

      sort (clusters.begin (), clusters.end (), compareRegions);

      vector<int> inliers;
      vector<double> coeff, z_coeff (3);
      z_coeff[0] = z_axis_.x; z_coeff[1] = z_axis_.y; z_coeff[2] = z_axis_.z;
      std::vector<int> good_walls;
      std::vector<int> indices_wall_points;

      double eps_angle_deg = angles::to_degrees (eps_angle_);
      for (int i = clusters.size () - 1; i >= 0; i--)
      {
        std::vector<int> cur_clust = (clusters[i]);
        int old_nr_indices;
//         do 
//         {
          old_nr_indices = cur_clust.size();
          // Find the best plane in this cluster
          if (fitSACPlane (&cloud_down_, &cur_clust, inliers, coeff) == -1)
            continue;
          if (coeff.size() < 4)
            continue;
          if ((signed int)inliers.size() < clusters_min_pts_)
            continue;
          double angle = angles::to_degrees (cloud_geometry::angles::getAngleBetweenPlanes (coeff, z_coeff));
          Polygon wall_polygon;
          cloud_geometry::areas::convexHull2D (cloud_down_, inliers, coeff, wall_polygon);
          double area = cloud_geometry::areas::compute2DPolygonalArea (wall_polygon, coeff);
            if ( fabs (angle - 90.0) < eps_angle_deg)
            //|| fabs (180.0 - angle) < eps_angle_deg )
          {
            if (area > 0.8)
            {
              for (unsigned int j = 0; j < cloud_in_.points.size(); j++)
                if (cloud_geometry::distances::pointToPlaneDistance (cloud_in_.points.at(j), coeff) < sac_distance_threshold_)
                  indices_wall_points.push_back (j);
              good_walls.push_back (i);
            }
          }
      }

      if (good_walls.size() == 0)
      {
        ROS_WARN ("No wall found");
        return;
      }
      ROS_INFO ("Number of clusters found: %d, %d of which are walls, largest cluster: %d.", (int)clusters.size (), good_walls.size(), (int)clusters[good_walls[0]].size ());

      PointCloud pcd_out;
      pcd_out.header = cloud_in_.header;
      pcd_out.points.resize (cloud_in_.points.size());
      pcd_out.channels.resize (cloud_in_.channels.size ());
      for (int j = 0; j < (int)cloud_in_.channels.size(); j++)
      {
        pcd_out.channels[j].name = cloud_in_.channels[j].name;
        pcd_out.channels[j].values.resize (cloud_in_.points.size());
      }

      int cur_point_wall = 0;
      int cur_point_out = 0;
      for (int j = 0; j < (int)cloud_in_.points.size(); j++)
        if (indices_wall_points[cur_point_wall] != j)
        {
          for (int c = 0; c < (int)cloud_in_.channels.size(); c++)
            pcd_out.channels[c].values.at(cur_point_out) = cloud_in_.channels[c].values.at(j);
          pcd_out.points.at(cur_point_out) = cloud_in_.points.at(j);
          cur_point_out++;
        }
        else
          cur_point_wall ++;
      pcd_out.points.resize (cur_point_out);
      for (int j = 0; j < (int)cloud_in_.channels.size(); j++)
        pcd_out.channels[j].values.resize (cur_point_out);

      cloud_pub_.publish (pcd_out); 
      ROS_INFO ("# points in: %i, wall: %i, out: %i (total time spent: %f)", cloud_in_.points.size(), cur_point_wall, cur_point_out, (ros::Time::now () - ts).toSec());
      ROS_INFO ("- callback");
      return;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
    * \brief fits a plane to the set of 3D points
    * \param points set of 3D points
    * \param indices points' vector indices
    * \param inliers indices of in-lying points (to be filled) 
    * \param coeff plane equation coefficients
    */
    int
      fitSACPlane (sensor_msgs::PointCloud *points, vector<int> *indices, vector<int> &inliers, vector<double> &coeff)
    {
      ROS_INFO ("+ fitSACPlane %i", indices->size ());
      if ((int)indices->size () < clusters_min_pts_)
      {
        inliers.resize (0);
        coeff.resize (0);
        return (-1);
      }

      // Create and initialize the SAC model
      sample_consensus::SACModelPlane *model = new sample_consensus::SACModelPlane ();
      sample_consensus::SAC *sac             = new sample_consensus::MSAC (model, sac_distance_threshold_);
      sac->setMaxIterations (500);
      sac->setProbability (0.99);
      model->setDataSet (points, *indices);

      // Search for the best plane
      if (sac->computeModel ())
      {
        // Obtain the inliers and the planar model coefficients
        if ((int)sac->getInliers ().size () < clusters_min_pts_)
        {
          //ROS_ERROR ("fitSACPlane: Inliers.size (%d) < sac_min_points_per_model (%d)!", sac->getInliers ().size (), sac_min_points_per_model_);
          inliers.resize (0);
          coeff.resize (0);
          return (0);
        }
        sac->computeCoefficients (coeff);     // Compute the model coefficients
        sac->refineCoefficients (coeff);      // Refine them using least-squares
        model->selectWithinDistance (coeff, sac_distance_threshold_, inliers);

        //fprintf (stderr, "> Found a model supported by %d inliers: [%g, %g, %g, %g]\n", sac->getInliers ().size (),
        //         coeff[coeff.size () - 1][0], coeff[coeff.size () - 1][1], coeff[coeff.size () - 1][2], coeff[coeff.size () - 1][3]);

        // Project the inliers onto the model
        model->projectPointsInPlace (inliers, coeff);
      }
      ROS_INFO ("- fitSACPlane");
      return (0);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
    * \brief estimates normals for all 3D points
    * \param cloud input point cloud data
    */
    void
      estimatePointNormals (sensor_msgs::PointCloud &cloud)
    {
      ROS_INFO ("+ estimatePointNormals");
      cloud_kdtree::KdTree *kdtree = new cloud_kdtree::KdTreeANN (cloud);
      vector<vector<int> > points_k_indices;
      // Allocate enough space for point indices
      points_k_indices.resize (cloud.points.size ());
      for (int i = 0; i < (int)cloud.points.size (); i++)
        points_k_indices[i].resize (k_);
      // Get the nerest neighbors for all the point indices in the bounds
      vector<float> distances;
      for (int i = 0; i < (int)cloud.points.size (); i++)
        kdtree->nearestKSearch (i, k_, points_k_indices[i], distances);

      // Figure out the viewpoint value in the point cloud frame
      geometry_msgs::PointStamped viewpoint_laser, viewpoint_cloud;
      viewpoint_laser.header.frame_id = "laser_tilt_mount_link";
      // Set the viewpoint in the laser coordinate system to 0,0,0
      viewpoint_laser.point.x = viewpoint_laser.point.y = viewpoint_laser.point.z = 0.0;

      try
      {
//         tf_.transformPoint (cloud.header.frame_id, viewpoint_laser, viewpoint_cloud);
      }
      catch (tf::TransformException)
      {
        viewpoint_cloud.point.x = viewpoint_cloud.point.y = viewpoint_cloud.point.z = 0.0;
      }

      for (int i = 0; i < (int)cloud.points.size (); i++)
      {
        // Compute the point normals (nx, ny, nz), surface curvature estimates (c)
        Eigen::Vector4d plane_parameters;
        double curvature;
        cloud_geometry::nearest::computePointNormal (cloud, points_k_indices[i], plane_parameters, curvature);

        cloud_geometry::angles::flipNormalTowardsViewpoint (plane_parameters, cloud_down_.points[i], viewpoint_cloud);

        cloud.channels[0].values[i] = plane_parameters (0);
        cloud.channels[1].values[i] = plane_parameters (1);
        cloud.channels[2].values[i] = plane_parameters (2);
      }
      // Delete the kd-tree
      ROS_INFO ("- estimatePointNormals");
      delete kdtree;
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "wall_filter");
  ros::NodeHandle ros_node("~");
  WallFilter p (ros_node);
  ros::spin();

  return (0);
}
/* ]--- */

