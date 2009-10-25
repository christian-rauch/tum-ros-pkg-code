/*
 * Copyright (c) 2008 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
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
 * $Id: table_object_detector.cpp 24607 2009-10-07 02:42:21Z gerkey $
 *
 */

/**
@mainpage

@htmlinclude manifest.html

\author Radu Bogdan Rusu

@b table_object_detector detects tables and objects.

 **/

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

#include <point_cloud_mapping/cloud_io.h>

#include <sys/time.h>

#include <ias_table_msgs/TableWithObjects.h>

using namespace std;
using namespace std_msgs;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace mapping_msgs;

// Comparison operator for a vector of vectors
bool
  compareRegions (const std::vector<int> &a, const std::vector<int> &b)
{
  return (a.size () < b.size ());
}

class TableObjectDetector
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

    tf::TransformListener tf_;

    // Parameters
    string global_frame_;
    double min_z_bounds_, max_z_bounds_;
    string output_table_topic_;
    string input_cloud_topic_;
    int k_;
    double clusters_growing_tolerance_;
    int clusters_min_pts_;

    int object_cluster_min_pts_;
    double object_cluster_tolerance_;

    bool publish_debug_;

    double sac_distance_threshold_, eps_angle_, region_angle_threshold_;

    double table_min_height_, table_max_height_, delta_z_, object_min_distance_from_table_;

    ros::Subscriber cloud_sub_;
    ros::Publisher semantic_map_publisher_, cloud_publisher_, table_pub_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    TableObjectDetector (ros::NodeHandle& anode) : node_ (anode)
    {
      node_.param ("/global_frame_id", global_frame_, std::string("/map"));

      node_.param ("min_z_bounds", min_z_bounds_, 0.0);                      // restrict the Z dimension between 0
      node_.param ("max_z_bounds", max_z_bounds_, 3.0);                      // and 3.0 m

      // Use downsampling internally to estimate solutions faster.
      {
        node_.param ("downsample_leaf_width_x", leaf_width_.x, 0.03);          // 3cm radius by default
        node_.param ("downsample_leaf_width_y", leaf_width_.y, 0.03);          // 3cm radius by default
        node_.param ("downsample_leaf_width_z", leaf_width_.z, 0.03);          // 3cm radius by default
      }
      node_.param ("search_k_closest", k_, 10);                              // 10 k-neighbors by default

      z_axis_.x = 0; z_axis_.y = 0; z_axis_.z = 1;
      node_.param ("normal_eps_angle", eps_angle_, 15.0);                   // 15 degrees
      eps_angle_ = angles::from_degrees (eps_angle_);                        // convert to radians

      node_.param ("region_angle_threshold", region_angle_threshold_, 30.0);   // Difference between normals in degrees for cluster/region growing
      region_angle_threshold_ = angles::from_degrees (region_angle_threshold_); // convert to radians

      node_.param ("clusters_growing_tolerance", clusters_growing_tolerance_, 0.5);   // 0.5 m
      node_.param ("clusters_min_pts", clusters_min_pts_, 10);                        // 10 points

      // These parameters define what we mean by "separate clusters" (e.g., separate objects).
      {
        node_.param ("object_cluster_dist_tolerance", object_cluster_tolerance_, 0.05);   // 5cm between two objects
        node_.param ("object_cluster_min_pts", object_cluster_min_pts_, 30);              // minimum 30 points per object cluster
      }

      // Optimization parameters: used to reduce the search space for horizontal planar search.
      {
        node_.param ("table_min_height", table_min_height_, 0.5);              // minimum height of a table : 0.5m
        node_.param ("table_max_height", table_max_height_, 1.5);              // maximum height of a table : 1.5m
      }
      node_.param ("table_delta_z", delta_z_, 0.03);                         // consider objects starting at 3cm from the table
      node_.param ("object_min_distance_from_table", object_min_distance_from_table_, 0.10); // objects which have their support more 10cm from the table will not be considered
      ROS_DEBUG ("Using the following thresholds for table detection [min / max height]: %f / %f.", table_min_height_, table_max_height_);

      // Used to publish the results as additional messages. 
      {
        node_.param ("publish_debug", publish_debug_, true);
      }

      node_.param ("output_table_topic", output_table_topic_, string ("table"));
      node_.param ("input_cloud_topic", input_cloud_topic_, string ("cloud_pcd"));

      // This should be set to whatever the leaf_width factor is in the downsampler
      node_.param ("sac_distance_threshold", sac_distance_threshold_, 0.03);     // 5 cm

      if (publish_debug_)
      {
        semantic_map_publisher_ = node_.advertise<PolygonalMap> ("semantic_polygonal_map", 1);
        cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud> ("cloud_annotated", 1);
      }

      cloud_sub_ = node_.subscribe (input_cloud_topic_, 1, &TableObjectDetector::cloud_cb, this);
      table_pub_ = node_.advertise<ias_table_msgs::TableWithObjects> (output_table_topic_, 1);
      ROS_INFO ("callback registered");
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
    void
      findObjectClusters (sensor_msgs::PointCloud &points, const vector<double> &coeff, const Polygon &poly,
                          const geometry_msgs::Point32 &minP, const geometry_msgs::Point32 &maxP,
                          vector<int> &object_indices, ias_table_msgs::TableWithObjects &table)
    {
      int nr_p = 0;
      geometry_msgs::Point32 pt;
      object_indices.resize (points.points.size ());
      for (unsigned int i = 0; i < points.points.size (); i++)
      {
        // Select all the points in the given bounds
        if ( points.points.at (i).x > minP.x &&
             points.points.at (i).x < maxP.x &&
             points.points.at (i).y > minP.y &&
             points.points.at (i).y < maxP.y &&
             points.points.at (i).z > (maxP.z + delta_z_)
           )
        {
          // Calculate the distance from the point to the plane
          double distance_to_plane = coeff.at (0) * points.points.at (i).x +
                                     coeff.at (1) * points.points.at (i).y +
                                     coeff.at (2) * points.points.at (i).z +
                                     coeff.at (3) * 1;
          // Calculate the projection of the point on the plane
          pt.x = points.points.at (i).x - distance_to_plane * coeff.at (0);
          pt.y = points.points.at (i).y - distance_to_plane * coeff.at (1);
          pt.z = points.points.at (i).z - distance_to_plane * coeff.at (2);

          if (cloud_geometry::areas::isPointIn2DPolygon (pt, poly))
          {
            object_indices[nr_p] = i;
            nr_p++;
          }
        }
      }
      object_indices.resize (nr_p);

      // Find the clusters
      nr_p = 0;
      vector<vector<int> > object_clusters;
      cloud_geometry::nearest::extractEuclideanClusters (points, object_indices, object_cluster_tolerance_, object_clusters, -1, -1, -1, -1, object_cluster_min_pts_);

      geometry_msgs::Point32 minPCluster, maxPCluster;
      table.objects.resize (object_clusters.size ());
      for (unsigned int i = 0; i < object_clusters.size (); i++)
      {
        vector<int> object_idx = object_clusters.at (i);

        // Check whether this object cluster is supported by the table or just flying through thin air
        cloud_geometry::statistics::getMinMax (points, object_idx, minPCluster, maxPCluster);
        if (minPCluster.z > (maxP.z + object_min_distance_from_table_) )
            continue;

        table.objects[i].points.points.resize (object_idx.size ());

        table.objects[i].points.channels.resize(1);
        table.objects[i].points.channels[0].name="intensities";
        table.objects[i].points.channels[0].values.resize (object_idx.size());
//         int intensity_idx = cloud_io::getIndex (&points, "intensities");
        // Process this cluster and extract the centroid and the bounds
        for (unsigned int j = 0; j < object_idx.size (); j++)
        {
          table.objects[i].points.points.at (j) = points.points.at (object_idx.at (j));
          object_indices[nr_p] = object_idx.at (j);
          nr_p++;
        }
        
        cloud_geometry::statistics::getMinMax (points, object_idx, table.objects[i].min_bound, table.objects[i].max_bound);
        cloud_geometry::nearest::computeCentroid (points, object_idx, table.objects[i].center);
      }
      ROS_INFO ("created %i clusters with %i points total.", object_clusters.size (), nr_p);
      object_indices.resize (nr_p);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Callback
    void cloud_cb (const sensor_msgs::PointCloudConstPtr& pc)
    {
      cloud_in_ = *pc;
      tf_.transformPointCloud(global_frame_, *pc, cloud_in_);
      
      ros::Time ts = ros::Time::now ();
      // We have a pointcloud, estimate the true point bounds
      vector<int> indices_in_bounds (cloud_in_.points.size ());
      int nr_p = 0;
      for (unsigned int i = 0; i < cloud_in_.points.size (); i++)
      {
        if (cloud_in_.points[i].z >= table_min_height_ && cloud_in_.points[i].z <= table_max_height_)
        {
          indices_in_bounds[nr_p] = i;
          nr_p++;
        }
      }
      indices_in_bounds.resize (nr_p);
      ROS_DEBUG ("%d of %d points are within the table height bounds of [%.2lf,%.2lf]",
                 nr_p, (int)cloud_in_.points.size (), table_min_height_, table_max_height_);

      // Downsample the cloud in the bounding box for faster processing
      // NOTE: <leaves_> gets allocated internally in downsamplePointCloud() and is not deallocated on exit
      vector<cloud_geometry::Leaf> leaves;
      try
      {
        cloud_geometry::downsamplePointCloud (cloud_in_, indices_in_bounds, cloud_down_, leaf_width_, leaves, -1);
      }
      catch (std::bad_alloc)
      {
        // downsamplePointCloud should issue a ROS_ERROR on screen, so we simply exit here
        return;
      }

      ROS_DEBUG ("Number of points after downsampling with a leaf of size [%f,%f,%f]: %d.", leaf_width_.x, leaf_width_.y, leaf_width_.z, (int)cloud_down_.points.size ());

      cloud_down_.header = cloud_in_.header;
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
      cloud_geometry::getPointIndicesAxisParallelNormals (cloud_down_, 0, 1, 2, eps_angle_, z_axis_, indices_z);
      ROS_DEBUG ("Number of points with normals parallel to Z: %d.", (int)indices_z.size ());

      vector<vector<int> > clusters;
      // Split the Z-parallel points into clusters
      cloud_geometry::nearest::extractEuclideanClusters (cloud_down_, indices_z, clusters_growing_tolerance_, clusters, 0, 1, 2, region_angle_threshold_, clusters_min_pts_);

      sort (clusters.begin (), clusters.end (), compareRegions);

      vector<int> inliers;
      vector<double> coeff, z_coeff (3);
      z_coeff[0] = z_axis_.x; z_coeff[1] = z_axis_.y; z_coeff[2] = z_axis_.z;
      int c_good = -1;
      double eps_angle_deg = angles::to_degrees (eps_angle_);
      for (int i = clusters.size () - 1; i >= 0; i--)
      {
        // Find the best plane in this cluster
        fitSACPlane (&cloud_down_, &clusters[i], inliers, coeff);
        double angle = angles::to_degrees (cloud_geometry::angles::getAngleBetweenPlanes (coeff, z_coeff));
        if ( fabs (angle) < eps_angle_deg || fabs (180.0 - angle) < eps_angle_deg )
        {
          c_good = i;
          break;
        }
      }

      if (c_good == -1)
      {
        ROS_WARN ("No table found");
        return;
      }
      ROS_INFO ("Number of clusters found: %d, largest cluster: %d.", (int)clusters.size (), (int)clusters[c_good].size ());

      // Fill in the header
      ias_table_msgs::TableWithObjects table;
      table.header.frame_id = cloud_in_.header.frame_id;
      table.header.stamp = cloud_in_.header.stamp;

      // Get the table bounds
      geometry_msgs::Point32 minP, maxP;
      cloud_geometry::statistics::getMinMax (cloud_down_, inliers, minP, maxP);
      // Transform to the global frame
      geometry_msgs::PointStamped minPstamped_local, maxPstamped_local;
      minPstamped_local.point.x = minP.x;
      minPstamped_local.point.y = minP.y;
      minPstamped_local.point.z = minP.z;
      minPstamped_local.header = cloud_in_.header;
      maxPstamped_local.point.x = maxP.x;
      maxPstamped_local.point.y = maxP.y;
      maxPstamped_local.point.z = maxP.z;
      maxPstamped_local.header = cloud_in_.header;
      geometry_msgs::PointStamped minPstamped_global, maxPstamped_global;
      try
      {
        tf_.transformPoint (global_frame_, minPstamped_local, minPstamped_global);
        tf_.transformPoint (global_frame_, maxPstamped_local, maxPstamped_global);
        table.table_min.x = minPstamped_global.point.x;
        table.table_min.y = minPstamped_global.point.y;
        table.table_min.z = minPstamped_global.point.z;
        table.table_max.x = maxPstamped_global.point.x;
        table.table_max.y = maxPstamped_global.point.y;
        table.table_max.z = maxPstamped_global.point.z;
      }
      catch (tf::TransformException)
      {
        ROS_ERROR ("Failed to transform table bounds from frame %s to frame %s", cloud_in_.header.frame_id.c_str (), global_frame_.c_str ());
        return;
      }

      // Compute the convex hull
      pmap_.header.stamp = cloud_down_.header.stamp;
      pmap_.header.frame_id = cloud_down_.header.frame_id;
      pmap_.polygons.resize (1);
      cloud_geometry::areas::convexHull2D (cloud_down_, inliers, coeff, pmap_.polygons[0]);

      // Find the object clusters supported by the table
      inliers.clear ();
      findObjectClusters (cloud_in_, coeff, pmap_.polygons[0], minP, maxP, inliers, table);

      // Transform into the global frame
      try
      {
        geometry_msgs::PointStamped local, global;
        local.header = cloud_down_.header;
        for (unsigned int i = 0; i < pmap_.polygons.size (); i++)
        {
          for (unsigned int j = 0; j < pmap_.polygons[i].points.size (); j++)
          {
            local.point.x = pmap_.polygons[i].points[j].x;
            local.point.y = pmap_.polygons[i].points[j].y;
            tf_.transformPoint (global_frame_, local, global);
            pmap_.polygons[i].points[j].x = global.point.x;
            pmap_.polygons[i].points[j].y = global.point.y;
          }
        }
      }
      catch (tf::TransformException)
      {
        ROS_ERROR ("Failed to PolygonalMap from frame %s to frame %s", cloud_down_.header.frame_id.c_str(), global_frame_.c_str());
        return;
      }

      table.table = pmap_.polygons[0];


      ROS_INFO ("Table found. Bounds: [%f, %f, %f] -> [%f, %f, %f]. Number of objects: %d. Total time: %f.",
                minP.x, minP.y, minP.z, maxP.x, maxP.y, maxP.z, (int)table.objects.size (), (ros::Time::now () - ts).toSec ());
      ROS_INFO ("Table found. Bounds: [%f, %f, %f] -> [%f, %f, %f]. Number of objects: %d. Total time: %f.",
                table.table_min.x, table.table_min.y, table.table_min.z, table.table_max.x, table.table_max.y, table.table_max.z, (int)table.objects.size (), (ros::Time::now () - ts).toSec ());

      table_pub_.publish (table);
      // Should only used for debugging purposes (on screen visualization)
      if (publish_debug_)
      {
        // Break the object inliers into clusters in an Euclidean sense
        vector<vector<int> > objects;
        cloud_geometry::nearest::extractEuclideanClusters (cloud_in_, inliers, object_cluster_tolerance_, objects, -1, -1, -1, -1, object_cluster_min_pts_);

        int total_nr_pts = 0;
        for (unsigned int i = 0; i < objects.size (); i++)
          total_nr_pts += objects[i].size ();

        cloud_annotated_.header = cloud_down_.header;
        cloud_annotated_.points.resize (total_nr_pts);

        // Copy all the channels from the original pointcloud
        cloud_annotated_.channels.resize (cloud_in_.channels.size () + 1);
        for (unsigned int d = 0; d < cloud_in_.channels.size (); d++)
        {
          cloud_annotated_.channels[d].name = cloud_in_.channels[d].name;
          cloud_annotated_.channels[d].values.resize (total_nr_pts);
        }
        cloud_annotated_.channels[cloud_in_.channels.size ()].name = "rgb";
        cloud_annotated_.channels[cloud_in_.channels.size ()].values.resize (total_nr_pts);

        // For each object in the set
        int nr_p = 0;
        for (unsigned int i = 0; i < objects.size (); i++)
        {
          float rgb = getRGB (rand () / (RAND_MAX + 1.0), rand () / (RAND_MAX + 1.0), rand () / (RAND_MAX + 1.0));
          // Get its points

          for (unsigned int j = 0; j < objects[i].size (); j++)
          {
            cloud_annotated_.points[nr_p] = cloud_in_.points.at (objects[i][j]);
            for (unsigned int d = 0; d < cloud_in_.channels.size (); d++)
              cloud_annotated_.channels[d].values[nr_p] = cloud_in_.channels[d].values.at (objects[i][j]);
            cloud_annotated_.channels[cloud_in_.channels.size ()].values[nr_p] = rgb;
            nr_p++;
          }
        }
        cloud_publisher_.publish(cloud_annotated_);
        semantic_map_publisher_.publish(pmap_);
      }
      ROS_INFO ("- callback");
      return;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    int
      fitSACPlane (sensor_msgs::PointCloud *points, vector<int> *indices, vector<int> &inliers, vector<double> &coeff)
    {
      ROS_INFO ("+ fitSACPlane");
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

#pragma omp parallel for schedule(dynamic)
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
  ros::init (argc, argv, "table_object_detector_passive");
  ros::NodeHandle ros_node("~");
  TableObjectDetector p (ros_node);
  ros::spin();

  return (0);
}
/* ]--- */

