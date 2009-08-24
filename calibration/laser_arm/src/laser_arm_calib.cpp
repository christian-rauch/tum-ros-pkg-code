/*
 * Copyright (c) 2009 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
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
 * $Id$
 *
 */

/** \author Radu Bogdan Rusu */

// ROS core
#include <ros/node.h>
// ROS messages
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>

// Cloud geometry
#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/statistics.h>

#include <cminpack.h>

using namespace std;
using namespace sensor_msgs;
using namespace geometry_msgs;

class ExtendedLeaf
{
  public:
    float centroid_x, centroid_y, centroid_z;
    vector<int> indices;
};

class LaserArmCalib
{
  protected:
    ros::Node& node_;

  public:

    // ROS messages
    PointCloud cloud_, cloud_down_, cloud_plane_;

    Point leaf_width_;
    vector<ExtendedLeaf> leaves_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    LaserArmCalib (ros::Node& anode) : node_ (anode)
    {
      node_.param ("~downsample_leaf_width_x", leaf_width_.x, 0.03);      // 3cm radius by default
      node_.param ("~downsample_leaf_width_y", leaf_width_.y, 0.03);      // 3cm radius by default
      node_.param ("~downsample_leaf_width_z", leaf_width_.z, 0.03);      // 3cm radius by default

      string cloud_topic ("tilt_laser_cloud");

      vector<pair<string, string> > t_list;
      node_.getPublishedTopics (&t_list);
      bool topic_found = false;
      for (vector<pair<string, string> >::iterator it = t_list.begin (); it != t_list.end (); it++)
      {
        if (it->first.find (node_.mapName (cloud_topic)) != string::npos)
        {
          topic_found = true;
          break;
        }
      }
      if (!topic_found)
        ROS_WARN ("Trying to subscribe to %s, but the topic doesn't exist!", node_.mapName (cloud_topic).c_str ());

      node_.subscribe (cloud_topic, cloud_, &LaserArmCalib::cloud_cb, this, 1);
      node_.advertise<PointCloud> ("~plane", 1);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////(
    /** \brief Calibration cost function to be minimized
      * \param p a pointer to our data structure array
      * \param m the number of functions
      * \param n the number of variables
      * \param x a pointer to the variables array
      * \param fvec a pointer to the resultant functions evaluations
      * \param iflag set to -1 inside the function to terminate execution
      */
    static int
      functionToOptimize (void *p, int m, int n, const double *x, double *fvec, int iflag)
    {
      PointCloud *points = (PointCloud*)p;

      // Get the point normal
      float normal[3];
      float eig1, eig2;

      // Assemble the cost function
      for (unsigned int i = 0; i < points->points.size (); i++)
      {
        // Get the point normal
        normal[0] = points->channels[0].values[i];
        normal[1] = points->channels[1].values[i];
        normal[2] = points->channels[2].values[i];
        eig1 = points->channels[3].values[i];
        eig2 = points->channels[4].values[i];

        // Use the rotation / translation values from X

        // Update the cost function
        fvec[i] = eig1 - eig2;
      }
      return (0);
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Obtain a solution for a rotation and a translation between the laser sensor and the pan/tilt unit
      * by non-linear minimizing of a cost function describing the relations between the data and the groundtruth.
      * \NOTE We aim to obtain 3 rotations and 3 translations between the optical center of the laser and the center of
      * rotation of the pan/tilt unit (robot arm in our case). To do this, we need a scan of a vertical wall
      * perpendicular to the Z axis, and minimize a relation betwee the estimated eigenvalues and eigenvectors of each
      * point with respect to the wall.
      * \param cloud a downsampled (voxelized) point cloud representation, with at least 5 data channels
      * (nx, ny, nz, l1, l2)
      */
    void
      optimizeCalibrationTransformation (PointCloud *cloud)
    {
      if (cloud->channels.size () < 5)
        return;
      int m = cloud->points.size ();
      double fvec[m];

      int n = 6;      // 6 unknowns
      int iwa[n];

      int lwa = m * n + 5 * n + m;
      double wa[lwa];

      // Set the initial solution
      double x[n];
      x[0] = 0.0492;
      x[1] = -0.00125;
      x[2] = -0.1136;
      x[3] = 0;
      x[4] = 0;
      x[5] = 0;

      // Set tol to the square root of the machine. Unless high solutions are required, these are the recommended settings.
      double tol = sqrt (dpmpar (1));

      // Optimize using forward-difference approximation LM
      int info = lmdif1 (functionToOptimize, cloud, m, n, x, fvec, tol, iwa, wa, lwa);

      // Compute the L2 norm of the residuals
      ROS_INFO ("Residuals norm: %15.7f", enorm (m, fvec));
      ROS_INFO ("Exit parameter: %10i", info);
      ROS_INFO ("Final solution (rotation/translation): %g %g %g / %g % g %g", x[3], x[4], x[5], x[0], x[1], x[2]);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** Downsample a point cloud dataset using fixed-size voxels and estimate the eigenvalues and eigenvectors for each
      * voxel.
      * \param cloud the input point cloud
      * \param cloud_down the resultant downsampled point cloud
      * \param leaf_width the individual width of the voxels along 3 dimensions
      * \param leaves a resultant vector of voxels
      */
    void
      downsampleEstimateEigen (const PointCloud &cloud, PointCloud &cloud_down, Point leaf_width, vector<ExtendedLeaf> &leaves)
    {
      // Copy the header (and thus the frame_id) + allocate enough space for points
      cloud_down.header = cloud.header;
      cloud_down.points.resize (cloud.points.size ());
      // We need the normal + the 2 largest eigenvalues
      cloud_down.channels.resize (5);
      cloud_down.channels[0].name = "nx";
      cloud_down.channels[1].name = "ny";
      cloud_down.channels[2].name = "nz";
      cloud_down.channels[3].name = "l1";
      cloud_down.channels[4].name = "l2";
      for (unsigned int d = 0; d < cloud_down.channels.size (); d++)
        cloud_down.channels[d].values.resize (cloud_down.points.size ());

      Point32 min_p, max_p, min_b, max_b, div_b;
      cloud_geometry::statistics::getMinMax (cloud, min_p, max_p);

      // Compute the minimum and maximum bounding box values
      min_b.x = (int)(floor (min_p.x / leaf_width.x));
      max_b.x = (int)(floor (max_p.x / leaf_width.x));

      min_b.y = (int)(floor (min_p.y / leaf_width.y));
      max_b.y = (int)(floor (max_p.y / leaf_width.y));

      min_b.z = (int)(floor (min_p.z / leaf_width.z));
      max_b.z = (int)(floor (max_p.z / leaf_width.z));

      // Compute the number of divisions needed along all axis
      div_b.x = (int)(max_b.x - min_b.x + 1);
      div_b.y = (int)(max_b.y - min_b.y + 1);
      div_b.z = (int)(max_b.z - min_b.z + 1);

      // Allocate the space needed
      try
      {
        if (leaves.capacity () < div_b.x * div_b.y * div_b.z)
          leaves.reserve (div_b.x * div_b.y * div_b.z);             // fallback to x*y*z from 2*x*y*z due to memory problems
        leaves.resize (div_b.x * div_b.y * div_b.z);
      }
      catch (std::bad_alloc)
      {
        ROS_ERROR ("Failed while attempting to allocate a vector of %f (%g x %g x %g) leaf elements (%f bytes total)", div_b.x * div_b.y * div_b.z,
                   div_b.x, div_b.y, div_b.z, div_b.x * div_b.y * div_b.z * sizeof (ExtendedLeaf));
        return;
      }

      for (unsigned int cl = 0; cl < leaves.size (); cl++)
      {
        if (leaves[cl].indices.size () > 0)
        {
          leaves[cl].centroid_x = leaves[cl].centroid_y = leaves[cl].centroid_z = 0.0;
          leaves[cl].indices.resize (0);
        }
      }

      // First pass: go over all points and insert them into the right leaf
      for (unsigned int cp = 0; cp < cloud.points.size (); cp++)
      {
        int i = (int)(floor (cloud.points[cp].x / leaf_width_.x));
        int j = (int)(floor (cloud.points[cp].y / leaf_width_.y));
        int k = (int)(floor (cloud.points[cp].z / leaf_width_.z));

        int idx = ( (k - min_b.z) * div_b.y * div_b.x ) + ( (j - min_b.y) * div_b.x ) + (i - min_b.x);
        leaves[idx].centroid_x += cloud.points[cp].x;
        leaves[idx].centroid_y += cloud.points[cp].y;
        leaves[idx].centroid_z += cloud.points[cp].z;
        leaves[idx].indices.push_back (cp);
      }

      Point32 centroid;
      Eigen::Matrix3d covariance_matrix;
      // Second pass: go over all leaves and compute centroids and the point normals (nx, ny, nz), surface curvature estimates (c)
      int nr_p = 0;
      for (unsigned int cl = 0; cl < leaves.size (); cl++)
      {
        if (leaves[cl].indices.size () > 0)
        {
          cloud_down_.points[nr_p].x = leaves[cl].centroid_x / leaves[cl].indices.size ();
          cloud_down_.points[nr_p].y = leaves[cl].centroid_y / leaves[cl].indices.size ();
          cloud_down_.points[nr_p].z = leaves[cl].centroid_z / leaves[cl].indices.size ();

          // Compute the 3x3 covariance matrix
          cloud_geometry::nearest::computeCovarianceMatrix (cloud, leaves[cl].indices, covariance_matrix, centroid);

          // Extract the eigenvalues and eigenvectors
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ei_symm (covariance_matrix);
          Eigen::Vector3d eigen_values  = ei_symm.eigenvalues ();
          Eigen::Matrix3d eigen_vectors = ei_symm.eigenvectors ();

          // Normalize the surface normal (eigenvector corresponding to the smallest eigenvalue)
          // Note: Remember to take care of the eigen_vectors ordering
          double norm = sqrt ( eigen_vectors (0, 0) * eigen_vectors (0, 0) +
                               eigen_vectors (1, 0) * eigen_vectors (1, 0) +
                               eigen_vectors (2, 0) * eigen_vectors (2, 0));

          cloud_down.channels[0].values[nr_p] = eigen_vectors (0, 0) / norm;
          cloud_down.channels[1].values[nr_p] = eigen_vectors (1, 0) / norm;
          cloud_down.channels[2].values[nr_p] = eigen_vectors (2, 0) / norm;
          cloud_down.channels[3].values[nr_p] = eigen_values (1);
          cloud_down.channels[4].values[nr_p] = eigen_values (2);

          nr_p++;
        }
      }
      cloud_down.points.resize (nr_p);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Callback
    void cloud_cb ()
    {
      ROS_INFO ("Received %d data points in frame %s with %d channels (%s).", (int)cloud_.points.size (), cloud_.header.frame_id.c_str (),
                (int)cloud_.channels.size (), cloud_geometry::getAvailableChannels (cloud_).c_str ());
      if (cloud_.points.size () == 0)
      {
        ROS_ERROR ("No data points found. Exiting...");
        return;
      }

      ros::Time ts = ros::Time::now ();

      // ---[ Step 1: downsample the point cloud + compute the eigenvalues/eigenvectors of the centroids
      downsampleEstimateEigen (cloud_, cloud_down_, leaf_width_, leaves_);
      ROS_INFO ("Number of points after downsampling with a leaf of size [%f,%f,%f]: %d.", leaf_width_.x, leaf_width_.y, leaf_width_.z, (int)cloud_down_.points.size ());
      leaves_.resize (0);    // dealloc memory used for the downsampling process

      ROS_INFO ("Downsampling and eigen analysis done in %g seconds.\n", (ros::Time::now () - ts).toSec ());

      ts = ros::Time::now ();
      // ---[ Step 2: formulate the calibration as a non-linear minimization problem
      optimizeCalibrationTransformation (&cloud_down_);

      ROS_INFO ("Non-linear optimization done in %g seconds.\n", (ros::Time::now () - ts).toSec ());

      node_.publish ("~plane", cloud_plane_);
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);

  ros::Node ros_node ("laser_arm_calib");

  LaserArmCalib p (ros_node);
  ros_node.spin ();

  return (0);
}
/* ]--- */

