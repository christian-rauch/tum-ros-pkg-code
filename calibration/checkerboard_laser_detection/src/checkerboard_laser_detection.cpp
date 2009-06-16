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
 * $Id: checkerboard_laser_detection.cpp 13239 2009-04-01 18:05:46Z veedee $
 *
 */

// ROS core
#include <ros/node.h>
// ROS messages
#include <robot_msgs/PointCloud.h>
#include <robot_msgs/Polygon3D.h>
#include <robot_msgs/PolygonalMap.h>

// Sample Consensus
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/msac.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>

// Cloud geometry
#include <point_cloud_mapping/geometry/areas.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/distances.h>
#include <point_cloud_mapping/geometry/nearest.h>

#include <point_cloud_mapping/cloud_io.h>
#include <point_cloud_mapping/kdtree/kdtree_ann.h>

#include <sys/time.h>
#include <angles/angles.h>

using namespace std;
using namespace robot_msgs;

class CheckerboardLaserDetection 
{
  protected:
    ros::Node& node_;
  public:

    // ROS messages
    PointCloud cloud_, cloud_annotated_;
    PolygonalMap pmap_;

    string src_normal_;

    // Parameters
    double sac_distance_threshold_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    CheckerboardLaserDetection (ros::Node& anode) : node_ (anode)
    {
      node_.param ("~p_sac_distance_threshold", sac_distance_threshold_, 0.025);     // 3 cm

      node_.subscribe ("cloud_pcd", cloud_, &CheckerboardLaserDetection::cloud_cb, this, 1);

      node_.advertise<PointCloud> ("cloud_annotated", 1);
      node_.advertise<PolygonalMap> ("pmap", 1);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Find a plane model in a point cloud with SAmple Consensus methods
      * \param points the point cloud message
      * \param indices a subset of point indices to use
      * \param inliers the resultant planar inliers
      * \param coeff the resultant plane coefficients
      * \param viewpoint_cloud a point to the pose where the data was acquired from (for normal flipping)
      * \param dist_thresh the maximum allowed distance threshold of an inlier to the model
      * \param min_pts the minimum number of points allowed as inliers for a plane model
      */
    bool
      fitSACPlane (PointCloud *points, vector<int> &indices, vector<int> &inliers, vector<double> &coeff,
                   double dist_thresh)
    {
      // Create and initialize the SAC model
      sample_consensus::SACModelPlane *model = new sample_consensus::SACModelPlane ();
      sample_consensus::SAC *sac             = new sample_consensus::MSAC (model, dist_thresh);
      sac->setMaxIterations (100);
      model->setDataSet (points, indices);

      // Search for the best plane
      if (sac->computeModel (0))
      {
        sac->computeCoefficients (coeff);     // Compute the model coefficients
        sac->refineCoefficients (coeff);      // Refine them using least-squares
        model->selectWithinDistance (coeff, dist_thresh, inliers);

        ROS_INFO ("Found a planar model supported by %d inliers: [%g, %g, %g, %g]", (int)inliers.size (),
                   coeff[0], coeff[1], coeff[2], coeff[3]);

        // Project the inliers onto the model
        //model->projectPointsInPlace (inliers, coeff);
      }
      else
      {
        ROS_ERROR ("Could not compute a plane model.");
        return (false);
      }

      delete sac;
      delete model;
      return (true);
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Decompose a region of space into clusters based on the euclidean distance between points, and the normal
      * angular deviation
      * \NOTE: assumes normalized point normals !
      * \param points pointer to the point cloud message
      * \param tolerance the spatial tolerance as a measure in the L2 Euclidean space
      * \param clusters the resultant clusters
      * \param nx_idx the index of the channel containing the X component of the normal
      * \param ny_idx the index of the channel containing the Y component of the normal
      * \param nz_idx the index of the channel containing the Z component of the normal
      * \param eps_angle the maximum allowed difference between normals in degrees for cluster/region growing
      * \param min_pts_per_cluster minimum number of points that a cluster may contain (default = 1)
      */
    void
      findClusters (PointCloud &points, double tolerance, vector<vector<int> > &clusters,
                    int nx_idx, int ny_idx, int nz_idx,
                    double eps_angle, unsigned int min_pts_per_cluster = 1)
    {
      // Create a tree for these points
      cloud_kdtree::KdTree* tree = new cloud_kdtree::KdTreeANN (points);

      int nr_points = points.pts.size ();
      // Create a bool vector of processed point indices, and initialize it to false
      vector<bool> processed;
      processed.resize (nr_points, false);

      vector<int> nn_indices;
      vector<float> nn_distances;
      // Process all points in the indices vector
      for (int i = 0; i < nr_points; i++)
      {
        if (processed[i])
          continue;

        vector<int> seed_queue;
        int sq_idx = 0;
        seed_queue.push_back (i);

        processed[i] = true;

        while (sq_idx < (int)seed_queue.size ())
        {
          // Search for sq_idx
          tree->radiusSearch (seed_queue.at (sq_idx), tolerance, nn_indices, nn_distances);

          for (unsigned int j = 1; j < nn_indices.size (); j++)       // nn_indices[0] should be sq_idx
          {
            if (processed.at (nn_indices[j]))                         // Has this point been processed before ?
              continue;

            processed[nn_indices[j]] = true;
            if (nx_idx != -1)                                         // Are point normals present ?
            {
              // [-1;1]
              double dot_p = points.chan[nx_idx].vals[i] * points.chan[nx_idx].vals[nn_indices[j]] +
                             points.chan[ny_idx].vals[i] * points.chan[ny_idx].vals[nn_indices[j]] +
                             points.chan[nz_idx].vals[i] * points.chan[nz_idx].vals[nn_indices[j]];
              if ( fabs (acos (dot_p)) < eps_angle )
              {
                processed[nn_indices[j]] = true;
                seed_queue.push_back (nn_indices[j]);
              }
            }
            // If normal information is not present, perform a simple Euclidean clustering
            else
            {
              processed[nn_indices[j]] = true;
              seed_queue.push_back (nn_indices[j]);
            }
          }

          sq_idx++;
        }

        // If this queue is satisfactory, add to the clusters
        if (seed_queue.size () >= min_pts_per_cluster)
        {
          vector<int> r (seed_queue.size ());
          for (unsigned int j = 0; j < seed_queue.size (); j++)
            r[j] = seed_queue[j];

          // Remove duplicates
          sort (r.begin (), r.end ());
          r.erase (unique (r.begin (), r.end ()), r.end ());

          clusters.push_back (r);
        }
      }

      // Destroy the tree
      delete tree;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Spin (!)
    bool spin ()
    {
//      cloud_io::loadPCDFile (src_normal_.c_str (), cloud_);
//      cloud_.header.frame_id = cloud_annotated_.header.frame_id = "base_link";

      while (node_.ok ())
      {
        usleep (1000000);

        cloud_cb ();
//        break;
      }

      return true;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Callback
    void cloud_cb ()
    {
      if (cloud_.pts.size () == 0)
        return;

      cloud_annotated_.header = cloud_.header;
      pmap_.header = cloud_.header;
      cloud_annotated_.pts.resize (cloud_.pts.size ());

      ROS_INFO ("Received %d data points.", (int)cloud_.pts.size ());
      
      int nx_idx = cloud_geometry::getChannelIndex (cloud_, "nx");      // get the channel index of the first normal component
      ROS_INFO ("Normal information found at index: %u", nx_idx);
      
      vector<vector<int> > clusters;
      double euclidean_cluster_angle_tolerance_ = angles::from_degrees (5);
      findClusters (cloud_, 0.05, clusters, nx_idx + 0, nx_idx + 1, nx_idx + 2, euclidean_cluster_angle_tolerance_, 100);
      
      ROS_INFO ("Number of clusters found: %u", clusters.size ());
      // Check every cluster
      vector<int> inliers;
      vector<double> coeff;

      pmap_.polygons.resize (clusters.size ());
      
      for (unsigned int i = 0; i < clusters.size (); i++)
      {
        fitSACPlane (&cloud_, clusters[i], inliers, coeff, 0.03);    // Fit the plane model
        cloud_geometry::areas::convexHull2D (cloud_, inliers, coeff, pmap_.polygons[i]);
      }
      
//      cloud_annotated_.pts.resize (nr_p);
//      node_.publish ("cloud_annotated", cloud_annotated_);
      node_.publish ("pmap", pmap_);
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);

  ros::Node ros_node ("convex_patch_histogram_node");
  CheckerboardLaserDetection p (ros_node);
//  p.src_normal_   = string (argv[1]); 
  ros_node.spin ();

  return (0);
}
/* ]--- */

