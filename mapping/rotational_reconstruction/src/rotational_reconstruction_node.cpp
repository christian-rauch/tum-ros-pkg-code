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
 * $Id:$
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
#include <mapping_msgs/PolygonalMap.h>

// Sample Consensus
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/msac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <sac_model_rotational.h>

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

#include <perception_srvs/RotationalReconstruction.h>
#include <perception_srvs/ClustersVoxels.h>
#include <perception_msgs/VoxelList.h>

using namespace std;
using namespace ros;
using namespace std_msgs;
using namespace robot_msgs;
using namespace mapping_msgs;
using namespace perception_srvs;
using namespace perception_msgs;
using namespace sample_consensus;


void
  findRotationalObjects (PointCloud cloud, std::vector<std::vector<std::vector<bool> > > freevoxels, double threshold_, double probability_, int max_iterations_, PointCloud &cloud_synth, Point32 min, Point32 ndivs, Point32 leaf_width, mapping_msgs::PolygonalMap &pmap)
{
  int debug = 1;
  SACModelRotational *sac_model_ = new SACModelRotational (freevoxels, min, ndivs,leaf_width, pmap);
  sac_model_->setDataSet (&cloud);

  int iterations_ = 0;
  int n_best_inliers_count = -INT_MAX;
  double k = 1.0;

  std::vector<int> best_model;
  std::vector<int> best_inliers, inliers;
  std::vector<int> selection;

  int n_inliers_count = 0;
  std::vector<double> best_coeffs;
  // Iterate
  while (iterations_ < k)
  {
    // Get X samples which satisfy the model criteria
    sac_model_->getSamples (iterations_, selection);
    iterations_ += 1;
    if (iterations_ > max_iterations_)
    {
      if (debug > 0)
        std::cerr << "[RANSAC::computeModel] RANSAC reached the maximum number of trials." << std::endl;
      break;
    }

    if (selection.size () == 0) break;

    // Search for inliers in the point cloud for the current plane model M
    if (!(sac_model_->computeModelCoefficients (selection)))
      continue;

    sac_model_->selectWithinDistance (sac_model_->getModelCoefficients (), threshold_, inliers);
    n_inliers_count = inliers.size ();

    // Better match ?
    if (n_inliers_count > n_best_inliers_count)
    {
      n_best_inliers_count = n_inliers_count;
      best_inliers = inliers;
      //inliers.clear ();
      best_model = selection;
      best_coeffs = sac_model_->getModelCoefficients ();
      cerr << "BEST COEFFS: ";
      for (unsigned int i = 0; i < best_coeffs.size(); i++)
        cerr << best_coeffs[i] << " "; 
      cerr << endl;

      // Compute the k parameter (k=log(z)/log(1-w^n))
      double w = (double)((double)n_inliers_count / (double)sac_model_->getIndices ()->size ());
      double p_no_outliers = 1 - pow (w, (double)selection.size ());
      p_no_outliers = std::max (std::numeric_limits<double>::epsilon (), p_no_outliers);       // Avoid division by -Inf
      p_no_outliers = std::min (1 - std::numeric_limits<double>::epsilon (), p_no_outliers);   // Avoid division by 0.
      k = log (1 - probability_) / log (p_no_outliers);
    }

    if (debug > 1)
      std::cerr << "[RANSAC::computeModel] Trial " << iterations_ << " out of " << ceil (k) << ": " << n_inliers_count << " inliers (best is: " << n_best_inliers_count << " so far)." << std::endl;
  }

  if (best_model.size () != 0)
  {
    cerr << "before" <<endl;
    double score = sac_model_->computeScore (best_coeffs, getMinMaxK (cloud, best_coeffs, best_inliers) , best_inliers, cloud_synth, threshold_);
    cerr << "after" <<endl;
    if (debug > 0)
      std::cerr << "[RANSAC::computeModel] Model found: " << n_best_inliers_count << " inliers, score is: " << score << std::endl;
    sac_model_->setBestModel (best_model);
    sac_model_->setBestInliers (best_inliers);
    //      pmap.polygons.resize (1);
    //      pmap.polygons[0].points.push_back ();
    return;
  }
  else
    if (debug > 0)
      std::cerr << "[RANSAC::computeModel] Unable to find a solution!" << std::endl;
  return;
}

class RotationalReconstructionService
{
  protected:
    ros::NodeHandle nh_;

  public:
    // ROS messages
    PointCloudConstPtr cloud_in_;

    PointCloud cloud_down_;
    Point leaf_width_;
    PointCloud cloud_annotated_;
    Point32 axis_;
    string global_frame_;

    // Parameters
    double thresh_;
    int clusters_min_pts_;

    ServiceServer rotational_reconstruction_service_;
    Publisher pmap_pub_;
    Publisher cloud_pub_;
    Publisher cloud_synth_pub_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    RotationalReconstructionService ()
    {
      nh_.param ("~ransac_dist_threshold", thresh_, 0.01); // 1 cm
      nh_.param ("/global_frame_id", global_frame_, std::string("/base_link"));
      rotational_reconstruction_service_ = nh_.advertiseService ("rotational_reconstruction_service", &RotationalReconstructionService::rotational_reconstruction_service, this);
      pmap_pub_           = nh_.advertise<PolygonalMap>  ("rotational_polygonal_map", 1);
      cloud_pub_          = nh_.advertise<PointCloud>  ("voxel_list_cloud", 1);
      cloud_synth_pub_    = nh_.advertise<PointCloud>  ("rot_synth_cloud", 1);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   void
      updateParametersFromServer ()
    {
      nh_.getParam ("~ransac_dist_threshold", thresh_);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool
      rotational_reconstruction_service (RotationalReconstruction::Request &req, RotationalReconstruction::Response &resp)
    {
      ROS_INFO ("Service request initiated.");
      updateParametersFromServer ();

      ros::ServiceClient client = nh_.serviceClient<ClustersVoxels>("clusters_service");
      
      ClustersVoxels srv;
      srv.request.leaf_width.x = 0.05;
      srv.request.leaf_width.y = 0.05;
      srv.request.leaf_width.z = 0.05;
      if (client.call (srv))
      {
        ROS_INFO ("Service call successful.");
      }
      else
      {
        ROS_ERROR ("Failed to call service!");
        return (-1);
      }
      
      perception_msgs::VoxelList vl = srv.response.vlist;
      std::vector<robot_msgs::PointCloud> clusters = srv.response.clusters;
      std::vector<perception_msgs::Voxel> voxels = vl.voxels;

      Point32 min = vl.min;
      Point32 leafwidth = vl.leaf_width;
      Point32 size = vl.ndivs;

      // create a 3D array for faster lookups
       std::vector<std::vector<std::vector<bool> > > lookup_v;
       lookup_v.resize((int)size.x);
       for (int i = 0; i < (int)lookup_v.size (); i++)
       {
         lookup_v [i].resize((int)size.y);
         for (int j = 0; j < (int)lookup_v [i].size (); j++)
         {
           lookup_v [i][j].resize((int)size.z);
           for (int k = 0; k < (int)lookup_v [i][j].size (); k++)
             lookup_v [i][j][k] = false;
         }
       }
      ROS_INFO ("Created lookup grid for free space testing.");
      // fill 3D array with occupancy information from the voxellist
      Voxel xyz;
      PointCloud cloud;
      PointCloud cloud_synth;
      
      cloud_synth.header.frame_id = global_frame_; 
      cloud_synth.header.stamp = ros::Time::now ();
      cloud_synth.set_chan_size (1);
      cloud_synth.chan[0].name = "intensities";
      
      cloud.header.frame_id = global_frame_; 
      cloud.header.stamp = ros::Time::now ();
      cloud.set_pts_size (voxels.size());
      cloud.set_chan_size (1);
      cloud.chan[0].name = "intensities";
      cloud.chan[0].vals.resize(voxels.size ());
      for (int i = 0; i < (int)voxels.size (); i++)
      {
        xyz = voxels[i];
        
        cloud.pts[i].x = (((double)xyz.i) + 0.5) * leafwidth.x + min.x;
        cloud.pts[i].y = (((double)xyz.j) + 0.5) * leafwidth.y + min.y;
        cloud.pts[i].z = (((double)xyz.k) + 0.5) * leafwidth.z + min.z;
        cloud.chan[0].vals[i] = 1.0;

         
        ROS_INFO ("accessing lookup grid cell <%i, %i, %i>", xyz.i, xyz.j, xyz.k);
        lookup_v [xyz.i][xyz.j][xyz.k] = true;
      }
      
      ROS_INFO ("Done accessing lookup grid.");
      
      // create something publishable
      PolygonalMap pmap;
      pmap.header.frame_id = global_frame_;
      pmap.header.stamp = ros::Time::now ();
      pmap.set_chan_size (3);
      pmap.chan[0].name = "r";
      pmap.chan[1].name = "g";
      pmap.chan[2].name = "b";

      // init all ransac related classes
      std::vector<int> inliers;
      std::vector<double> coeff;

//       for (unsigned int i = 0; i < clusters.size(); i++)
      for (unsigned int i = 0; i < 1; i++)
      {
        ROS_INFO ("accessing cluster nr. %i", i);
        PointCloud cur_cloud = clusters.at(i);
        ROS_INFO ("finding rot. objects");
        findRotationalObjects (cur_cloud, lookup_v, thresh_, 0.99, 1000, cloud_synth, min, size, leafwidth, pmap);
//          for (int j = 0; j < cur_cloud.pts.size(); j++)
//            cloud.pts.push_back (cur_cloud.pts[j]);
//         model->setOccupancyLookup (lookup_v);
//         model->setDataSet (&cur_cloud);
//         if (sac->computeModel (0))
//         {
//           sac->computeCoefficients (coeff);
//           sac->refineCoefficients (coeff);
//           model->selectWithinDistance (coeff, thresh_, inliers);
//           pmap.header.frame_id = global_frame_;
//           pmap.polygons.resize (1);
//           pmap.polygons[0].points.resize(10); 
//         }
      }
      ROS_INFO ("Done accessing clusters.");
      
      pmap_pub_.publish (pmap);
      cloud_pub_.publish (cloud);
      cloud_synth_pub_.publish (cloud_synth);

      ROS_INFO ("Service request terminated.");
      return (true);
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "rotational_reconstruction_node");

  RotationalReconstructionService r;
  ros::spin ();

  return (0);
}
/* ]--- */

