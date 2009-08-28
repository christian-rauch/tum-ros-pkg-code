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
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <mapping_msgs/PolygonalMap.h>

// Sample Consensus
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/msac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>
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
using namespace sensor_msgs;
using namespace mapping_msgs;
using namespace perception_srvs;
using namespace perception_msgs;
using namespace sample_consensus;
using namespace geometry_msgs;

void
  findRotationalObjects (PointCloud cloud, std::vector<std::vector<std::vector<bool> > > freevoxels, double threshold_, double probability_, int max_iterations_, PointCloud &cloud_synth, Point32 min, Point32 ndivs, Point32 leaf_width, mapping_msgs::PolygonalMap &pmap)
{
  int debug = 2;
  SACModelRotational *sac_model_ = new SACModelRotational (freevoxels, min, ndivs,leaf_width, pmap);
  SACModelPlane *sac_model_planes = new SACModelPlane ();
  sac_model_->setDataSet (&cloud);
  sac_model_planes->setDataSet (&cloud);

  int iterations_ = 0;
  int n_best_inliers_count = -1;
  double k = INT_MAX;

  std::vector<int> best_model;
  std::vector<int> best_inliers, inliers;
  std::vector<int> selection;
  std::vector<int> inliers_planes;
  std::vector<int> selection_planes;
  
  int best_model_type = -1;

  int n_inliers_count = 0;
  std::vector<double> best_coeffs;
  // Iterate
  while (iterations_ < k)
  {
    iterations_ += 1;
    if (iterations_ > max_iterations_)
    {
      if (debug > 0)
        std::cerr << "[RANSAC::computeModel] RANSAC reached the maximum number of trials." << std::endl;
      break;
    }
    if (debug > 1)
      std::cerr << "[RANSAC::computeModel] Trial " << iterations_ << " out of " << ceil (k) << ": " << n_inliers_count << " inliers (best is: " << n_best_inliers_count << " so far)." << std::endl;
    
    // Get X samples which satisfy the model criteria
    sac_model_->getSamples (iterations_, selection);
    // Get X samples which satisfy the model criteria
    sac_model_planes->getSamples (iterations_, selection_planes);

    if (selection.size () == 0 && selection_planes.size() == 0) break;

    // Search for inliers in the point cloud for the current plane model M
    bool success = sac_model_->computeModelCoefficients (selection);
    // Search for inliers in the point cloud for the current plane model M
    bool success_planes = sac_model_planes->computeModelCoefficients (selection_planes);

    sac_model_->selectWithinDistance (sac_model_->getModelCoefficients (), threshold_, inliers);
    sac_model_planes->selectWithinDistance (sac_model_planes->getModelCoefficients (), threshold_, inliers_planes);
    if (success_planes)
    {
      if (inliers_planes.size() < 3)
        ROS_ERROR ("this shoudlgnbt hapenre goddamoit");
     
      std::vector<double> coeffs = sac_model_planes->getModelCoefficients ();
      int after, before = inliers_planes.size ();
      do
      {
        std::vector<double> backup_coeff (coeffs);
        std::vector<int> backup_inliers (inliers_planes);
        before = inliers_planes.size ();
        ROS_WARN ("before refit: %i inliers", inliers_planes.size());
        sac_model_planes->refitModel  (inliers_planes, coeffs);
        sac_model_planes->selectWithinDistance (coeffs, threshold_, inliers_planes);
//         sac_model_->refitModelNoAxis (inliers, coeffs);
//         sac_model_->selectWithinDistance (coeffs, threshold_, inliers);
        ROS_WARN ("after refit: %i inliers", inliers_planes.size());
        after = inliers_planes.size ();
        if (after > before)
        {
          backup_inliers = inliers_planes;
          backup_coeff = coeffs;
        }
        else
        {
          coeffs = backup_coeff;
          inliers_planes = backup_inliers;
          break;
        }
      } while (true);

      n_inliers_count = ((double)inliers_planes.size ());

      // Better match ?
      if (n_inliers_count > n_best_inliers_count)
      {
        n_best_inliers_count = n_inliers_count;
        best_inliers = inliers_planes;
        //inliers.clear ();
        best_model = selection_planes;
        best_coeffs = coeffs;
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
        best_model_type = 0; // plane
      }
    }
    if (success)
    {
      if (inliers.size() < 4)
        ROS_ERROR ("this shoudlgnbt hapenre goddamoit");
     
      std::vector<double> coeffs = sac_model_->getModelCoefficients ();
      int after, before = inliers.size ();
      do
      {
        std::vector<double> backup_coeff (coeffs);
        std::vector<int> backup_inliers (inliers);
        before = inliers.size ();
        ROS_WARN ("before refit: %i inliers", inliers.size());
          if (sac_model_->refitAxis  (inliers, coeffs))
             sac_model_->selectWithinDistance (coeffs, threshold_, inliers);
//         sac_model_->refitModelNoAxis (inliers, coeffs);
//         sac_model_->selectWithinDistance (coeffs, threshold_, inliers);
        ROS_WARN ("after refit: %i inliers", inliers.size());
        after = inliers.size ();
        if (after > before)
        {
          backup_inliers = inliers;
          backup_coeff = coeffs;
        }
        else
        {
          coeffs = backup_coeff;
          inliers = backup_inliers;
          break;
        }
      } while (true);

      PointCloud temp;
      temp.channels.resize (1);
      double score = sac_model_->computeScore (coeffs, getMinMaxK (cloud, coeffs, inliers) , inliers, temp, threshold_);
      n_inliers_count = ((double)inliers.size ()) * (1.0+score);

      // Better match ?
      if (n_inliers_count > n_best_inliers_count)
      {
        n_best_inliers_count = n_inliers_count;
        best_inliers = inliers;
        //inliers.clear ();
        best_model = selection;
        best_coeffs = coeffs;
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
        best_model_type = 1; // plane
      }
    }
  }

  if (best_model.size () != 0)
  {
    if (best_model_type == 0)
    {
      Polygon p;
      cloud_geometry::areas::convexHull2D (cloud, inliers_planes, best_coeffs, p);
      pmap.polygons.push_back (p); 
    }
    else
    {
      double score = sac_model_->computeScore (best_coeffs, getMinMaxK (cloud, best_coeffs, best_inliers) , best_inliers, cloud_synth, threshold_);

    }

    cerr << "before" <<endl;
    if (debug > 0)
      std::cerr << "[RANSAC::computeModel] Model found: " << n_best_inliers_count << std::endl;
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
      nh_.param ("~ransac_dist_threshold", thresh_, 0.005); // 1 cm
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
      std::vector<PointCloud> clusters = srv.response.clusters;
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
      cloud_synth.channels.resize (1);
      cloud_synth.channels[0].name = "intensities";
      
      cloud.header.frame_id = global_frame_; 
      cloud.header.stamp = ros::Time::now ();
      cloud.points.resize (voxels.size());
      cloud.channels.resize (1);
      cloud.channels[0].name = "intensities";
      cloud.channels[0].values.resize(voxels.size ());
      for (int i = 0; i < (int)voxels.size (); i++)
      {
        xyz = voxels[i];
        
        cloud.points[i].x = (((double)xyz.i) + 0.5) * leafwidth.x + min.x;
        cloud.points[i].y = (((double)xyz.j) + 0.5) * leafwidth.y + min.y;
        cloud.points[i].z = (((double)xyz.k) + 0.5) * leafwidth.z + min.z;
        cloud.channels[0].values[i] = 1.0;

         
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

      for (unsigned int i = 0; i < clusters.size(); i++)
//       for (unsigned int i = 0; i < 1; i++)
      {
        ROS_INFO ("accessing cluster nr. %i", i);
        PointCloud cur_cloud = clusters.at(i);
        ROS_INFO ("finding rot. objects");
        findRotationalObjects (cur_cloud, lookup_v, thresh_, 0.99, 100, cloud_synth, min, size, leafwidth, pmap);
//          for (int j = 0; j < cur_cloud.pts.size(); j++)
//            cloud.pts.push_back (cur_cloud.points[j]);
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

