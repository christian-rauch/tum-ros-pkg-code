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
// #include <sac_model_rotational.h>

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
using namespace perception_srvs;
using namespace perception_msgs;
using namespace sample_consensus;

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

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    RotationalReconstructionService ()
    {
      nh_.param ("~ransac_dist_threshold", thresh_, 0.01); // 1 cm
      nh_.param ("/global_frame_id", global_frame_, std::string("/base_link"));
      rotational_reconstruction_service_ = nh_.advertiseService ("rotational_reconstruction_service", &RotationalReconstructionService::rotational_reconstruction_service, this);
      pmap_pub_           = nh_.advertise<PolygonalMap>  ("rotational_polygonal_map", 1);
      cloud_pub_           = nh_.advertise<PointCloud>  ("voxel_list_cloud", 1);
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
      
      // fill 3D array with occupancy information from the voxellist
      Voxel xyz;
      PointCloud cloud;
      cloud.set_pts_size (voxels.size());
      for (int i = 0; i < (int)voxels.size (); i++)
      {
        xyz = voxels[i];
        
      Point32 min = vl.min;
      Point32 leafwidth = vl.leaf_width;
      Point32 size = vl.ndivs;
        cloud.pts[i].x = ((double)xyz.i + 0.5) * leafwidth.x + min.x;
        cloud.pts[i].y = ((double)xyz.j + 0.5) * leafwidth.y + min.y;
        cloud.pts[i].z = ((double)xyz.k + 0.5) * leafwidth.z + min.z;

        lookup_v [xyz.i][xyz.j][xyz.k] = true;
      }
      
      // create something publishable
      PolygonalMap pmap;

      // init all ransac related classes
      std::vector<int> inliers;
      std::vector<double> coeff;
      
//       SACModel *model = new SACModelRotational ();
//       MSAC *sac = new MSAC (model, thresh_);
//       
//       sac->setMaxIterations (500);
//       sac->setProbability (0.99);

      for (unsigned int i = 0; i < clusters.size(); i++)
      {
//         PointCloud cur_cloud = clusters.at(i);
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
      
      pmap_pub_.publish (pmap);
      cloud_pub_.publish (cloud);

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

