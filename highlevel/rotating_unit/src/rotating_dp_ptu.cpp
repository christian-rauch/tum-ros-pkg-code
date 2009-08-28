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
 * $Id: rotating_unit.cpp 90 2009-08-24 09:44:53Z veedee $
 *
 */

/**
@mainpage

@htmlinclude manifest.html

\author Radu Bogdan Rusu

@b rotating_dp_ptu controls a PTU D47 Directed Perception unit and a SICK LMS400 to acquire dense 3D point cloud data automatically.

 **/

// ROS core
#include <ros/ros.h>
// ROS messages
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point32.h>
#include <boost/algorithm/string.hpp>
#include <boost/thread/mutex.hpp>
#include <Eigen/Core>
#include <point_cloud_mapping/geometry/nearest.h>

#include <mapping_srvs/RotatePTU.h>
#include <mapping_srvs/TriggerSweep.h>

using namespace std;
using namespace ros;
using namespace sensor_msgs;
using namespace geometry_msgs;

class RotatingDPPTU
{
  protected:
    NodeHandle nh_;
    boost::mutex s_lock_, a_lock_;

    int total_laser_scans_;
  public:
    // ROS messages
    list<LaserScanConstPtr> scans_;
    double first_act_stamp_;

    PointCloud cloud_;

    Publisher cloud_pub_;

    ServiceClient ptu_serv_, scan_serv_;

    // Parameters
    double min_distance_, max_distance_, laser_min_angle_, laser_max_angle_;
    bool left_arm_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    RotatingDPPTU () : total_laser_scans_ (0),
                       left_arm_ (true)
    {
      nh_.param ("~min_distance", min_distance_, .7);     // minimum distance range to be considered
      nh_.param ("~max_distance", max_distance_, 3.01);   // maximum distance range to be considered

      //laserscan_sub_ = nh_.subscribe ("/laser_scan", 1000, &RotatingDPPTU::scan_cb, this);

      cloud_pub_ = nh_.advertise<PointCloud> ("/tilt_laser_cloud", 1);

      ptu_serv_  = nh_.serviceClient<mapping_srvs::RotatePTU>("get_angle_service");
      scan_serv_ = nh_.serviceClient<mapping_srvs::TriggerSweep>("trigger_sweep");
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual ~RotatingDPPTU () { }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Eigen::Matrix3f
      assembleRotationMatrixZ (float angle)
    {
      double cz = cos (angle), sz = sin (angle);
      Eigen::Matrix3f rot_mat;
      rot_mat << 
        cz, -sz, 0, 
        sz,  cz, 0,
        0 ,   0, 1;
      return (rot_mat);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Rotate a PointCloud relative to the angle of the turning table
    void
      rotateCloudRelative (float relative_angle, const PointCloud &cloud_in, PointCloud &cloud_out)
    {
      cloud_out.header   = cloud_in.header;
      cloud_out.channels = cloud_in.channels;
      cloud_out.points.resize (cloud_in.points.size ());

      // Demean the point cloud 
      Point32 centroid;
      cloud_geometry::nearest::computeCentroid (cloud_in, centroid); 
      
      // Rotate it around Z with the given angle
      Eigen::Matrix3f rot_mat = assembleRotationMatrixZ (relative_angle);
      
      for (unsigned int i = 0; i < cloud_in.points.size (); i++)
      {
        double cx = cloud_in.points[i].x - centroid.x; 
        double cy = cloud_in.points[i].y - centroid.y; 
        double cz = cloud_in.points[i].z - centroid.z; 

        cloud_out.points[i].x = rot_mat (0, 0) * cx + rot_mat (0, 1) * cy + rot_mat (0, 2) * cz;
        cloud_out.points[i].y = rot_mat (1, 0) * cx + rot_mat (1, 1) * cy + rot_mat (1, 2) * cz;
        cloud_out.points[i].x = rot_mat (2, 0) * cx + rot_mat (2, 1) * cy + rot_mat (2, 2) * cz;
      }

      // Fine alignment using LM
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool
      spin ()
    {
      float angle = -170.0, s_angle = -170.0;
      mapping_srvs::RotatePTU p_s;
      mapping_srvs::TriggerSweep s_s;
      ros::Duration tictoc (1, 0);

      PointCloud cloud_r;

      while (nh_.ok ())
      {
        // Send a request to the PTU to move
        p_s.request.angle = angle;
        ptu_serv_.call (p_s);

        ROS_INFO ("Setting angle to %f. Sleeping for %f seconds.", angle, tictoc.toSec ());
        tictoc.sleep ();

        // Trigger the LMS400 to sweep
        scan_serv_.call (s_s);

        // Rotate the point cloud and publish it
        rotateCloudRelative (angle - s_angle, s_s.response.cloud, cloud_r);
       
        if (cloud_r.points.size () > 0)
        {
          cloud_pub_.publish (cloud_r);
          ROS_INFO ("Publishing cloud with %d points and %d channels on topic %s.", (int)cloud_r.points.size (), (int)cloud_r.channels.size (), cloud_r.header.frame_id.c_str ());
        }
        // Increase angle and repeat
        angle += 30.0;
        if (angle >= 180.0)
          break;
        ros::spinOnce ();
      }

      ROS_INFO ("Scanning complete.");
      return (true);
    }

};

/* ---[ */
int
  main (int argc, char** argv)
{
  init (argc, argv, "rotating_dp_ptu");

  RotatingDPPTU p;
  p.spin ();

  return (0);
}
/* ]--- */
