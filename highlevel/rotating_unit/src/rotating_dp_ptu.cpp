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

@b rotating_dp_ptu controls a PTU D47 Directed Perception unit.

 **/

// ROS core
#include <ros/ros.h>
// ROS messages
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>
#include <Eigen/Core>

#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/thread/mutex.hpp>

#include <list>

using namespace std;
using namespace ros;
using namespace sensor_msgs;

class RotatingDPPTU
{
  protected:
    string tf_frame_;
    NodeHandle nh_;
    tf::TransformBroadcaster broadcaster_;
    tf::Stamped<tf::Transform> transform_;
    boost::mutex s_lock_, a_lock_;

    int total_laser_scans_;
  public:
    // ROS messages
    list<LaserScanConstPtr> scans_;
    double first_act_stamp_;

    PointCloud cloud_;

    Publisher cloud_pub_;

    // Parameters
    Eigen::Vector4d translation_;
    double min_distance_, max_distance_, laser_min_angle_, laser_max_angle_;
    bool left_arm_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    RotatingDPPTU () : tf_frame_ ("laser_tilt_mount_link"),
                                transform_ (btTransform (btQuaternion (0, 0, 0), btVector3 (0, 0, 0)), Time::now (), tf_frame_, tf_frame_),
                                total_laser_scans_ (0),
                                left_arm_ (true)
    {
      nh_.param ("~min_distance", min_distance_, .7);     // minimum distance range to be considered
      nh_.param ("~max_distance", max_distance_, 3.01);   // maximum distance range to be considered

      nh_.param ("~laser_min_angle", laser_min_angle_, DBL_MIN);    // minimum range angle to be considered
      nh_.param ("~laser_max_angle", laser_max_angle_, DBL_MAX);    // maximum range angle to be considered

      // To make the code general, no translations with respect to the rotation origin are given here as default
      nh_.param ("~translations_x", translation_ (0), 0.0);
      nh_.param ("~translations_y", translation_ (1), 0.0);
      nh_.param ("~translations_z", translation_ (2), 0.0);

      nh_.param ("~left", left_arm_, true);
      string armDH;
      ROS_INFO ("Using the following DH parameters for the arm: ");
      if (left_arm_)
        nh_.param ("~larm_DH", armDH, string ());
      else
        nh_.param ("~rarm_DH", armDH, string ());

      //laserscan_sub_ = nh_.subscribe ("/laser_scan", 1000, &RotatingDPPTU::scan_cb, this);

      cloud_pub_ = nh_.advertise<PointCloud> ("/tilt_laser_cloud", 1);

      cloud_.header.frame_id = "laser_tilt_mount_link";
      cloud_.channels.resize (7);
      cloud_.channels[0].name = "intensity";
      cloud_.channels[1].name = "distance";
      cloud_.channels[2].name = "sid";
      cloud_.channels[3].name = "pid";
      cloud_.channels[4].name = "vx";
      cloud_.channels[5].name = "vy";
      cloud_.channels[6].name = "vz";

      ROS_INFO ("Using the following translation values: %f, %f, %f", translation_ (0), translation_ (1), translation_ (2));
      first_act_stamp_ = -1.0;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual ~RotatingDPPTU () { }


};

/* ---[ */
int
  main (int argc, char** argv)
{
  init (argc, argv, "rotating_dp_ptu");

  RotatingDPPTU p;
  ros::spin ();
//  p.spin ();

  return (0);
}
/* ]--- */
