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

#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/thread/mutex.hpp>

#include <mapping_srvs/Ptu.h>

using namespace std;
using namespace ros;
using namespace sensor_msgs;

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

    ServiceClient ptu_serv_;

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

      ptu_serv_ = nh_.serviceClient<mapping_srvs::Ptu>("get_angle_service");
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual ~RotatingDPPTU () { }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool
      spin ()
    {
      float angle = -100.0;
      mapping_srvs::Ptu srv;
      ros::Duration tictoc (1, 0);

      while (nh_.ok ())
      {
        // Send a request to the PTU to move
        srv.request.angle = angle;
        ptu_serv_.call (srv);

        ROS_INFO ("Setting angle to %f. Sleeping for %f seconds.", angle, tictoc.toSec ());
        tictoc.sleep ();

        // Trigger the LMS400 to sweep
         

        // Rotate the point cloud and publish it
        
        
       
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
