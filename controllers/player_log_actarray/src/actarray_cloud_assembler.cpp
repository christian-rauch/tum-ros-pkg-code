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

/**
@mainpage

@htmlinclude manifest.html

\author Radu Bogdan Rusu

@b actarray_cloud_assembler makes use of LaserScan and PlayerActarray ROS messages to assemble a 3D point cloud dataset.

 **/

// ROS core
#include <ros/ros.h>
// ROS messages
#include <robot_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <player_log_actarray/PlayerActarray.h>
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
using namespace robot_msgs;
using namespace player_log_actarray;

struct DH
{
  int type;
  double t, d, l, a; // theta, d, a, alpha => t, d, l, a
};

class ActarrayCloudAssembler
{
  protected:
    string tf_frame_;
    NodeHandle nh_;
    tf::TransformBroadcaster broadcaster_;
    tf::Stamped<tf::Transform> transform_;
    boost::mutex s_lock_, a_lock_;

  public:
    // ROS messages
    list<LaserScanConstPtr> scans_;
    vector<PlayerActarrayConstPtr> actarrays_;
    double first_act_stamp_;

    PointCloud cloud_;

    Subscriber actarray_sub_, laserscan_sub_;

    Publisher cloud_pub_;

    vector<DH> arm_params_;
    // Parameters
    Eigen::Vector4d translation_;
    double min_distance_, max_distance_, laser_min_angle_, laser_max_angle_;
    bool left_arm_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ActarrayCloudAssembler () : tf_frame_ ("laser_tilt_mount_link"),
                                transform_ (btTransform (btQuaternion (0, 0, 0), btVector3 (0, 0, 0)), Time::now (), tf_frame_, tf_frame_),
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
      getDHParameters (armDH, arm_params_);
      printDHParameters (arm_params_);

      actarray_sub_  = nh_.subscribe ("/player_actarray", 1000, &ActarrayCloudAssembler::actarray_cb, this);
      laserscan_sub_ = nh_.subscribe ("/laser_scan", 1000, &ActarrayCloudAssembler::scan_cb, this);

      cloud_pub_ = nh_.advertise<PointCloud> ("/tilt_laser_cloud", 1);

      cloud_.header.frame_id = "laser_tilt_mount_link";
      cloud_.chan.resize (7);
      cloud_.chan[0].name = "intensity";
      cloud_.chan[1].name = "distance";
      cloud_.chan[2].name = "sid";
      cloud_.chan[3].name = "pid";
      cloud_.chan[4].name = "vx";
      cloud_.chan[5].name = "vy";
      cloud_.chan[6].name = "vz";

      ROS_INFO ("Using the following translation values: %f, %f, %f", translation_ (0), translation_ (1), translation_ (2));
      first_act_stamp_ = -1.0;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual ~ActarrayCloudAssembler () { }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Parse a string representing the DH parameters for an actarray and convert to our internal DH structure
    void
      getDHParameters (string armDH, vector<DH> &arm_params)
    {
      // Split the line into tokens
      vector<string> st;
      boost::split (st, armDH, boost::is_any_of (" "));

      arm_params.resize ((int)(st.size () / 4));
      int j = 0;
      for (unsigned int i = 0; i < arm_params.size (); i++)
      {
        arm_params[i].type = 0;
        arm_params[i].t = atof (st.at (j++).c_str ());
        arm_params[i].d = atof (st.at (j++).c_str ());
        arm_params[i].l = atof (st.at (j++).c_str ());
        arm_params[i].a = atof (st.at (j++).c_str ());
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Print a vector of DH parameters to screen
    void
      printDHParameters (const vector<DH> &params)
    {
      for (unsigned int i = 0; i < params.size (); i++)
        fprintf (stderr, "t: %4.4f     d: %4.4f     l: %4.4f     a: %4.4f\n", params[i].t, params[i].d, params[i].l, params[i].a);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Obtain a transformaion matrix for a joint defined by its DH parameters and joint values
    inline void
      getJointTransformation (const DH &param, const double &q, Eigen::Matrix4d &T)
    {
      double t = param.t, d = param.d;
      t += (param.type == 0) ? q : 0;
      d += (param.type == 0) ? 0 : q;
      double st = sin (t), ct = cos (t);
      double sa = sin (param.a), ca = cos (param.a);
      T <<
            ct, -st * ca, +st * sa, param.l * ct,
            st, +ct * ca, -ct * sa, param.l * st,
            0,        sa,       ca,            d,
            0,         0,        0,            1;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Obtain a global transormation matrix for an array of joints
    inline void
      getGlobalTransformation (const vector<DH> &params, const vector<double> &joints, Eigen::Matrix4d &global_transformation)
    {
      // Compute the transformation matrix as a product of relative transformations
      Eigen::Matrix4d local_transformation;
      getJointTransformation (params.at (0), joints.at (0), global_transformation);
      for (unsigned int i = 1; i < joints.size (); i++)
      {
        getJointTransformation (params[i], joints[i], local_transformation);
        global_transformation *= local_transformation;
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool
      interpolateActarrayValues (const LaserScanConstPtr &laser_packet, vector<PlayerActarrayConstPtr> &actarrays,
                                 vector<double> &q_values)
    {
      PlayerActarrayConstPtr prev_act = actarrays.at (0);
      if (laser_packet->header.stamp < prev_act->header.stamp)
        return (false);

      bool found = false;
      for (unsigned int la = 1; la < actarrays.size (); la++)
      {
        PlayerActarrayConstPtr cur_act = actarrays.at (la);

        // Check if the laser entry is between these two actarray entries
        if ((prev_act->header.stamp.toSec () <= laser_packet->header.stamp.toSec ()) && (laser_packet->header.stamp.toSec () <= cur_act->header.stamp.toSec ()))
        {
          double t0 = (laser_packet->header.stamp - prev_act->header.stamp).toSec ();
          double t1 = (cur_act->header.stamp - prev_act->header.stamp).toSec ();

//           ROS_INFO ("Using %f and %f to interpolate %f", prev_act->header.stamp.toSec (), cur_act->header.stamp.toSec (), laser_packet->header.stamp.toSec ());
          // Interpolate joint values
          for (unsigned int q_idx = 0; q_idx < cur_act->joints.size (); q_idx++)
            q_values[q_idx] = prev_act->joints[q_idx] + t0 * (cur_act->joints[q_idx] - prev_act->joints[q_idx]) / t1;
//           ROS_INFO ("%f %f %f -> %f", prev_act->joints[4], cur_act->joints[4], q_values[4], laser_packet->header.stamp.toSec ());

          found = true;
          break;
        }

/*        ROS_ERROR ("Wrong laser timestamp encountered - ignoring packet with time (%f -> %f-> %f)",
                    prev_act->header.stamp.toSec (), laser_packet->header.stamp.toSec (), cur_act->header.stamp.toSec ());*/
        prev_act = cur_act;
      }

      return (found);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PlayerActarray message callback
    void
      actarray_cb (const PlayerActarrayConstPtr &actarray)
    {
      ///ROS_INFO ("PlayerActarray (%f) message received with %d joint poses (%d). Number of laser scans in queue = %d.", actarray->header.stamp.toSec (), (int)actarray->joints.size (), (int)actarrays_.size (), (int)scans_.size ());
/*      if (first_act_stamp_ == -1.0)
        first_act_stamp_ = actarray->header.stamp.toSec ();
      // Remove all laserscans with a timestamp smaller than the first actarray packet
      for (list<LaserScanConstPtr>::iterator it = scans_.begin (); it != scans_.end (); ++it)
      {
        LaserScanConstPtr laser_packet = *it;
        // Check timestamps
        if (laser_packet->header.stamp.toSec () < first_act_stamp_)
        {
//           ROS_WARN ("Removing LaserScan with timestamp %f (minimum actarray is %f -> %f)", laser_packet->header.stamp.toSec (), first_act_stamp_, laser_packet->header.stamp.toSec () - first_act_stamp_);
//           s_lock_.lock (); it = scans_.erase (it); s_lock_.unlock ();
        }
      }*/
      a_lock_.lock (); actarrays_.push_back (actarray); a_lock_.unlock ();

//       // Need at least 2 values to interpolate
//       if (actarrays_.size () < 2)
//         return;

/*      bool found = false;
      double good_stamp;
      a_lock_.lock ();
      if (scans_.size () > 0)
      {
//         ROS_WARN ("Actarray buffer size (before): %d", (int)actarrays_.size ());
        for (vector<PlayerActarrayConstPtr>::reverse_iterator rit = actarrays_.rbegin (); rit != actarrays_.rend (); )
        {
          PlayerActarrayConstPtr cur_act = *rit;
          if (cur_act->header.stamp.toSec () < scans_.front ()->header.stamp.toSec ())
          {
            if (found)
            {
              vector<PlayerActarrayConstPtr>::iterator it = (++rit).base ();
//               ROS_ERROR ("Removing %f because it's way smaller than %f (good stamp is %f)", ((PlayerActarrayConstPtr)*it)->header.stamp.toSec (), scans_.front ()->header.stamp.toSec (), good_stamp);
              actarrays_.erase (it);
            }
            else
            {
              ++rit;
              found = true;
              good_stamp = cur_act->header.stamp.toSec ();
            }
          }
          else
          {
//             ROS_INFO ("%f found to be larger", cur_act->header.stamp.toSec ());
            ++rit;
          }
        }
//         ROS_WARN ("Actarray buffer size (after): %d", (int)actarrays_.size ());
      }
      a_lock_.unlock ();*/
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // LaserScan message callback
    void
      scan_cb (const LaserScanConstPtr &scan)
    {
      ///ROS_INFO ("LaserScan (%f) message received with %d measurements. Current queue size is %d.", scan->header.stamp.toSec (), (int)scan->ranges.size (), (int)scans_.size ());
      s_lock_.lock ();
      scans_.push_back (scan);
      s_lock_.unlock ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool
      spin ()
    {
      int laser_packet_scan_id = 1;
      vector<double> q_values;
      Eigen::Vector4d pt, pt_t, vp, vp_old;
      Eigen::Matrix4d robot_transform;

      // Infinite loop
      while (1)
      {
        // We need at least 2 actarray values and 1 laser scan to begin
        if (actarrays_.size () < 2 || scans_.size () == 0)
        {
          usleep (500);
          ros::spinOnce ();
          continue;
        }

        q_values.resize (actarrays_.at (0)->joints.size ());
        // For each buffered laser packet
        for (list<LaserScanConstPtr>::iterator it = scans_.begin (); it != scans_.end ();)
        {
          LaserScanConstPtr laser_packet = *it;

          // Interpolate actarray values
          if (!interpolateActarrayValues (laser_packet, actarrays_, q_values))
          {
            //ROS_ERROR ("Could not interpolate LaserScan with timestamp %f (minimum actarray is %f)", laser_packet->header.stamp.toSec (), first_act_stamp_);
            //for (int d = 0; d < actarrays_.size (); d++)
            //  ROS_WARN ("%f", actarrays_[d]->header.stamp.toSec ());
            ++it;
            continue;
  //           m_lock_.lock (); scans_.pop_front (); m_lock_.unlock ();
  //           break;
          }

          // Obtain the transformation corresponding to the current joint values
          getGlobalTransformation (arm_params_, q_values, robot_transform);
          // Calculate the viewpoint for the current interpolated joint angles
          vp_old = vp;
          vp = robot_transform * translation_;

          if (vp (0) == vp_old (0) && vp (1) == vp_old (1) && vp (2) == vp_old (2))
          {
            ROS_WARN ("same viewpoint");
            s_lock_.lock (); it = scans_.erase (it); s_lock_.unlock ();
  //           ++it;
            continue;
          }

          // Calculate the horizontal angles and the cartesian coordinates
          double angle_x    = laser_packet->angle_min;
          double resolution = laser_packet->angle_increment;

          int nr_points = 0;
          cloud_.pts.resize (laser_packet->ranges.size ());
          for (unsigned int d = 0; d < cloud_.chan.size (); d++)
            cloud_.chan[d].vals.resize (laser_packet->ranges.size ());

          for (unsigned int i = 0; i < laser_packet->ranges.size (); i++)
          {
            double distance = laser_packet->ranges[i];
            double intensity = laser_packet->intensities[i];
            if ((distance > max_distance_) || (distance < min_distance_))
            {
              angle_x += resolution;
              continue;
            }
            if ((angle_x < angles::from_degrees (laser_min_angle_)) || (angle_x > angles::from_degrees (laser_max_angle_)))
            {
              angle_x += resolution;
              continue;
            }

            // 2D
            pt(0) = translation_ (0) + 0.0;
            pt(1) = translation_ (1) + distance * cos (M_PI - angle_x);
            pt(2) = translation_ (2) + distance * sin (M_PI - angle_x);
            pt(3) = 1.0;

            // Transform the point
            pt_t = robot_transform * pt;
            cloud_.pts[nr_points].x = pt_t(0);
            cloud_.pts[nr_points].y = pt_t(1);
            cloud_.pts[nr_points].z = pt_t(2);

            // Save the rest of the values
            cloud_.chan[0].vals[nr_points] = intensity;
            cloud_.chan[1].vals[nr_points] = distance;
            cloud_.chan[2].vals[nr_points] = i;
            cloud_.chan[3].vals[nr_points] = laser_packet_scan_id++;
            cloud_.chan[4].vals[nr_points] = vp (0);
            cloud_.chan[5].vals[nr_points] = vp (1);
            cloud_.chan[6].vals[nr_points] = vp (2);
            nr_points++;

            angle_x += resolution;
          }

          if (nr_points == 0)
          {
            cerr << "nr_points" << endl;
            s_lock_.lock (); it = scans_.erase (it); s_lock_.unlock ();
            continue;
          }
          cloud_.pts.resize (nr_points);
          for (unsigned int d = 0; d < cloud_.chan.size (); d++)
            cloud_.chan[d].vals.resize (nr_points);

          cloud_.header.stamp = Time::now ();
          //ROS_INFO ("Publishing a PointCloud message with %d points and %d channels.", (int)cloud_.pts.size (), (int)cloud_.chan.size ());
          cloud_pub_.publish (cloud_);

//         ROS_ERROR ("Erasing %f", ((LaserScanConstPtr)*it)->header.stamp.toSec ());
          s_lock_.lock (); it = scans_.erase (it); s_lock_.unlock ();
        }
        ros::spinOnce ();
      }
      return (true);
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  init (argc, argv, "actarray_cloud_assembler");

  ActarrayCloudAssembler p;
//  ros::spin ();
  p.spin ();

  return (0);
}
/* ]--- */
