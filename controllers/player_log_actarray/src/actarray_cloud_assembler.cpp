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

  public:
    // ROS messages
    PlayerActarrayConstPtr cur_act_;
    PlayerActarray prev_act_;
    vector<LaserScanConstPtr> scans_;

    PointCloud cloud_;

    Subscriber actarray_sub_, laserscan_sub_;

    Publisher cloud_pub_;

    vector<DH> arm_params_;
    // Parameters
    Point translation_;
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
      nh_.param ("~translations_x", translation_.x, 0.0);
      nh_.param ("~translations_y", translation_.y, 0.0);
      nh_.param ("~translations_z", translation_.z, 0.0);

      string armDH;
      ROS_INFO ("Using the following DH parameters for the arm: ");
      if (left_arm_)
        nh_.param ("~larm_DH", armDH, string ());
      else
        nh_.param ("~rarm_DH", armDH, string ());
      getDHParameters (armDH, arm_params_);
      printDHParameters (arm_params_);

      actarray_sub_  = nh_.subscribe ("/player_actarray", 1, &ActarrayCloudAssembler::actarray_cb, this);
      laserscan_sub_ = nh_.subscribe ("/laser_scan", 1, &ActarrayCloudAssembler::scan_cb, this);

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

      ROS_INFO ("Using the following translation values: %f, %f, %f", translation_.x, translation_.y, translation_.z);
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
    Eigen::Matrix4f
      getJointTransformation (const DH& param, const double& q)
    {
      Eigen::Matrix4f T;
      double t = param.t, d = param.d;
      t += (param.type == 0) ? q : 0;
      d += (param.type == 0) ? 0 : q;
      double st = sin (t), ct = cos (t);
      double sa = sin (param.a), ca = cos (param.a);
      T(0, 0) = ct;  T(0, 1) = -st * ca;  T(0, 2) = +st * sa; T(0, 3) = param.l * ct;
      T(1, 0) = st;  T(1, 1) = +ct * ca;  T(1, 2) = -ct * sa; T(1, 3) = param.l * st;
      T(2, 0) = 0;   T(2, 1) = sa;        T(2, 2) = ca;       T(2, 3) = d;
      T(3, 0) = 0;   T(3, 1) = 0;         T(3, 2) = 0;        T(3, 3) = 1;
      return (T);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Obtain a global transormation matrix for an array of joints
    Eigen::Matrix4f
      getGlobalTransformation (const vector<DH>& params, const vector<double>& joints)
    {
      // Compute the transformation matrix as a product of relative transformations
      Eigen::Matrix4f global_transformation = getJointTransformation (params[0], joints[0]);
      for (unsigned int i = 1; i < joints.size (); i++)
      {
        Eigen::Matrix4f local_transformation = getJointTransformation (params[i], joints[i]);
        Eigen::Matrix4f result = global_transformation * local_transformation;
        global_transformation = result;
      }
      return (global_transformation);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Obtain a transformation matrix from a set of given DH parameters
    Eigen::Matrix4f
      getDHTransformation (const vector<DH>& arm_params, const vector<double>& q_values)
    {
      // Obtain the coordinates of the end-effector relative to shoulder
      Eigen::Matrix4f shoulder_transform = getGlobalTransformation (arm_params, q_values);
      // Obtain the coordinates of end-effector relative to robot base
      //shoulder_transform *= base2arm;
      return (shoulder_transform);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PlayerActarray message callback
    void
      actarray_cb (const PlayerActarrayConstPtr& actarray)
    {
      cur_act_ = actarray;
      ROS_DEBUG ("PlayerActarray message received with %d joint poses. Number of laser scans in queue = %d.", (int)cur_act_->joints.size (), (int)scans_.size ());

      if (prev_act_.joints.size () == 0)
      {
        prev_act_.header = cur_act_->header;
        prev_act_.joints = cur_act_->joints;
        scans_.clear ();                // We lose initial scans, but it's ok, as the unit is not moving yet
        return;
      }

      int laser_packet_scan_id = 1;
      vector<double> q_values (cur_act_->joints.size ());
      Point vp_old, vp;
      Eigen::Vector4f pt, pt_t;
      // For each buffered laser packet
      for (unsigned int lc = 0; lc < scans_.size (); lc++)
      {
        LaserScanConstPtr laser_packet = scans_.at (lc);

        // Convert to XYZ
        if ((laser_packet->header.stamp < prev_act_.header.stamp) || (laser_packet->header.stamp > cur_act_->header.stamp))
        {
          ROS_ERROR ("Wrong laser timestamp encountered - ignoring packet %d with time %f (%f -> %f)", laser_packet_scan_id, laser_packet->header.stamp.toSec (),
                     prev_act_.header.stamp.toSec (), cur_act_->header.stamp.toSec ());
          continue;
        }

        double t0 = (laser_packet->header.stamp - prev_act_.header.stamp).toSec ();
        double t1 = (cur_act_->header.stamp - prev_act_.header.stamp).toSec ();

        for (unsigned int q_idx = 0; q_idx < cur_act_->joints.size (); q_idx++)
          q_values[q_idx] = prev_act_.joints[q_idx] + t0 * (cur_act_->joints[q_idx] - prev_act_.joints[q_idx]) / t1;

        Eigen::Matrix4f robot_transform = getDHTransformation (arm_params_, q_values);
        // Calculate the viewpoint for the current interpolated joint angles
        vp_old = vp;
/*        cerr << "-----" << endl;
            for (int d = 0; d < q_values.size (); d++)
              cerr << q_values[d] << " ";
            cerr << endl;
        cerr << robot_transform << endl;*/
        ///cANN::transform (robot_transform, translations, vp);
        ///if (_ANNpointEqual(vp, vp_old))
          ///continue;

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
          if ((distance > max_distance_) || (distance < min_distance_))
            continue;
          if ((angle_x < angles::from_degrees (laser_min_angle_)) || (angle_x > angles::from_degrees (laser_max_angle_)))
            continue;

          // 2D
          pt(0) = translation_.x + 0.0;
          pt(1) = translation_.y + distance * cos (M_PI - angle_x);
          pt(2) = translation_.z + distance * sin (M_PI - angle_x);
          pt(3) = 1.0;

          // Transform the point
          pt_t = robot_transform * pt;
          if (isnan (pt_t(0)) || isnan (pt_t(1)) || isnan (pt_t(2)))
          {
            cerr << " ---------- " << endl;
            for (int d = 0; d < q_values.size (); d++)
              cerr << q_values[d] << " ";
            cerr << endl;
            cerr << robot_transform << endl;
            cerr << pt(0) << " " << pt(1) << " " << pt(2) << " " << pt_t(0) << " " << pt_t(1) << " " << pt_t(2) << endl;
          }
          cloud_.pts[nr_points].x = pt_t(0);
          cloud_.pts[nr_points].y = pt_t(1);
          cloud_.pts[nr_points].z = pt_t(2);

          // Save the rest of the values
          cloud_.chan[0].vals[nr_points] = laser_packet->intensities[i];
          cloud_.chan[1].vals[nr_points] = distance;
          cloud_.chan[2].vals[nr_points] = i;
          cloud_.chan[3].vals[nr_points] = laser_packet_scan_id++;
          cloud_.chan[4].vals[nr_points] = vp.x;
          cloud_.chan[5].vals[nr_points] = vp.y;
          cloud_.chan[6].vals[nr_points] = vp.z;
          nr_points++;

          angle_x += resolution;
        }

        if (nr_points == 0)
          continue;
        cloud_.pts.resize (nr_points);
        for (unsigned int d = 0; d < cloud_.chan.size (); d++)
          cloud_.chan[d].vals.resize (nr_points);

        cloud_.header.stamp = Time::now ();
        ROS_INFO ("Publishing a PointCloud message with %d points and %d channels.", (int)cloud_.pts.size (), (int)cloud_.chan.size ());
        cloud_pub_.publish (cloud_);
      }

      prev_act_.joints = cur_act_->joints;
      scans_.clear ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // LaserScan message callback
    void
      scan_cb (const LaserScanConstPtr& scan)
    {
      //ROS_INFO ("LaserScan message received with %d measurements.", (int)scan->ranges.size ());
      scans_.push_back (scan);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Spin (!)
    bool
      spin ()
    {
      return (true);
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  init (argc, argv, "actarray_cloud_assembler");

  ActarrayCloudAssembler p;
  ros::spin ();
//   p.spin ();

  return (0);
}
/* ]--- */
