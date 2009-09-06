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
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <player_log_actarray/PlayerActarray.h>
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>
#include <Eigen/Core>

#include<iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/thread/mutex.hpp>
//for get_log_files()
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include "boost/filesystem.hpp"
//for savePCDFile ()
#include <point_cloud_mapping/cloud_io.h>
#include <list>

using namespace std;
using namespace ros;
using namespace sensor_msgs;
using namespace player_log_actarray;
using namespace cloud_io;
using namespace boost::filesystem; 

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

    int total_laser_scans_;
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
    //arguments only used if assemble pcds from multiple files
    int save_to_pcd_laser_, save_to_pcd_actarray_;
    vector<string> file_list_;
    string cur_file_, dir_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ActarrayCloudAssembler () : tf_frame_ ("laser_tilt_mount_link"),
                                transform_ (btTransform (btQuaternion (0, 0, 0), btVector3 (0, 0, 0)), Time::now (), tf_frame_, tf_frame_),
                                total_laser_scans_ (0),
                                left_arm_ (true)
    {
      nh_.param ("~dir", dir_, string(""));     //dir to fetch .log files from
      //get the list of .log files
      if(dir_ != "")
	{
	  get_log_files(dir_, file_list_);
	  save_to_pcd_laser_ = 0, save_to_pcd_actarray_ = 0;
	}
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

          // Interpolate joint values
          for (unsigned int q_idx = 0; q_idx < cur_act->joints.size (); q_idx++)
            q_values[q_idx] = prev_act->joints[q_idx] + t0 * (cur_act->joints[q_idx] - prev_act->joints[q_idx]) / t1;
          found = true;
          break;
        }

       prev_act = cur_act;
      }

      return (found);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PlayerActarray message callback
    void
      actarray_cb (const PlayerActarrayConstPtr &actarray)
    {
      a_lock_.lock (); 
      actarrays_.push_back (actarray); 
      a_lock_.unlock ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // LaserScan message callback
    void
      scan_cb (const LaserScanConstPtr &scan)
    {
      ++total_laser_scans_;
      s_lock_.lock ();
      scans_.push_back (scan);
      s_lock_.unlock ();
    }
  
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Get a list of log files
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void get_log_files ( const path & directory, vector <string> &file_list, string suffix=".log", bool recurse_into_subdirs = false )
    {
      if( exists( directory ) )
	{
	  directory_iterator end ;
	  for( directory_iterator iter(directory) ; iter != end ; ++iter )
	    if ( is_directory( *iter ) )
	      {
		ROS_WARN("Directory %s", iter->string().c_str());
		//if( recurse_into_subdirs ) get_log_files(*iter) ;
	      }
	    else 
	      {
		int len = iter->string().length();
		int npos =  iter->string().rfind(suffix);
		if((len - npos) == suffix.length())
		    file_list.push_back(iter->string());
	      }
	}
    }
    
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Update parameters from server
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  int
  update_parameters_from_server()
  {
    nh_.getParam("/save_pcd_laser", save_to_pcd_laser_);
    nh_.getParam("/save_pcd_actarray", save_to_pcd_actarray_);
    if (save_to_pcd_laser_ == 1 && save_to_pcd_actarray_ == 1)
      {
	nh_.setParam("/save_pcd_actarray", 2);
	nh_.setParam("/save_pcd_laser", 2);
	sleep(1);
	string pcd_file = cur_file_ + ".pcd";
	ROS_INFO("Saving to file: %s", pcd_file.c_str());
	//save PointCloud to pcd file
	savePCDFile (pcd_file.c_str(), cloud_, false);
	if (file_list_.size() == 0)
	  return -1;
	cur_file_ = file_list_.back(), file_list_.pop_back();
	ROS_INFO("Re-starting player_[laser|actarray]_log_to_msg: %s. Filelist size: %d", cur_file_.c_str(), file_list_.size());
	nh_.setParam("/log_filename", cur_file_);
	return 1;
      }
    return 0;
  }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  bool
  spin ()
  {
    int laser_packet_scan_id = 1, point_cloud_total = 0;
    vector<double> q_values;
    Eigen::Vector4d pt, pt_t, vp, vp_old;
    Eigen::Matrix4d robot_transform;
    int nr_points = 0;
    if(dir_ != "")
      {
	cur_file_ = file_list_.back(), file_list_.pop_back();
	nh_.setParam("/log_filename", cur_file_);
	ROS_INFO("Starting (first time) player_[laser|actarray]_log_to_msg: %s. Filelist size: %d", cur_file_.c_str(), file_list_.size());
      }
    while (nh_.ok ())
      {
	
	if(dir_ != "")
	  {
	    int update = update_parameters_from_server();
	    if(update == 1)
	      {
		laser_packet_scan_id = 1, point_cloud_total = 0;
		q_values.clear();
		for (int i = 0; i < 4; i++)
		  pt(i)=0, pt_t(i)=0, vp(i)=0, vp_old(0);
		for (int i = 0; i < 4; i++)
		  {
		    for (int j = 0; j < 4; j++)
		      robot_transform(i, j)=0;
		  }		 
		nr_points = 0;
		actarrays_.clear();
		scans_.clear();
		ROS_INFO("Assembler reseting all values!");
	      }
	//no more files to convert
	    if (update == -1)
	      break;
	  }

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
		++it;
		continue;
	      }
	    
	    // Obtain the transformation corresponding to the current joint values
	    getGlobalTransformation (arm_params_, q_values, robot_transform);
	    // Calculate the viewpoint for the current interpolated joint angles
	    vp_old = vp;
	    vp = robot_transform * translation_;
	    
	    if (vp (0) == vp_old (0) && vp (1) == vp_old (1) && vp (2) == vp_old (2))
	      {
		s_lock_.lock (); it = scans_.erase (it); s_lock_.unlock ();
		continue;
	      }
	    
	    // Calculate the horizontal angles and the cartesian coordinates
	    double angle_x    = laser_packet->angle_min;
	    double resolution = laser_packet->angle_increment;
	    
	    if(dir_ == "")
	      nr_points = 0;
	   	    
	    if(dir_ == "")
	      cloud_.points.resize (laser_packet->ranges.size ());
	    else
	      cloud_.points.resize (cloud_.points.size() + laser_packet->ranges.size ());
	    
	    if(dir_ == "")
	      for (unsigned int d = 0; d < cloud_.channels.size (); d++)
		cloud_.channels[d].values.resize (laser_packet->ranges.size ());
	    else
	      {
		for (unsigned int d = 0; d < cloud_.channels.size (); d++)
		  cloud_.channels[d].values.resize (cloud_.channels[d].values.size() + laser_packet->ranges.size ());
	      }
	    
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
		cloud_.points[nr_points].x = pt_t(0);
		cloud_.points[nr_points].y = pt_t(1);
		cloud_.points[nr_points].z = pt_t(2);
		
		// Save the rest of the values
		cloud_.channels[0].values[nr_points] = intensity;
		cloud_.channels[1].values[nr_points] = distance;
		cloud_.channels[2].values[nr_points] = i;
		cloud_.channels[3].values[nr_points] = laser_packet_scan_id++;
		cloud_.channels[4].values[nr_points] = vp (0);
		cloud_.channels[5].values[nr_points] = vp (1);
		cloud_.channels[6].values[nr_points] = vp (2);
		nr_points++;
		
		angle_x += resolution;
	      }
	   	    
	    if (nr_points == 0)
	      {
		s_lock_.lock (); it = scans_.erase (it); s_lock_.unlock ();
		continue;
	      }
	    cloud_.points.resize (nr_points);
	    for (unsigned int d = 0; d < cloud_.channels.size (); d++)
	      cloud_.channels[d].values.resize (nr_points);
	    
	    cloud_.header.stamp = Time::now ();
	    ROS_INFO ("Publishing a PointCloud message (%d) with %d points and %d channels on topic /%s  %d.", 
		      ++point_cloud_total, (int)cloud_.points.size (), (int)cloud_.channels.size (), tf_frame_.c_str (), scans_.size());
	    cloud_pub_.publish (cloud_);
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
  p.spin ();

  return (0);
}
/* ]--- */
