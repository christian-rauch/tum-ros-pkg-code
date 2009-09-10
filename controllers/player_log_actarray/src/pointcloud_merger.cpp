/*
 * Copyright (c) 2009 Dejan Pangercic <pangercic -=- cs.tum.edu>
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
 * $Id: pointcloud_merger.cpp 161 2009-09-06 15:18:34Z dejanpan $
 *
 */

/**
@mainpage

@htmlinclude manifest.html

\author Dejan Pangercic

@b pointcloud_merger stiches together partial scans/pointclouds into
whole object representation.  

 **/

// ROS core
#include <ros/ros.h>
// ROS messages
#include <sensor_msgs/PointCloud.h>
//read input pcds
#include  <tools/file_io.h>
//for savePCDFile ()
#include <point_cloud_mapping/cloud_io.h>
//cloud rotation
#include  <tools/transform.h>
#include <geometry_msgs/Point32.h>
#include <angles/angles.h>

#include <string>
using namespace std;
using namespace ros;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace cloud_io;
using namespace player_log_actarray;


#define ANGLE_SHIFT 180.0

class PointcloudMerger
{
protected:
  NodeHandle nh_;
  
  public:
    // ROS messages
  PointCloud cloud_in_, cloud_out_, cloud_interim_;
  string dir_;
  vector<string> file_list_;
  Point32 axis_, trans_;
  //rotation axis
  double x_, y_, z_;
  //translation
  double tx_, ty_, tz_;
  
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Constructor
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  PointcloudMerger()
    {
      nh_.param ("~dir", dir_, string(""));     //dir to fetch .pcd files from
      if(dir_ == "")
	ROS_ERROR("You must specify path to .pcd files");
      //get the list of .log files
      file_io::get_log_files(dir_, file_list_, ".pcd");
	
      nh_.param ("~axis_x", x_, 0.0);     // rotate around x
      nh_.param ("~axis_y", y_, 0.0);     // rotate around y
      nh_.param ("~axis_z", z_, 0.0);     // rotate around z
      axis_.x = float(x_), axis_.y = float(y_), axis_.z = float(z_); 

      nh_.param ("~tx", tx_, 0.0);     // translate x
      nh_.param ("~ty", ty_, 0.0);     // translate y
      nh_.param ("~tz", tz_, 0.0);     // translate z
      trans_.x = float(tx_), trans_.y = float(ty_), trans_.z = float(tz_); 
  }
  

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //gets an angle from a following string format: chopping-board_0000_.log.delimited.pcd
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  int get_angle (string file_name)
  {
    string angle;
    int angle_int;
    int found = file_name.find("_");
    if (found!=-1)
      {
      for (int i = found+1; i < found+5; i++)
	{
	  angle += file_name[i];
	  angle_int = atoi(angle.c_str()); 
	}
      }
    else
      ROS_ERROR("Delimiter _ not found!");
    return angle_int;
  }
    
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Destructor
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  virtual ~PointcloudMerger () { }

  bool
  spin ()
  {
    int angle = 0;
    ROS_INFO("Rot. axis: [%f %f %f]", axis_.x, axis_.y, axis_.z);
    while (nh_.ok ())
      {	
	for (unsigned int i = 0; i < file_list_.size(); i++)
	  {
	    ROS_INFO("Loading file %s", file_list_[i].c_str());
	    loadPCDFile(file_list_[i].c_str(), cloud_in_);
	    angle = get_angle(file_list_[i]) + ANGLE_SHIFT;
	    ROS_INFO("Rotating for %d deg.", angle);
	    transform::translatePointCloud (cloud_in_, cloud_interim_, trans_);
	    if(transform::rotatePointCloud (cloud_interim_, cloud_out_, angles::from_degrees(-angle), axis_))
	    {
	      if(cloud_out_.points.size() != 0)		
		{
		  int str_len = file_list_[i].length();
		  string pcd_del = file_list_[i].erase((str_len - 4), 4);
		  pcd_del += ".rotated.pcd";
		  ROS_INFO("Saving file %s with nr. of points %d", pcd_del.c_str(), cloud_out_.points.size());
		  savePCDFile (pcd_del.c_str(), cloud_out_, false);
		  pcd_del.clear();
		}
	    }
	    else
	      continue;
	  }
        ros::spinOnce ();
	break;
      }
    return (true);
  }
  
};

/* ---[ */
int
  main (int argc, char** argv)
{
  init (argc, argv, "pointcloud_merger");

  PointcloudMerger p;
  p.spin ();

  return (0);
}
/* ]--- */
