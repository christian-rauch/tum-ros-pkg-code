/*
 * Copyright (c) 2008 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
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
 * $Id: pcd_to_msg.cpp 21050 2009-08-07 21:24:30Z jfaustwg $
 *
 */

/**
@mainpage

@htmlinclude manifest.html

\author Radu Bogdan Rusu, Dejan  Pangercic

@b multi_pcd_generator is a simple node that loads PCD (Point Cloud Data) files from directory 
on a disk and publishes them as ROS messages in given interval.

 **/

// ROS core
#include <ros/node.h>
#include <sensor_msgs/PointCloud.h>
#include <point_cloud_mapping/cloud_io.h>
#include <tf/transform_broadcaster.h>

//for get_log_files()
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include "boost/filesystem.hpp"

using namespace std;
using namespace boost::filesystem; 

class MultiPCDGenerator
{
  protected:
    string tf_frame_;
    ros::NodeHandle nh_;
    tf::TransformBroadcaster broadcaster_;
    tf::Stamped<tf::Transform> transform_;

  public:

    // ROS messages
    sensor_msgs::PointCloud msg_cloud_;

  string file_name_, cloud_topic_, dir_;
  double rate_;
  vector<string> file_list_;

    ros::Publisher cloud_pub_;

    MultiPCDGenerator () : tf_frame_ ("base_link"),
                      transform_ (btTransform (btQuaternion (0, 0, 0), btVector3 (0, 0, 0)), ros::Time::now (), tf_frame_, tf_frame_)
    {
      // Maximum number of outgoing messages to be queued for delivery to subscribers = 1
      cloud_topic_ = "cloud_pcd";
      cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud> (cloud_topic_.c_str (), 1);
      ROS_INFO ("Publishing data on topic %s.", nh_.resolveName (cloud_topic_).c_str ());
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Start
    int
      start ()
    {
      get_log_files(dir_, file_list_);
      // if (file_name_ == "" || cloud_io::loadPCDFile (file_name_.c_str (), msg_cloud_) == -1)
      //  return (-1);
      //msg_cloud_.header.frame_id = tf_frame_;
      return (0);
    }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Get a list of lpcd files
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void get_log_files ( const path & directory, vector <string> &file_list, string suffix=".pcd", bool recurse_into_subdirs = false )
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
  
  ////////////////////////////////////////////////////////////////////////////////
    // Spin (!)
    bool spin ()
    {
      double interval = rate_ * 1e+6;
      while (nh_.ok ())
	
	{
	  for ( int pcd = 0; pcd < file_list_.size(); pcd++)
	    {
	      msg_cloud_.points.resize(0);
	      transform_.stamp_ = ros::Time::now ();
	      broadcaster_.sendTransform (transform_);
	      cloud_io::loadPCDFile (file_list_[pcd].c_str (), msg_cloud_);
	      msg_cloud_.header.frame_id = tf_frame_;
	      ROS_INFO ("Publishing data (%d points) on topic %s in frame %s.", (int)msg_cloud_.points.size (), 
			nh_.resolveName (cloud_topic_).c_str (), msg_cloud_.header.frame_id.c_str ());
	      msg_cloud_.header.stamp = ros::Time::now ();
	      cloud_pub_.publish (msg_cloud_);
	      
	      //if (interval == 0)                      // We only publish once if a 0 seconds interval is given
	      //  break;
	      usleep (interval);
	      ros::spinOnce ();
	    }
	}
      
      return (true);
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  if (argc < 3)
  {
    ROS_ERROR ("Syntax is: %s <dir> [publishing_interval between respective pcd(in seconds)]", argv[0]);
    return (-1);
  }

  ros::init (argc, argv, "multi_pcd_generator");

  MultiPCDGenerator c;
  c.dir_ = string (argv[1]);
  c.rate_      = atof (argv[2]);
  //ROS_INFO ("Loading file %s...", c.file_name_.c_str ());

  if (c.start () == -1)
  {
    ROS_ERROR ("Could not load file. Exiting.");
    return (-1);
  }
  c.spin ();

  return (0);
}
/* ]--- */
