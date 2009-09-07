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
 * $Id: pointcloud_delimiter.cpp 161 2009-09-06 15:18:34Z dejanpan $
 *
 */

/**
@mainpage

@htmlinclude manifest.html

\author Dejan Pangercic

@b pointcloud_delimiter cuts out a volume out of the pointcloud(s)

 **/

// ROS core
#include <ros/ros.h>
// ROS messages
#include <sensor_msgs/PointCloud.h>

#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/thread/mutex.hpp>
//for get_log_files()
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include "boost/filesystem.hpp"
//for savePCDFile ()
#include <point_cloud_mapping/cloud_io.h>

using namespace std;
using namespace ros;
using namespace sensor_msgs;
using namespace cloud_io;
using namespace boost::filesystem; 

struct delimiter
{
  double min_x, max_x, min_y, max_y, min_z, max_z;
};

class PointcloudDelimiter
{
protected:
  NodeHandle nh_;
  
  public:
    // ROS messages
  PointCloud cloud_in_, cloud_out_;
  delimiter  del_;
  string dir_;
  vector<string> file_list_;
  int normalize_;
  double norm_;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Constructor
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  PointcloudDelimiter()
    {
      nh_.param ("~dir", dir_, string(""));     //dir to fetch .pcd files from
      if(dir_ == "")
	ROS_ERROR("You must specify path to .pcd files");
      //get the list of .log files
      get_log_files(dir_, file_list_, ".pcd");
	
      nh_.param ("~min_x", del_.min_x, 0.0);     // minimum x to be considered
      nh_.param ("~max_x", del_.max_x, 0.0);   // maximum x to be considered
      
      nh_.param ("~min_y", del_.min_y, 0.0);     // minimum y to be considered
      nh_.param ("~max_y", del_.max_y, 0.0);   // maximum y to be considered

      nh_.param ("~min_z", del_.min_z, 0.0);     // minimum z to be considered
      nh_.param ("~max_z", del_.max_z, 0.0);   // maximum z to be considered
      nh_.param ("~normalize", normalize_, 0);   // shall we normalize data?
      nh_.param ("~norm", norm_, 1.0);   // normalization coefficient

      if(!normalize_)
	{
	  cloud_out_.header.frame_id = "base_link";
	  cloud_out_.channels.resize (7);
	  cloud_out_.channels[0].name = "intensity";
	  cloud_out_.channels[1].name = "distance";
	  cloud_out_.channels[2].name = "sid";
	  cloud_out_.channels[3].name = "pid";
	  cloud_out_.channels[4].name = "vx";
	  cloud_out_.channels[5].name = "vy";
	  cloud_out_.channels[6].name = "vz";
	}
      }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Destructor
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  virtual ~PointcloudDelimiter () { }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Get a list of pcd files
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
    

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Get a list of log files
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  bool
  spin ()
  {
    int nr_points = 0;
    unsigned int progress = 0;
    ROS_INFO("minima/maxima x, y, z : %f, %f, %f, %f, %f, %f", del_.min_x, del_.max_x, del_.min_y, del_.max_y, del_.max_z, del_.min_z);
    ROS_INFO("file_list_ size %d", file_list_.size());
    while (nh_.ok ())
      {
	for (unsigned int i = 0; i < file_list_.size(); i++)
	  {
	    ROS_INFO("Loading file %s", file_list_[i].c_str());
	    loadPCDFile(file_list_[i].c_str(), cloud_in_);
	    ROS_INFO("cloud_in_ with nr_points %d", cloud_in_.points.size());
	    //reset sizes
	    nr_points = 0, progress = 0;
	    cloud_out_.points.resize(0);
	    if(!normalize_)
	      {
		for (unsigned int d = 0; d < cloud_in_.channels.size (); d++)
		  cloud_out_.channels[d].values.resize (0);
		cloud_out_.header =  cloud_in_.header;
	      }
	    for (unsigned int j = 0; j < cloud_in_.points.size(); j++)
	      {
		if(!normalize_)
		  {
		    if ((del_.min_x <= cloud_in_.points[j].x) && (cloud_in_.points[j].x <= del_.max_x) &&
			(del_.min_y <= cloud_in_.points[j].y) && (cloud_in_.points[j].y <= del_.max_y) &&
			(del_.min_z <= cloud_in_.points[j].z) && (cloud_in_.points[j].z <= del_.max_z))
		      {
			//resize
			cloud_out_.points.resize(nr_points + 1);
			for (unsigned int d = 0; d < cloud_in_.channels.size (); d++)
			  cloud_out_.channels[d].values.resize (nr_points + 1);
			
			//fill with values
			cloud_out_.points[nr_points].x =  cloud_in_.points[j].x;
			cloud_out_.points[nr_points].y =  cloud_in_.points[j].y;
			cloud_out_.points[nr_points].z =  cloud_in_.points[j].z;
			// Save the rest of the values
			cloud_out_.channels[0].values[nr_points] =  cloud_in_.channels[0].values[j];
			cloud_out_.channels[1].values[nr_points] =  cloud_in_.channels[1].values[j];
			cloud_out_.channels[2].values[nr_points] =  cloud_in_.channels[2].values[j];
			cloud_out_.channels[3].values[nr_points] =  cloud_in_.channels[3].values[j];
			cloud_out_.channels[4].values[nr_points] =  cloud_in_.channels[4].values[j];
			cloud_out_.channels[5].values[nr_points] =  cloud_in_.channels[5].values[j];
			cloud_out_.channels[6].values[nr_points] =  cloud_in_.channels[6].values[j];
			nr_points++;
		      }
		  }
		//if data need to be normalized (like the ones from david)
		if(normalize_)
		  {
		    cloud_out_.points.resize(nr_points + 1);
		    cloud_out_.points[nr_points].x =  cloud_in_.points[j].x/norm_;
		    cloud_out_.points[nr_points].y =  cloud_in_.points[j].y/norm_;
		    cloud_out_.points[nr_points].z =  cloud_in_.points[j].z/norm_;
		    nr_points++;
		  }

		if (j == int(cloud_in_.points.size() * (float(progress + 1)/10)) )
		  {
		    ROS_INFO("%d percent", (progress+1) * 10);
		    progress++;
		  }
	      }
	    if(cloud_out_.points.size() != 0)		
	      {
		int str_len = file_list_[i].length();
		string pcd_del = file_list_[i].erase((str_len - 4), 4);
		if(!normalize_)
		  pcd_del += ".delimited.pcd";
		else 
		  pcd_del += ".normalized.pcd";
		ROS_INFO("Saving file %s with nr. of points %d", pcd_del.c_str(), nr_points);
		savePCDFile (pcd_del.c_str(), cloud_out_, false);
		pcd_del.clear();
	      }
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
  init (argc, argv, "actarray_cloud_out_assembler");

  PointcloudDelimiter p;
  p.spin ();

  return (0);
}
/* ]--- */
