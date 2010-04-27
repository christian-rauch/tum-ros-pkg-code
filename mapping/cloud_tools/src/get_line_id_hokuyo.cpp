/*
 * Copyright (c) 2008 Dejan Pangercic <pangercic -=- cs.tum.edu>
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
 *
 */

/** 
@file

@brief get_line_id_hokuyo reads out all *.pcd files from a given directory dir_
and retrieves scan "line" IDs for Hokuyo UTM 30LX laser scanner. It is needed
since ROS's driver only provides an "index" that is respective beam ID in one
laser scan.

@par Advertises

@par Subscribes

@par Parameters
- \b string dir_, tf_frame_
*/

// ROS core
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud.h>
#include <point_cloud_mapping/cloud_io.h>
#include <tf/transform_broadcaster.h>


//for get_log_files()
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include "boost/filesystem.hpp"

using namespace std;
using namespace boost::filesystem; 

class GetLineIDHokuyo
{
protected:
  ros::NodeHandle nh_;

public:
  // ROS messages
  sensor_msgs::PointCloud msg_cloud_;
  //Parameters
  string dir_, tf_frame_;
  vector<string> file_list_;
  
  ////////////////////////////////////////////////////////////////////////////////
  /**
   * Constructor
   */
  GetLineIDHokuyo () : nh_("~"),  tf_frame_ ("/base_link")
  {
    //nothing to do
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief read pcd files from dir_
   */
  int start ()
  {
    get_log_files(dir_, file_list_);
    return (0);
  }
  
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief get a list of pcd files
   * \param directory directory with pcd files
   * \param file_list a list of pcd files
   * \param suffix files' suffix (pcd by default)
   * \param recurse_into_subdirs whether we want to read out files from 
   * subdirectories or not (false by default)
   */
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
          if((unsigned long)(len - npos) == suffix.length())
            file_list.push_back(iter->string());
        }
    }
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief retrieve scan line IDs
   * \param cloud input point cloud 
   * \param line_index index in the point cloud channel
   **/
  void get_line_id (sensor_msgs::PointCloud &cloud, size_t line_index)
  {
    int line_id = 0;
    int point_id = 0;
    int temp_point_id = 0;
    ros::Time ts = ros::Time::now ();
    cloud.channels.resize(cloud.channels.size()+1);
    cloud.channels[line_index].name = "line";
    int j = cloud_io::getIndex (cloud, "index");
    cloud.channels[line_index].values.resize(cloud.channels[j].values.size());
    for (unsigned int k = 0; k < cloud.channels[j].values.size()-1; k++)
    {
      point_id = cloud.channels[j].values[k];
      temp_point_id =  cloud.channels[j].values[k+1];
      cloud.channels[line_index].values[k] = line_id;
      //new line found
      if (temp_point_id < point_id)
      {
        line_id++;
      }
    }
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief node's spin function. It iterates over all pcd files in file_list_, 
   * finds line IDs, adds them as a channel and saves the file under the same name.
   */
  bool spin ()
  {
    ros::spinOnce ();
    for ( unsigned long pcd = 0; pcd < file_list_.size(); pcd++)
    {
      msg_cloud_.points.resize(0);            
      msg_cloud_.channels.resize(0);            
      ROS_INFO("Loading file %s", file_list_[pcd].c_str());
      cloud_io::loadPCDFile (file_list_[pcd].c_str (), msg_cloud_);
      msg_cloud_.header.frame_id = tf_frame_;
      msg_cloud_.header.stamp = ros::Time::now ();
      if (cloud_io::getIndex (msg_cloud_, "line") != -1)
      {
        ROS_WARN("file %s already contains channel named \"lines\"", file_list_[pcd].c_str());
        continue;
      }
      get_line_id (msg_cloud_, msg_cloud_.channels.size());
      ROS_INFO("Writting to file %s", file_list_[pcd].c_str());
      cloud_io::savePCDFile (file_list_[pcd].c_str(), msg_cloud_, false);
    }    
    return (true);
  }
};

/* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 2)
  {
    ROS_ERROR ("Syntax is: %s <dir>", argv[0]);
    return (-1);
  }  
  ros::init (argc, argv, "get_line_id_hokuyo");
  
  GetLineIDHokuyo c;
  c.dir_ = string (argv[1]);
  
  if (c.start () == -1)
  {
    ROS_ERROR ("Could not load file. Exiting.");
    return (-1);
  }
  c.spin ();
  
  return (0);
}
/* ]--- */
