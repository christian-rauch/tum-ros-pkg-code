/*
 * Copyright (c) 2008 Nico Blodow <blodow@cs.tum.edu>
 * Dejan Pangercic <pangercic -=- cs.tum.edu>
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

@brief msg_to_pcd subscribes to the /cloud_pcd topic and, depending on the
parameter configuration, saves pcd file(s) under the corresponding name. 

@par Advertises

@par Subscribes
- \b /cloud_pcd topic with PointCloud message

@par Parameters
- \b std::string input_cloud_topic_, dir_ (destination folder for pcds)
- \b std::string get_name_from_param_server_ (check if there is someone exporting the name for our pcd)
- \b int nr_saved_pcds_   
- \b bool debug_
- \b  std::string name_ (cloud's final name)
*/

// #include <unistd.h>

#include <ctime>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud.h>
#include <point_cloud_mapping/cloud_io.h>
#include <boost/thread/mutex.hpp>
#include <boost/format.hpp>
#include <ros/ros.h>
class MsgToPCD
{
protected:
  ros::NodeHandle nh_;
  //dir_ - destination folder for pcds
  std::string input_cloud_topic_, dir_;
  //check if there is someone exporting the name for our pcd
  std::string get_name_from_param_server_;
  ros::Subscriber cloud_sub_;
  int counter_;
  //for continous saving of pcds
  int nr_saved_pcds_; 
  bool debug_;
  std::string name_;
  //lock while saving to pcd
  boost::mutex m_lock_;
  boost::format filename_format_;
  
public:
  MsgToPCD () :  nh_("~"), counter_(0), debug_(true)
  {
    nh_.param ("input_cloud_topic", input_cloud_topic_, std::string("/cloud_pcd"));
    nh_.param ("name", name_, std::string("cloud"));
    nh_.param ("dir", dir_, std::string(""));       
    nh_.param ("nr_saved_pcds", nr_saved_pcds_, 1);   
    nh_.param ("get_name_from_param_server", get_name_from_param_server_, std::string(""));
    if(debug_)
      ROS_INFO("input_cloud_topic_: %s", input_cloud_topic_.c_str());
    cloud_sub_ = nh_.subscribe (input_cloud_topic_, 1, &MsgToPCD::cloud_cb, this);  
  }
  
  /**
   * \brief cloud_cb determines the correct pcd name to be used for saving and
   * writes the file to disk
   * \param cloud input point cloud data
   */
  void
  cloud_cb (const sensor_msgs::PointCloudConstPtr& cloud)
  {
    if(get_name_from_param_server_ != "")
    {
      ros::param::get(get_name_from_param_server_, name_);
      ROS_INFO("name: %s", name_.c_str());
    }
    filename_format_.parse(name_ + std::string("_%f.pcd"));
    std::string filename = dir_ + (filename_format_ %  ros::Time::now().toSec()).str();
    if(debug_)
      ROS_INFO("parameter nr_saved_pcds: %d", nr_saved_pcds_);
    if (counter_ < nr_saved_pcds_)
    {
      ROS_INFO ("PointCloud message received on %s with %d points. Saving to %s", 
                input_cloud_topic_.c_str (), (int)cloud->points.size (), filename.c_str ());
      m_lock_.lock ();
      cloud_io::savePCDFile (filename.c_str (), *cloud, true);
      m_lock_.unlock ();
    }
    counter_ ++;
  }
  
  /**
   * \brief spin spins until we do not reach the number of desired PCD 
   * messages to be saved (nr_saved_pcds_)
   */
  bool  spin ()
  {
    ros::Rate loop_rate(1);
    while (nh_.ok())
    {
      ros::spinOnce ();
      if (counter_ >= nr_saved_pcds_)
        return true;
      loop_rate.sleep();
    }
    return true;
  }
};

int main (int argc, char* argv[])
{
  ros::init (argc, argv, "msg_to_pcd");

  MsgToPCD n;
  n.spin ();

  return (0);
}

