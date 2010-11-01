/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  \author E. Gil Jones
 *********************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <collision_environment_msgs/MakeStaticCollisionMapAction.h>
#include <pr2_msgs/SetPeriodicCmd.h>
#include <cotesys_ros_grasping/TakeStaticCollisionMapAction.h>

namespace cotesys_ros_grasping
{

class TakeStaticCollisionMapServer
{

public:
  TakeStaticCollisionMapServer();
  
  bool execute(const cotesys_ros_grasping::TakeStaticCollisionMapGoalConstPtr& goal);

private:
  
  boost::shared_ptr<actionlib::SimpleActionClient<collision_environment_msgs::MakeStaticCollisionMapAction> > make_static_client_; 
  boost::shared_ptr<actionlib::SimpleActionServer<cotesys_ros_grasping::TakeStaticCollisionMapAction> > action_server_;
  
  ros::ServiceClient set_laser_client_;
  ros::NodeHandle priv_nh_, root_nh_;

  std::string cloud_source_;
  double laser_period_, laser_amplitude_, laser_offset_;
};

TakeStaticCollisionMapServer::TakeStaticCollisionMapServer()
  : priv_nh_("~")
{
  priv_nh_.param<double>("laser_period", laser_period_, 2);
  priv_nh_.param<double>("laser_amplitude", laser_amplitude_, .25);
  priv_nh_.param<double>("laser_offset", laser_offset_, .7);
  priv_nh_.param<std::string>("cloud_source", cloud_source_, "full_cloud_filtered");

  ros::service::waitForService("/laser_tilt_controller/set_periodic_cmd");
  set_laser_client_ = root_nh_.serviceClient<pr2_msgs::SetPeriodicCmd>("/laser_tilt_controller/set_periodic_cmd");
  
  make_static_client_.reset(new actionlib::SimpleActionClient<collision_environment_msgs::MakeStaticCollisionMapAction>("/make_static_collision_map", true));

  action_server_.reset(new actionlib::SimpleActionServer<cotesys_ros_grasping::TakeStaticCollisionMapAction>(root_nh_, "take_static_collision_map",
                                                                                                             boost::bind(&TakeStaticCollisionMapServer::execute, this, _1)));
}

bool TakeStaticCollisionMapServer::execute(const cotesys_ros_grasping::TakeStaticCollisionMapGoalConstPtr& goal)
{ 
  collision_environment_msgs::MakeStaticCollisionMapGoal stat_goal;
  stat_goal.cloud_source = cloud_source_;
  stat_goal.number_of_clouds = 2;
  make_static_client_->sendGoal(stat_goal);
 
  pr2_msgs::SetPeriodicCmd::Request laser_req;
  laser_req.command.header.stamp = ros::Time::now();
  laser_req.command.profile = "linear";
  laser_req.command.period = laser_period_;
  laser_req.command.amplitude = laser_amplitude_;
  laser_req.command.offset = laser_offset_;
  
  cotesys_ros_grasping::TakeStaticCollisionMapResult res;

  pr2_msgs::SetPeriodicCmd::Response laser_res;
  if(!set_laser_client_.call(laser_req, laser_res)) {
    ROS_WARN("Something wrong with laser");
    res.success = false;
    action_server_->setAborted(res);
    return true;
  }
 
  make_static_client_->waitForResult();
  res.success = false;
  action_server_->setSucceeded(res);

  return true;
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "take_static_collision_map");
  
  ros::AsyncSpinner spinner(1); // Use 1 thread
  spinner.start();

  cotesys_ros_grasping::TakeStaticCollisionMapServer take_static_map;

  ROS_INFO("Take static collision map started");
  ros::waitForShutdown();
    
  return 0;
}
