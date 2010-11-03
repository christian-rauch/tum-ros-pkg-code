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
#include <sstream>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <mapping_msgs/AttachedCollisionObject.h>

#include <cotesys_ros_grasping/AttachBoundingBoxAction.h>

namespace cotesys_ros_grasping
{

class AttachBoundingBoxServer
{
public:
  
  AttachBoundingBoxServer();

  bool execute(const cotesys_ros_grasping::AttachBoundingBoxGoalConstPtr& goal);

private:
  
  boost::shared_ptr<actionlib::SimpleActionServer<cotesys_ros_grasping::AttachBoundingBoxAction> > action_server_;
  ros::NodeHandle priv_nh_, root_nh_;
  ros::Publisher attached_object_publisher_;

  std::string default_object_name_;
  std::string left_arm_name_, right_arm_name_;
  std::string right_attach_link_, left_attach_link_;

  std::vector<std::string> left_end_effector_links_;
  std::vector<std::string> right_end_effector_links_;

  std::string left_end_effector_planning_group_, right_end_effector_planning_group_;

  double link_to_gripper_x_diff_, link_to_gripper_y_diff_, link_to_gripper_z_diff_;

  tf::TransformListener tf_;
};

AttachBoundingBoxServer::AttachBoundingBoxServer()
  : priv_nh_("~")
{
  priv_nh_.param<std::string>("left_arm_name", left_arm_name_, "left_arm");
  priv_nh_.param<std::string>("right_arm_name", right_arm_name_, "right_arm");
  priv_nh_.param<std::string>("default_object_name", default_object_name_, "_object");
  priv_nh_.param<std::string>("right_attach_link", right_attach_link_, "r_gripper_palm_link");
  priv_nh_.param<std::string>("left_attach_link", left_attach_link_, "l_gripper_palm_link");
  priv_nh_.param<std::string>("left_end_effector_planning_group", left_end_effector_planning_group_,"l_end_effector");
  priv_nh_.param<std::string>("right_end_effector_planning_group", right_end_effector_planning_group_,"r_end_effector");
  priv_nh_.param<double>("link_to_gripper_x_diff", link_to_gripper_x_diff_, .06);
  priv_nh_.param<double>("link_to_gripper_y_diff", link_to_gripper_y_diff_, .00);
  priv_nh_.param<double>("link_to_gripper_z_diff", link_to_gripper_z_diff_, .00);

  //getting end effector link members
  XmlRpc::XmlRpcValue group_list;
  root_nh_.getParam("/robot_description_planning/groups", group_list);
  if(group_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_WARN("Problem getting group list");
  }
  bool found_left = false;
  bool found_right = false;
  for(int i = 0; i < group_list.size(); i++) {
    std::vector<std::string>* group_link_vector = NULL;
  
    if(group_list[i]["name"] == left_end_effector_planning_group_) {
      found_left = true;
      group_link_vector = &left_end_effector_links_;
    } else if(group_list[i]["name"] == right_end_effector_planning_group_) {
      found_right = true;
      group_link_vector = &right_end_effector_links_;
    }
    if(group_link_vector != NULL) {
      if(!group_list[i].hasMember("links")) {
        ROS_WARN("No links in end effector group");
        continue;
      }
      std::string link_list = std::string(group_list[i]["links"]);
      std::stringstream link_name_stream(link_list);
      while(link_name_stream.good() && !link_name_stream.eof()){
        std::string lname; 
        link_name_stream >> lname;
        if(lname.size() == 0) continue;
        group_link_vector->push_back(lname);
      }
    }
  }
  if(!found_right) {
    ROS_INFO_STREAM("Couldn't find planning group for " << right_end_effector_planning_group_);
  }

  if(!found_left) {
    ROS_INFO_STREAM("Couldn't find planning group for " << left_end_effector_planning_group_);
  }
  
  attached_object_publisher_ = root_nh_.advertise<mapping_msgs::AttachedCollisionObject>("attached_collision_object", true);
  
  action_server_.reset(new actionlib::SimpleActionServer<cotesys_ros_grasping::AttachBoundingBoxAction>(root_nh_, "attach_bounding_box",
                                                                                                        boost::bind(&AttachBoundingBoxServer::execute, this, _1)));
};

bool AttachBoundingBoxServer::execute(const cotesys_ros_grasping::AttachBoundingBoxGoalConstPtr& req)
{
  if(req->arm_name != left_arm_name_ && req->arm_name != right_arm_name_) {
    ROS_ERROR_STREAM("Can't attach to arm named " << req->arm_name);
    action_server_->setAborted();
    return true;
  }

  std::string att_link_name;
  std::vector<std::string>* touch_link_vector;
  if(req->arm_name == left_arm_name_) {
    att_link_name = left_attach_link_;
    touch_link_vector = &left_end_effector_links_;
  } else {
    att_link_name = right_attach_link_;
    touch_link_vector = &right_end_effector_links_;
  }

  mapping_msgs::AttachedCollisionObject att_object;

  att_object.link_name = att_link_name;
  att_object.object = req->object;
  if(req->remove) {
    att_object.object.operation.operation = mapping_msgs::CollisionObjectOperation::REMOVE;
  } else {
    att_object.object.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;

    att_object.touch_links = *touch_link_vector;
  }

  attached_object_publisher_.publish(att_object);

  action_server_->setSucceeded();
  return true;
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "attach_bounding_box_server");
  
  ros::AsyncSpinner spinner(1); // Use 1 thread
  spinner.start();

  cotesys_ros_grasping::AttachBoundingBoxServer attach_bounding_box;

  ROS_INFO("Attach bounding box server started");
  ros::waitForShutdown();
    
  return 0;
}
