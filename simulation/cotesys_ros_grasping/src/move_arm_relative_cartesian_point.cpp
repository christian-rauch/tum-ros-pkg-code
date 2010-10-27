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
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <motion_planning_msgs/GetMotionPlan.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <cotesys_ros_grasping/MoveArmRelativeCartesianPointAction.h>

namespace cotesys_ros_grasping
{

class MoveArmRelativeCartesianPointServer
{
public:
  MoveArmRelativeCartesianPointServer();

  bool execute(const cotesys_ros_grasping::MoveArmRelativeCartesianPointGoalConstPtr& goal);

private:

  std::string left_arm_name_, right_arm_name_;
  std::string right_ik_link_, left_ik_link_;

  bool disable_collisions_;

  tf::TransformListener tf_;

  ros::NodeHandle priv_nh_, root_nh_;
  ros::ServiceClient left_interp_ik_client_, right_interp_ik_client_; 
  boost::shared_ptr<actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> > right_arm_traj_client_, left_arm_traj_client_;
  boost::shared_ptr<actionlib::SimpleActionServer<cotesys_ros_grasping::MoveArmRelativeCartesianPointAction> > action_server_;
  
};

MoveArmRelativeCartesianPointServer::MoveArmRelativeCartesianPointServer()
  : priv_nh_("~")
{
  priv_nh_.param<std::string>("left_arm_name", left_arm_name_, "left_arm");
  priv_nh_.param<std::string>("right_arm_name", right_arm_name_, "right_arm");
  priv_nh_.param<std::string>("right_ik_link", right_ik_link_, "r_wrist_roll_link");
  priv_nh_.param<std::string>("left_ik_link", left_ik_link_, "l_wrist_roll_link");
  priv_nh_.param<bool>("disable_collisions", disable_collisions_, false);  

  left_interp_ik_client_ = root_nh_.serviceClient<motion_planning_msgs::GetMotionPlan>("/l_interpolated_ik_motion_plan");
  right_interp_ik_client_ = root_nh_.serviceClient<motion_planning_msgs::GetMotionPlan>("/r_interpolated_ik_motion_plan");

  right_arm_traj_client_.reset(new actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>("/r_arm_controller/joint_trajectory_action", true));
  left_arm_traj_client_.reset(new actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>("/l_arm_controller/joint_trajectory_action", true));

  //TODO - wait for the servers

  action_server_.reset(new actionlib::SimpleActionServer<cotesys_ros_grasping::MoveArmRelativeCartesianPointAction>(root_nh_, "move_arm_relative_cartesian_point",
                                                                                                        boost::bind(&MoveArmRelativeCartesianPointServer::execute, this, _1)));
}

bool MoveArmRelativeCartesianPointServer::execute(const cotesys_ros_grasping::MoveArmRelativeCartesianPointGoalConstPtr& req)
{
  if(req->arm_name != left_arm_name_ && req->arm_name != right_arm_name_) {
    ROS_ERROR_STREAM("Can't do anything for arm named " << req->arm_name);
  }
  
  std::string ik_link_name;
  ros::ServiceClient* interp_ik_client;
  boost::shared_ptr<actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> > joint_traj_client;
  if(req->arm_name == left_arm_name_) {
    interp_ik_client = &left_interp_ik_client_;
    joint_traj_client = left_arm_traj_client_;
    ik_link_name = left_ik_link_;
  } else {
    interp_ik_client = &right_interp_ik_client_;
    joint_traj_client = right_arm_traj_client_;
    ik_link_name = right_ik_link_;
  }

  //now we need to get the current position of the the link in the base_link frame
  tf::StampedTransform cur_ik_link_pos;
  try {
    tf_.lookupTransform("/base_link", "/"+ik_link_name, ros::Time(), cur_ik_link_pos);
  } catch(...) {
    ROS_WARN("Tf error");
  }
  geometry_msgs::Point rel_pos;
  rel_pos.x = cur_ik_link_pos.getOrigin().x()+req->rel_point.x;
  rel_pos.y = cur_ik_link_pos.getOrigin().y()+req->rel_point.y;
  rel_pos.z = cur_ik_link_pos.getOrigin().z()+req->rel_point.z;

  motion_planning_msgs::GetMotionPlan::Request plan_req;

  plan_req.motion_plan_request.group_name=req->arm_name;
  plan_req.motion_plan_request.num_planning_attempts = 1;
  plan_req.motion_plan_request.planner_id = "";
  plan_req.motion_plan_request.allowed_planning_time = ros::Duration(2.0);
  
  //interpolated IK requires start state
  plan_req.motion_plan_request.start_state.multi_dof_joint_state.child_frame_id = ik_link_name;
  geometry_msgs::Pose pose;
  tf::poseTFToMsg(cur_ik_link_pos, pose);
  ROS_INFO_STREAM("Ik link name is " << ik_link_name);
  plan_req.motion_plan_request.start_state.multi_dof_joint_state.pose = pose;
  plan_req.motion_plan_request.start_state.multi_dof_joint_state.frame_id = "base_link";
  plan_req.motion_plan_request.start_state.multi_dof_joint_state.stamp = cur_ik_link_pos.stamp_;

  plan_req.motion_plan_request.goal_constraints.position_constraints.resize(1);
  plan_req.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  plan_req.motion_plan_request.goal_constraints.position_constraints[0].header.frame_id = "base_link";
    
  plan_req.motion_plan_request.goal_constraints.position_constraints[0].link_name = ik_link_name;
  plan_req.motion_plan_request.goal_constraints.position_constraints[0].position.x = rel_pos.x;
  plan_req.motion_plan_request.goal_constraints.position_constraints[0].position.y = rel_pos.y;
  plan_req.motion_plan_request.goal_constraints.position_constraints[0].position.z = rel_pos.z;
    
  plan_req.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.type = geometric_shapes_msgs::Shape::BOX;
  plan_req.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.02);
  plan_req.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.02);
  plan_req.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.02);

  plan_req.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_orientation.w = 1.0;
  plan_req.motion_plan_request.goal_constraints.position_constraints[0].weight = 1.0;

  plan_req.motion_plan_request.goal_constraints.orientation_constraints.resize(1);
  plan_req.motion_plan_request.goal_constraints.orientation_constraints[0].header.stamp = ros::Time::now();
  plan_req.motion_plan_request.goal_constraints.orientation_constraints[0].header.frame_id = "base_link";
  plan_req.motion_plan_request.goal_constraints.orientation_constraints[0].link_name = ik_link_name;
  plan_req.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.x = cur_ik_link_pos.getRotation().x();
  plan_req.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.y = cur_ik_link_pos.getRotation().y();
  plan_req.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.z = cur_ik_link_pos.getRotation().z();
  plan_req.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.w = cur_ik_link_pos.getRotation().w();
    
  plan_req.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_roll_tolerance = 0.04;
  plan_req.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_pitch_tolerance = 0.04;
  plan_req.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_yaw_tolerance = 0.04;

  plan_req.motion_plan_request.goal_constraints.orientation_constraints[0].weight = 1.0;

  if(disable_collisions_) {
    plan_req.motion_plan_request.ordered_collision_operations.collision_operations.resize(1);
    plan_req.motion_plan_request.ordered_collision_operations.collision_operations[0].object1 = 
      plan_req.motion_plan_request.ordered_collision_operations.collision_operations[0].COLLISION_SET_ALL;
    plan_req.motion_plan_request.ordered_collision_operations.collision_operations[0].object2 = 
      plan_req.motion_plan_request.ordered_collision_operations.collision_operations[0].COLLISION_SET_ALL;
    plan_req.motion_plan_request.ordered_collision_operations.collision_operations[0].operation = 
      plan_req.motion_plan_request.ordered_collision_operations.collision_operations[0].DISABLE;
  }

  cotesys_ros_grasping::MoveArmRelativeCartesianPointResult cart_res;

  motion_planning_msgs::GetMotionPlan::Response plan_res;
  
  if(!interp_ik_client->call(plan_req,plan_res)) {
    ROS_WARN("Interpolated ik call failed");
    cart_res.success = false;
    action_server_->setAborted(cart_res);
    return true;
  }
  for(unsigned int i = 0; i < plan_res.trajectory_error_codes.size(); i++) {
    if(plan_res.trajectory_error_codes[i].val != plan_res.trajectory_error_codes[i].SUCCESS) {
      ROS_WARN_STREAM("Interpolated ik call did not succeed " << plan_res.error_code.val << " for point " << i);
      cart_res.success = false;
      action_server_->setAborted(cart_res);
      return true;
    }
  }

  pr2_controllers_msgs::JointTrajectoryGoal traj_goal;  
  traj_goal.trajectory = plan_res.trajectory.joint_trajectory;
  
  joint_traj_client->sendGoal(traj_goal);

  bool call_ok = joint_traj_client->waitForResult(ros::Duration(30.0));
  if(!call_ok) {
    ROS_WARN("Call to traj client not ok");
  }
  
  actionlib::SimpleClientGoalState state = joint_traj_client->getState();
  cart_res.success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
  if(cart_res.success) {
    action_server_->setSucceeded(cart_res);
  } else {
    action_server_->setAborted(cart_res);
  }
  return true;
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_arm_relative_cartesian_point");
  
  ros::AsyncSpinner spinner(1); // Use 1 thread
  spinner.start();
  ros::NodeHandle nh("~");

  cotesys_ros_grasping::MoveArmRelativeCartesianPointServer move_arm_pos;

  ROS_INFO("Move_arm_to_position action started");
  ros::waitForShutdown();
    
  return 0;
}
