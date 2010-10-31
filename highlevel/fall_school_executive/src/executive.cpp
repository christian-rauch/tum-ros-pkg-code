/***********************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
* Author: Brian Gerkey
***********************************************************/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

// Messages
#include <approach_table_tools/GetCheckerboardPose.h>
#include <approach_table_tools/GetApproachPose.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <pr2_common_action_msgs/TuckArmsAction.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "executive");
  ros::NodeHandle n;

  // Lower the torso
  actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> torso_client("/torso_controller/position_joint_action", true);
  torso_client.waitForServer();
  pr2_controllers_msgs::SingleJointPositionGoal torso_goal;
  torso_goal.position = 0.03;
  torso_goal.min_duration = ros::Duration(2.0);
  torso_goal.max_velocity = 1.0;
  ROS_INFO("lowering the torso");
  torso_client.sendGoal(torso_goal);
  torso_client.waitForResult();
  if(torso_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_BREAK();

  // Tuck the arms
  actionlib::SimpleActionClient<pr2_common_action_msgs::TuckArmsAction> tuck_arms_client("tuck_arms", true);
  tuck_arms_client.waitForServer();
  pr2_common_action_msgs::TuckArmsGoal tuck_arms_goal;
  tuck_arms_goal.tuck_left = true;
  tuck_arms_goal.tuck_right = true;
  tuck_arms_client.sendGoal(tuck_arms_goal);
  tuck_arms_client.waitForResult();
  if(tuck_arms_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_BREAK();
  
  // Get the checkerboard pose
  ros::service::waitForService("get_checkerboard_pose");
  approach_table_tools::GetCheckerboardPoseRequest cb_req;
  approach_table_tools::GetCheckerboardPoseResponse cb_resp;
  cb_req.corners_x = 5;
  cb_req.corners_y = 4;
  cb_req.spacing_x = 0.08;
  cb_req.spacing_y = 0.08;
  ROS_INFO("Attempting to get the pose of the board");
  if(!ros::service::call("get_checkerboard_pose", cb_req, cb_resp))
    ROS_BREAK();
  ROS_INFO("Got board pose: %.3f %.3f %.3f", 
           cb_resp.board_pose.pose.position.x,
           cb_resp.board_pose.pose.position.y,
           cb_resp.board_pose.pose.position.z);

  // Get the approach pose
  ros::service::waitForService("get_approach_pose");
  approach_table_tools::GetApproachPoseRequest ap_req;
  approach_table_tools::GetApproachPoseResponse ap_resp;
  ap_req.board_pose = cb_resp.board_pose;
  ROS_INFO("Attempting to get approach pose");
  if(!ros::service::call("get_approach_pose", ap_req, ap_resp))
    ROS_BREAK();
  ROS_INFO("Got approach pose: %.3f %.3f %.3f", 
           ap_resp.nav_pose.pose.position.x,
           ap_resp.nav_pose.pose.position.y,
           ap_resp.nav_pose.pose.position.z);
  // For visualiation
  ros::Publisher nav_pose_pub = n.advertise<geometry_msgs::PoseStamped>("nav_pose", 1, true);
  nav_pose_pub.publish(ap_resp.nav_pose);

  // Approach the table
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_local_client("move_base_local", true);
  move_base_local_client.waitForServer();
  move_base_msgs::MoveBaseGoal move_base_local_goal;
  move_base_local_goal.target_pose = ap_resp.nav_pose;
  move_base_local_client.sendGoal(move_base_local_goal);
  move_base_local_client.waitForResult();
  if(move_base_local_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_BREAK();

  // Untuck the arms
  tuck_arms_goal.tuck_left = false;
  tuck_arms_goal.tuck_right = false;
  tuck_arms_client.sendGoal(tuck_arms_goal);
  tuck_arms_client.waitForResult();
  if(tuck_arms_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_BREAK();

  // Raise the torso
  torso_goal.position = 0.195;
  torso_goal.min_duration = ros::Duration(2.0);
  torso_goal.max_velocity = 1.0;
  ROS_INFO("raising the torso");
  torso_client.sendGoal(torso_goal);
  torso_client.waitForResult();
  if(torso_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_BREAK();

  // Move close to the table, using local navigation (NO OBSTACLE AVOIDANCE!)
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> drive_base_client("drive_base_action", true);
  drive_base_client.waitForServer();
  move_base_msgs::MoveBaseGoal drive_base_goal;
  drive_base_goal.target_pose.header.frame_id = "odom_combined";
  drive_base_goal.target_pose.pose.position.x = 0.5;
  drive_base_goal.target_pose.pose.position.y = 0.0;
  drive_base_goal.target_pose.pose.position.z = 0.0;
  drive_base_goal.target_pose.pose.orientation.x = 0.0;
  drive_base_goal.target_pose.pose.orientation.y = 0.0;
  drive_base_goal.target_pose.pose.orientation.z = 0.0;
  drive_base_goal.target_pose.pose.orientation.w = 1.0;
  drive_base_client.sendGoal(drive_base_goal);
  drive_base_client.waitForResult();
  if(drive_base_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_BREAK();

  // Move to the right-hand table, using local navigation (NO OBSTACLE AVOIDANCE!)
  drive_base_goal.target_pose.header.frame_id = "odom_combined";
  drive_base_goal.target_pose.pose.position.x = 0.0;
  drive_base_goal.target_pose.pose.position.y = -1.1;
  drive_base_goal.target_pose.pose.position.z = 0.0;
  drive_base_goal.target_pose.pose.orientation.x = 0.0;
  drive_base_goal.target_pose.pose.orientation.y = 0.0;
  drive_base_goal.target_pose.pose.orientation.z = 0.0;
  drive_base_goal.target_pose.pose.orientation.w = 1.0;
  drive_base_client.sendGoal(drive_base_goal);
  drive_base_client.waitForResult();
  if(drive_base_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_BREAK();
  
  ros::spin();
  return 0;
}
