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

#include <move_arm_msgs/MoveArmAction.h>
#include <cotesys_ros_grasping/MoveArmToPositionAction.h>

namespace cotesys_ros_grasping
{

class MoveArmToPositionServer
{
public:
  MoveArmToPositionServer();

  bool execute(const cotesys_ros_grasping::MoveArmToPositionGoalConstPtr& goal);

private:

  std::string left_arm_name_, right_arm_name_;
  std::string right_ik_link_, left_ik_link_;

  double ik_to_gripper_x_diff_, ik_to_gripper_y_diff_, ik_to_gripper_z_diff_;

  double end_effector_rot_x_;
  double end_effector_rot_y_;
  double end_effector_rot_z_;
  double end_effector_rot_w_;

  ros::NodeHandle priv_nh_, root_nh_;
  boost::shared_ptr<actionlib::SimpleActionClient<move_arm_msgs::MoveArmAction> > left_move_arm_client_, right_move_arm_client_;
  boost::shared_ptr<actionlib::SimpleActionServer<cotesys_ros_grasping::MoveArmToPositionAction> > action_server_;
  
};

MoveArmToPositionServer::MoveArmToPositionServer()
  : priv_nh_("~")
{
  priv_nh_.param<std::string>("left_arm_name", left_arm_name_, "left_arm");
  priv_nh_.param<std::string>("right_arm_name", right_arm_name_, "right_arm");
  priv_nh_.param<std::string>("right_ik_link", right_ik_link_, "r_wrist_roll_link");
  priv_nh_.param<std::string>("left_ik_link", left_ik_link_, "l_wrist_roll_link");
  priv_nh_.param<double>("ik_to_gripper_x_diff", ik_to_gripper_x_diff_, .12);
  priv_nh_.param<double>("ik_to_gripper_y_diff", ik_to_gripper_y_diff_, 0.0);
  priv_nh_.param<double>("ik_to_gripper_z_diff", ik_to_gripper_z_diff_, 0.0);
  priv_nh_.param<double>("end_effector_rot_x", end_effector_rot_x_, 0.0);
  priv_nh_.param<double>("end_effector_rot_y", end_effector_rot_y_, 0.0);
  priv_nh_.param<double>("end_effector_rot_z", end_effector_rot_z_, 0.0);
  priv_nh_.param<double>("end_effector_rot_w", end_effector_rot_w_, 1.0);

  //TODO - test if the end effector parameters specify a valid quaternion

  left_move_arm_client_.reset(new actionlib::SimpleActionClient<move_arm_msgs::MoveArmAction>("left_move_arm_action", true));
  right_move_arm_client_.reset(new actionlib::SimpleActionClient<move_arm_msgs::MoveArmAction>("right_move_arm_action", true));

  //TODO - wait for the servers

  action_server_.reset(new actionlib::SimpleActionServer<cotesys_ros_grasping::MoveArmToPositionAction>(root_nh_, "move_arm_to_position",
                                                                                                        boost::bind(&MoveArmToPositionServer::execute, this, _1)));
}

bool MoveArmToPositionServer::execute(const cotesys_ros_grasping::MoveArmToPositionGoalConstPtr& req)
{
  if(req->arm_name != left_arm_name_ && req->arm_name != right_arm_name_) {
    ROS_ERROR_STREAM("Can't do anything for arm named " << req->arm_name);
  }
  
  std::string ik_link_name;
  boost::shared_ptr<actionlib::SimpleActionClient<move_arm_msgs::MoveArmAction> >  move_arm_client;
  if(req->arm_name == left_arm_name_) {
    ik_link_name = left_ik_link_;
    move_arm_client = left_move_arm_client_;
  } else {
    ik_link_name = right_ik_link_;
    move_arm_client = right_move_arm_client_;
  }

  move_arm_msgs::MoveArmGoal goal;

  goal.motion_plan_request.group_name=req->arm_name;
  goal.motion_plan_request.num_planning_attempts = 1;
  goal.motion_plan_request.planner_id = "";
  goal.planner_service_name = "ompl_planning/plan_kinematic_path";
  goal.motion_plan_request.allowed_planning_time = ros::Duration(2.0);
  
  goal.motion_plan_request.goal_constraints.position_constraints.resize(1);
  goal.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  goal.motion_plan_request.goal_constraints.position_constraints[0].header.frame_id = "base_link";
    
  goal.motion_plan_request.goal_constraints.position_constraints[0].link_name = ik_link_name;
  goal.motion_plan_request.goal_constraints.position_constraints[0].position.x = req->point.x-ik_to_gripper_x_diff_;
  goal.motion_plan_request.goal_constraints.position_constraints[0].position.y = req->point.y-ik_to_gripper_y_diff_;
  goal.motion_plan_request.goal_constraints.position_constraints[0].position.z = req->point.z-ik_to_gripper_z_diff_;
    
  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.type = geometric_shapes_msgs::Shape::BOX;
  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.02);
  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.02);
  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.02);

  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_orientation.w = 1.0;
  goal.motion_plan_request.goal_constraints.position_constraints[0].weight = 1.0;

  goal.motion_plan_request.goal_constraints.orientation_constraints.resize(1);
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].header.stamp = ros::Time::now();
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].header.frame_id = "base_link";
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].link_name = ik_link_name;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.x = end_effector_rot_x_;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.y = end_effector_rot_y_;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.z = end_effector_rot_z_;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.w = end_effector_rot_w_;
    
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_roll_tolerance = 0.04;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_pitch_tolerance = 0.04;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_yaw_tolerance = 0.04;

  goal.motion_plan_request.goal_constraints.orientation_constraints[0].weight = 1.0;

  cotesys_ros_grasping::MoveArmToPositionResult res;

  move_arm_client->sendGoal(goal);
  bool call_ok = move_arm_client->waitForResult(ros::Duration(30.0));
  if(!call_ok) {
    res.error_code = move_arm_client->getResult()->error_code;
    action_server_->setAborted(res);
  } else {
    actionlib::SimpleClientGoalState state = move_arm_client->getState();
    if(state != actionlib::SimpleClientGoalState::SUCCEEDED) {
      res.error_code = move_arm_client->getResult()->error_code;
      action_server_->setAborted(res);
    } else {
      res.error_code = move_arm_client->getResult()->error_code;
      action_server_->setSucceeded(res);
    } 
  }
  return true;
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_arm_to_position");
  
  ros::AsyncSpinner spinner(1); // Use 1 thread
  spinner.start();

  cotesys_ros_grasping::MoveArmToPositionServer move_arm_pos;

  ROS_INFO("Move_arm_to_position action started");
  ros::waitForShutdown();
    
  return 0;
}
