/* 
 * Copyright (c) 2010, Thomas Ruehr <ruehr@cs.tum.edu>
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
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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
 */


/*#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_arm_msgs/MoveArmAction.h>

int main(int argc, char **argv){
  ros::init (argc, argv, "move_arm_joint_goal_test");
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<move_arm_msgs::MoveArmAction> move_arm("move_right_arm",true);

  move_arm.waitForServer();
  ROS_INFO("Connected to server");

  move_arm_msgs::MoveArmGoal goalA;

  goalA.motion_plan_request.group_name = "right_arm";
  goalA.motion_plan_request.num_planning_attempts = 1;
  goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  nh.param<std::string>("planner_id",goalA.motion_plan_request.planner_id,std::string(""));
  nh.param<std::string>("planner_service_name",goalA.planner_service_name,std::string("ompl_planning/plan_kinematic_path"));
  goalA.motion_plan_request.goal_constraints.set_position_constraints_size(2);
  goalA.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  goalA.motion_plan_request.goal_constraints.position_constraints[0].header.frame_id = "base_link";

  goalA.motion_plan_request.goal_constraints.position_constraints[0].link_name = "r_wrist_roll_link";
  goalA.motion_plan_request.goal_constraints.position_constraints[0].position.x = 0.527;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].position.y = -0.258;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].position.z = 0.687;

  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.type = geometric_shapes_msgs::Shape::BOX;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.05);
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.05);
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.05);

  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_orientation.w = 1.0;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].weight = 0.5;

  goalA.motion_plan_request.goal_constraints.position_constraints[1].header.stamp = ros::Time::now();
  goalA.motion_plan_request.goal_constraints.position_constraints[1].header.frame_id = "base_footprint";

  goalA.motion_plan_request.goal_constraints.position_constraints[1].link_name = "r_elbow_flex_link";
  goalA.motion_plan_request.goal_constraints.position_constraints[1].position.x = 0;
  goalA.motion_plan_request.goal_constraints.position_constraints[1].position.y = 0;
  goalA.motion_plan_request.goal_constraints.position_constraints[1].position.z = 5.82;

  goalA.motion_plan_request.goal_constraints.position_constraints[1].constraint_region_shape.type = geometric_shapes_msgs::Shape::BOX;
  goalA.motion_plan_request.goal_constraints.position_constraints[1].constraint_region_shape.dimensions.push_back(10.0f);
  goalA.motion_plan_request.goal_constraints.position_constraints[1].constraint_region_shape.dimensions.push_back(10.0f);
  goalA.motion_plan_request.goal_constraints.position_constraints[1].constraint_region_shape.dimensions.push_back(10.0f);

  goalA.motion_plan_request.goal_constraints.position_constraints[1].constraint_region_orientation.w = 1.0;
  goalA.motion_plan_request.goal_constraints.position_constraints[1].weight = 0.5;

  goalA.motion_plan_request.goal_constraints.set_orientation_constraints_size(1);
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].header.stamp = ros::Time::now();
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].header.frame_id = "base_link";
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].link_name = "r_wrist_roll_link";
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.x = 0.0;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.y = 0.0;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.z = 0.0;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.w = 1.0;

  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_roll_tolerance = 0.04;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_pitch_tolerance = 0.04;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_yaw_tolerance = 0.04;

  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].weight = 1.0;

  if (nh.ok())
  {
    bool finished_within_time = false;
    move_arm.sendGoal(goalA);
    finished_within_time = move_arm.waitForResult(ros::Duration(200.0));
    if (!finished_within_time)
    {
      move_arm.cancelGoal();
      ROS_INFO("Timed out achieving goal A");
    }
    else
    {
      actionlib::SimpleClientGoalState state = move_arm.getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if(success)
        ROS_INFO("Action finished: %s",state.toString().c_str());
      else
        ROS_INFO("Action failed: %s",state.toString().c_str());
    }
  }
  ros::shutdown();
}*/

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
 *  \author Sachin Chitta
 *********************************************************************/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_arm_msgs/MoveArmAction.h>

int main(int argc, char **argv){
  ros::init (argc, argv, "move_arm_joint_goal_test");
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<move_arm_msgs::MoveArmAction> move_arm("move_right_arm",true);

  move_arm.waitForServer();
  ROS_INFO("Connected to server");

  move_arm_msgs::MoveArmGoal goalA;

  goalA.motion_plan_request.group_name = "right_arm";
  goalA.motion_plan_request.num_planning_attempts = 1;
  goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  nh.param<std::string>("planner_id",goalA.motion_plan_request.planner_id,std::string(""));
  nh.param<std::string>("planner_service_name",goalA.planner_service_name,std::string("ompl_planning/plan_kinematic_path"));
  goalA.motion_plan_request.goal_constraints.set_position_constraints_size(1);
  goalA.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  goalA.motion_plan_request.goal_constraints.position_constraints[0].header.frame_id = "base_link";

  goalA.motion_plan_request.goal_constraints.position_constraints[0].link_name = "r_wrist_roll_link";
  //goalA.motion_plan_request.goal_constraints.position_constraints[0].position.x = 0.75;
  //goalA.motion_plan_request.goal_constraints.position_constraints[0].position.y = -0.188;
  //goalA.motion_plan_request.goal_constraints.position_constraints[0].position.z = 0;

  goalA.motion_plan_request.goal_constraints.position_constraints[0].position.x = 0.527;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].position.y = -0.258;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].position.z = 0.787;

  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.type = geometric_shapes_msgs::Shape::BOX;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.02);
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.02);
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.02);
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_orientation.w = 1.0;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].weight = 1.0;

  goalA.motion_plan_request.goal_constraints.set_orientation_constraints_size(1);
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].header.stamp = ros::Time::now();
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].header.frame_id = "base_link";
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].link_name = "r_wrist_roll_link";
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.x = 0.0;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.y = 0.0;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.z = 0.0;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.w = 1.0;

  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_roll_tolerance = 0.04;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_pitch_tolerance = 0.04;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_yaw_tolerance = 0.04;

  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].weight = 1.0;

  float minimumHeight = .82 + .10; // minimum height for ellbow

  motion_planning_msgs::PositionConstraint pc;

  pc.header.stamp = ros::Time::now();
  pc.header.frame_id = "torso_lift_link";
  pc.link_name = "r_elbow_flex_link";
  pc.position.x = 0;
  pc.position.y = 0;
  pc.position.z = 2.5 + (minimumHeight - 1.2); // assuming torso all the way up
  //pc.position.z = 2.5 + (minimumHeight - .808); // assuming torso all the way down
  pc.constraint_region_shape.type = geometric_shapes_msgs::Shape::BOX;
  pc.constraint_region_shape.dimensions.push_back(5.0);
  pc.constraint_region_shape.dimensions.push_back(5.0);
  pc.constraint_region_shape.dimensions.push_back(5.0);
  pc.constraint_region_orientation.w = 1.0;
  pc.weight = 1.0;

  goalA.motion_plan_request.goal_constraints.position_constraints.push_back(pc);


  if (nh.ok())
  {
    bool finished_within_time = false;
    move_arm.sendGoal(goalA);
    finished_within_time = move_arm.waitForResult(ros::Duration(200.0));
    if (!finished_within_time)
    {
      move_arm.cancelGoal();
      ROS_INFO("Timed out achieving goal A");
    }
    else
    {
      actionlib::SimpleClientGoalState state = move_arm.getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if(success)
        ROS_INFO("Action finished: %s",state.toString().c_str());
      else
        ROS_INFO("Action failed: %s",state.toString().c_str());
    }
  }
  ros::shutdown();
}
