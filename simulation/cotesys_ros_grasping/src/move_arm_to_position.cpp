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

struct GraspDef {
  std::string name;
  double ik_to_gripper_x_diff_;
  double ik_to_gripper_y_diff_;
  double ik_to_gripper_z_diff_;

  double end_effector_rot_x_;
  double end_effector_rot_y_;
  double end_effector_rot_z_;
  double end_effector_rot_w_;
};

class MoveArmToPositionServer
{
public:
  MoveArmToPositionServer();

  bool execute(const cotesys_ros_grasping::MoveArmToPositionGoalConstPtr& goal);

private:

  std::string left_arm_name_, right_arm_name_;
  std::string right_ik_link_, left_ik_link_;
  
  std::map<std::string, GraspDef> grasp_def_map_;
  
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

  if(!priv_nh_.hasParam("grasps")) {
    ROS_WARN_STREAM("No grasps loaded");
  } else {
    XmlRpc::XmlRpcValue grasps_xml;
    priv_nh_.getParam("grasps", grasps_xml);
    if(grasps_xml.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_WARN("grasps is not an array");
    } else if(grasps_xml.size() == 0) {
      ROS_WARN("No grasps specified in grasps yaml");
    } else {
      bool hasDefault = false;
      for(int i = 0; i < grasps_xml.size(); i++) {
        if(!grasps_xml[i].hasMember("name")) {
          ROS_WARN("Each grasp must have a name");
          continue;
        }
        GraspDef grasp;
        grasp.name = std::string(grasps_xml[i]["name"]);
        if(grasp.name == "default") {
          hasDefault = true;
        }
        ROS_INFO_STREAM("Adding grasp named " << grasp.name);
        if(grasps_xml[i].hasMember("ik_to_gripper_x_diff")) {
          grasp.ik_to_gripper_x_diff_ = grasps_xml[i]["ik_to_gripper_x_diff"];
        }
        if(grasps_xml[i].hasMember("ik_to_gripper_y_diff")) {
          grasp.ik_to_gripper_y_diff_ = grasps_xml[i]["ik_to_gripper_y_diff"];
        }
        if(grasps_xml[i].hasMember("ik_to_gripper_z_diff")) {
          grasp.ik_to_gripper_z_diff_ = grasps_xml[i]["ik_to_gripper_z_diff"];
        }
        if(grasps_xml[i].hasMember("end_effector_x_rot")) {
          grasp.end_effector_rot_x_ = grasps_xml[i]["end_effector_x_rot"];
        } else {
          ROS_WARN("no x rot");
        }
        if(grasps_xml[i].hasMember("end_effector_y_rot")) {
          grasp.end_effector_rot_y_ = grasps_xml[i]["end_effector_y_rot"];
        } else {
          ROS_WARN("no y rot");
        }
        if(grasps_xml[i].hasMember("end_effector_z_rot")) {
          grasp.end_effector_rot_z_ = grasps_xml[i]["end_effector_z_rot"];
        } else {
          ROS_WARN("no z rot");
        }
        if(grasps_xml[i].hasMember("end_effector_w_rot")) {
          grasp.end_effector_rot_w_ = grasps_xml[i]["end_effector_w_rot"];
        } else {
          ROS_WARN("no w rot");
        }
        ROS_INFO_STREAM("Quaternion is " << grasp.end_effector_rot_x_ << " " 
                        << grasp.end_effector_rot_y_ << " " 
                        << grasp.end_effector_rot_z_ << " " 
                        << grasp.end_effector_rot_w_);

        grasp_def_map_[grasp.name] = grasp;
      }
      if(!hasDefault) {
        ROS_WARN("No default grasp pose");
      }
    }
  }

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
    return false;
  }

  std::string grasp_name = req->grasp_name;
  if(grasp_def_map_.find(req->grasp_name) == grasp_def_map_.end()) {
    if(grasp_def_map_.find("default") != grasp_def_map_.end()) {
      ROS_DEBUG("Using default");
      grasp_name = "default";
    } else {
      ROS_INFO_STREAM("Don't have grasp named " << req->grasp_name << " and no default defined");
      return false;
    }
  }

  ROS_INFO_STREAM("Going to execute grasp named " << grasp_name);
  GraspDef& grasp = grasp_def_map_[grasp_name];

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
  goal.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time();
  goal.motion_plan_request.goal_constraints.position_constraints[0].header.frame_id = "base_link";
    
  goal.motion_plan_request.goal_constraints.position_constraints[0].link_name = ik_link_name;
  goal.motion_plan_request.goal_constraints.position_constraints[0].position.x = req->point.x-grasp.ik_to_gripper_x_diff_;
  goal.motion_plan_request.goal_constraints.position_constraints[0].position.y = req->point.y-grasp.ik_to_gripper_y_diff_;
  goal.motion_plan_request.goal_constraints.position_constraints[0].position.z = req->point.z-grasp.ik_to_gripper_z_diff_;
    
  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.type = geometric_shapes_msgs::Shape::BOX;
  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.20);
  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.20);
  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.20);

  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_orientation.w = 1.0;
  goal.motion_plan_request.goal_constraints.position_constraints[0].weight = 1.0;

  goal.motion_plan_request.goal_constraints.orientation_constraints.resize(1);
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].header.stamp = ros::Time();
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].header.frame_id = "base_link";
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].link_name = ik_link_name;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.x = grasp.end_effector_rot_x_;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.y = grasp.end_effector_rot_y_;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.z = grasp.end_effector_rot_z_;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.w = grasp.end_effector_rot_w_;
    
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_roll_tolerance = 0.15;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_pitch_tolerance = 0.15;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_yaw_tolerance = 0.15;

  goal.motion_plan_request.goal_constraints.orientation_constraints[0].weight = 1.0;

  //turning off collisions with the stupid camera frame
  motion_planning_msgs::CollisionOperation coll;
  coll.object1 = "r_forearm_cam";
  coll.object2 = coll.COLLISION_SET_ALL;
  coll.operation = coll.DISABLE;
  goal.motion_plan_request.ordered_collision_operations.collision_operations.push_back(coll);
  coll.object1 = "l_forearm_cam";
  goal.motion_plan_request.ordered_collision_operations.collision_operations.push_back(coll);
  if(req->arm_name == left_arm_name_) {
    coll.object1 = "l_forearm_link";
    coll.object2 = coll.COLLISION_SET_ATTACHED_OBJECTS;
    coll.operation = coll.ENABLE;
    goal.motion_plan_request.ordered_collision_operations.collision_operations.push_back(coll);
  } else {
    coll.object1 = "r_forearm_link";
    coll.object2 = coll.COLLISION_SET_ATTACHED_OBJECTS;
    coll.operation = coll.ENABLE;
    goal.motion_plan_request.ordered_collision_operations.collision_operations.push_back(coll);
  }

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
