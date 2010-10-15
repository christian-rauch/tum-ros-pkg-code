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


#include <ias_drawer_executive/Gripper.h>
#include <ias_drawer_executive/RobotArm.h>
#include <tf/transform_listener.h>

Gripper::Gripper(int side){

    side_ = side;
    //Initialize the client for the Action interface to the gripper controller
    //and tell the action client that we want to spin a thread by default
    //gripper_client_ = new GripperClient(side ? "l_gripper_controller/gripper_action" : "r_gripper_controller/gripper_action", true);
    gripper_client_ = new GripperClient(side ? "l_gripper_fingersensor_controller/gripper_action" : "r_gripper_fingersensor_controller/gripper_action", true);

    //wait for the gripper action server to come up
    while(ros::ok() && !gripper_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");
    }
  }

Gripper::~Gripper(){
    delete gripper_client_;
}

float Gripper::getAmountOpen(){
    tf::StampedTransform trans;
    if (side_ == 0)
       trans = RobotArm::getInstance(0)->getRelativeTransform("/r_gripper_l_finger_tip_link", "/r_gripper_r_finger_tip_link");
    else
       trans = RobotArm::getInstance(1)->getRelativeTransform("/l_gripper_l_finger_tip_link", "/l_gripper_r_finger_tip_link");

    return trans.getOrigin().length() - 0.032162;

}

//Open the gripper
void Gripper::open(float amount){
    pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command.position = 0.08 * amount;
    open.command.max_effort = -1.0;  // Do not limit effort (negative)

    gripper_client_->sendGoal(open);
    gripper_client_->waitForResult();
    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The gripper opened!");
    else
      ROS_INFO("The gripper failed to open.");
  }

  //Close the gripper
void Gripper::close(float amount){
    pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
    squeeze.command.position = 0.08 - (amount * 0.08);
    squeeze.command.max_effort = 50.0;  // Close gently

    gripper_client_->sendGoal(squeeze);
    gripper_client_->waitForResult();
    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The gripper closed!");
    else
      ROS_INFO("The gripper failed to close.");
  }

    //Close the gripper
void Gripper::closeHard(float amount){
    pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
    squeeze.command.position = 0.08 - (amount * 0.08);
    squeeze.command.max_effort = 150.0;  // Close gently

    gripper_client_->sendGoal(squeeze);
    gripper_client_->waitForResult();
    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The gripper closed!");
    else
      ROS_INFO("The gripper failed to close.");
  }

Gripper *Gripper::instance[] = {0,0};

Gripper* Gripper::getInstance(int side) {
    if (!instance[side]) {
        instance[side] = new Gripper(side);
    }
    return instance[side];
}


void Gripper::updatePressureZero(){
ros::service::call((side_==0) ? "/r_gripper_fingersensor_controller/update_zeros" : "/l_gripper_fingersensor_controller/update_zeros", serv);
}

void Gripper::closeCompliant(){
  ros::service::call((side_==0) ? "/r_reactive_grasp/compliant_close" : "/l_reactive_grasp/compliant_close", serv);
}
