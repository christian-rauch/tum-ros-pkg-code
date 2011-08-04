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
#include <string>

Gripper::Gripper(int side){

    side_ = side;

    std::string sideletter((side_ == 0) ? "r" : "l");
    //Initialize the client for the Action interface to the gripper controller
    //and tell the action client that we want to spin a thread by default
    //gripper_client_ = new GripperClient(side ? "l_gripper_controller/gripper_action" : "r_gripper_controller/gripper_action", true);

    gripper_client_ = new GripperClient(side ? "l_gripper_sensor_controller/gripper_action" : "r_gripper_sensor_controller/gripper_action", true);
    grab_ = new GrabAC("/"+ sideletter + "_gripper_sensor_controller/grab", true);
    contact_ = new FindContactAC("/"+ sideletter + "_gripper_sensor_controller/find_contact", true);

    //wait for the gripper action server to come up
    while(ros::ok() && !gripper_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO((side_ == 0) ? "Waiting for the r_gripper_fingersensor_controller/gripper_action action server to come up":  "Waiting for the l_gripper_fingersensor_controller/gripper_action action server to come up");
    }
  }

Gripper::~Gripper(){
    delete gripper_client_;
}

float Gripper::getAmountOpen(){
    tf::Stamped<tf::Pose> trans;
    if (side_ == 0)
       trans = RobotArm::getInstance(0)->getRelativeTransform("/r_gripper_l_finger_tip_link", "/r_gripper_r_finger_tip_link");
    else
       trans = RobotArm::getInstance(1)->getRelativeTransform("/l_gripper_l_finger_tip_link", "/l_gripper_r_finger_tip_link");

    return trans.getOrigin().length() - .029245 ; // offset of links to surface' - 0.032162;
}

//Open the gripper
void Gripper::open(float amount){
    pr2_controllers_msgs::Pr2GripperCommandGoal open;
    //open.command.position = 0.085 * amount;
    open.command.position = amount;
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
    //squeeze.command.position = 0.085 - (amount * 0.085);
    squeeze.command.position = amount;
    squeeze.command.max_effort = 150.0;  // Close hard

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
    //squeeze.command.position = 0.085 - (amount * 0.085);
    squeeze.command.position = amount;
    squeeze.command.max_effort = -1.0;  // Close hard = do not limit force

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
  //ros::service::call((side_==0) ? "/r_gripper_sensor_controller/update_zeros" : "/l_gripper_sensor_controller/update_zeros", serv);
}

void Gripper::closeCompliant(float gain){
  ros::service::call((side_==0) ? "/r_reactive_grasp/compliant_close" : "/l_reactive_grasp/compliant_close", serv);
  //typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperGrabAction>    GrabAC;
  //GrabAC grab("/"+ side + "_gripper_sensor_controller/grab", true);

  /*while(!contact_->waitForServer())
    {
      ROS_INFO("Waiting for the /%s_gripper_sensor_controller/findcontact to come up", ((side_ == 0) ? "r" : "l"));
    }
  pr2_gripper_sensor_msgs::PR2GripperFindContactGoal fc_goal;
  fc_goal.command.contact_conditions = 0; // 0 both 1 left 2 right 3 either
  fc_goal.command.zero_fingertip_sensors = true;
  contact_->sendGoal(fc_goal);
  contact_->waitForResult(ros::Duration(5.0));
  if (contact_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("/%s_gripper_sensor_controller/findcontact SUCCEEDED", ((side_ == 0) ? "r" : "l"));
  else
    ROS_ERROR("/%s_gripper_sensor_controller/findcontact FAILED", ((side_ == 0) ? "r" : "l"));*/

  /*while(!grab_->waitForServer())
    {
      ROS_INFO("Waiting for the /%s_gripper_sensor_controller/grab to come up", ((side_ == 0) ? "r" : "l"));
    }
  pr2_gripper_sensor_msgs::PR2GripperGrabGoal grab_goal;
  grab_goal.command.hardness_gain=gain;
  ROS_INFO("sending grab goal");
  grab_->sendGoal(grab_goal);
  grab_->waitForResult(ros::Duration(5.0));
  if (grab_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("/%s_gripper_sensor_controller/grab SUCCEEDED", ((side_ == 0) ? "r" : "l"));
  else
    ROS_ERROR("/%s_gripper_sensor_controller/grab FAILED", ((side_ == 0) ? "r" : "l"));*/
  //close();
}

