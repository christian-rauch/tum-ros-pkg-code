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

#include <ias_drawer_executive/RobotArm.h>
#include <ias_drawer_executive/Pressure.h>
//#include <ias_drawer_executive/Poses.h>
#include <ias_drawer_executive/RobotDriver.h>
#include <boost/thread.hpp>
#include <visualization_msgs/Marker.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionFK.h>


void RobotArm::jointStateCallback(const  pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
    mutex_.lock();
    for (int i = 0; i < 7; ++i)
    {
        joint_names[i] = msg->joint_names[i].c_str();
        //ROS_INFO("JOINT NAME msg %s", msg->joint_names[i].c_str());
        //ROS_INFO("JOINT NAME %s", joint_names[i].c_str());

        jointState[i] = msg->actual.positions[i];
        jointStateDes[i] = msg->desired.positions[i];
        jointStateErr[i] = msg->error.positions[i];
    }
    haveJointState = true;
    mutex_.unlock();
}

//! Initialize the action client and wait for action server to come up
RobotArm::RobotArm(int side)
{
    if (!listener_)
        listener_ = new tf::TransformListener();


    retries = 0;

    raise_elbow = false;
    preset_angle = 1.4;

    tucked = false;
    side_ = side;
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient((side == 0) ? "r_arm_controller/joint_trajectory_action" : "l_arm_controller/joint_trajectory_action", true);
    // wait for action server to come up
    while (ros::ok() & !traj_client_->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the joint_trajectory_action server");
    }

    ac_ = new actionlib::SimpleActionClient<pr2_common_action_msgs::ArmMoveIKAction>((side == 0) ? "r_arm_ik" : "l_arm_ik", true);

    while (ros::ok() && !ac_->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move ik server %s", (side == 0) ? "r_arm_ik" : "l_arm_ik");
    }

    for (int i = 0; i < 7; ++i)
        jointState[i] = 0.0f;

    haveJointState = false;
    jointStateSubscriber_ = n_.subscribe((side == 0) ? "/r_arm_controller/state" : "/l_arm_controller/state", 1, &RobotArm::jointStateCallback,this);

    //ik_client = n_.serviceClient<kinematics_msgs::GetPositionIK>((side == 0) ? "/pr2_right_arm_kinematics/get_ik" : "/pr2_left_arm_kinematics/get_ik" , true);
    ik_client = n_.serviceClient<kinematics_msgs::GetPositionIK>((side == 0) ? "/pr2_right_arm_kinematics/get_ik" : "/pr2_left_arm_kinematics/get_ik" , true);

    time_to_target = 1;

    evil_switch =false;

    ros::service::waitForService((side_==0) ? "pr2_right_arm_kinematics/get_fk_solver_info" : "pr2_left_arm_kinematics/get_fk_solver_info");
    ros::service::waitForService((side_==0) ? "pr2_right_arm_kinematics/get_fk" : "pr2_left_arm_kinematics/get_fk");

    query_client = n_.serviceClient<kinematics_msgs::GetKinematicSolverInfo>((side_==0) ? "pr2_right_arm_kinematics/get_fk_solver_info" : "pr2_left_arm_kinematics/get_fk_solver_info");
    fk_client = n_.serviceClient<kinematics_msgs::GetPositionFK>((side_==0)? "pr2_right_arm_kinematics/get_fk" : "pr2_left_arm_kinematics/get_fk");

    excludeBaseProjectionFromWorkspace = false;
}

//! Clean up the action client
RobotArm::~RobotArm()
{
    ac_->cancelAllGoals();
    traj_client_->cancelAllGoals();
    delete traj_client_;
}

RobotArm* RobotArm::getInstance(int side)
{
    if (!instance[side])
        instance[side] = new RobotArm(side);
    return instance[side];
}

void RobotArm::getJointState(double state[])
{
    haveJointState = false; //make sure to get a new state
    ros::Rate rate(20);
    while (!haveJointState)
    {
        rate.sleep();
        ros::spinOnce();
    }
    mutex_.lock();
    for (int i = 0; i < 7; ++i)
        state[i]=jointState[i];
    mutex_.unlock();
}

void RobotArm::getJointStateDes(double state[])
{
    ros::Rate rate(20);
    while (!haveJointState)
    {
        rate.sleep();
        ros::spinOnce();
    }
    mutex_.lock();
    for (int i = 0; i < 7; ++i)
        state[i]=jointStateDes[i];
    mutex_.unlock();
}

void RobotArm::getJointStateErr(double state[])
{
    ros::Rate rate(20);
    while (!haveJointState)
    {
        rate.sleep();
        ros::spinOnce();
    }
    mutex_.lock();
    for (int i = 0; i < 7; ++i)
        state[i]=jointStateErr[i];
    mutex_.unlock();
}



tf::Stamped<tf::Pose> RobotArm::getRelativeTransform(const char source_frameid[], const char target_frameid[])
{
    tf::StampedTransform transform;

    listener_->waitForTransform(source_frameid, target_frameid,
                                ros::Time(), ros::Duration(10.0));

    try
    {
        listener_->lookupTransform(std::string(source_frameid), std::string(target_frameid), ros::Time(), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("getTransformIn %s",ex.what());
    }

    tf::Stamped<tf::Pose> ret;
    ret.frame_id_ = transform.frame_id_;
    ret.stamp_ = transform.stamp_;
    ret.setOrigin(transform.getOrigin());
    ret.setRotation(transform.getRotation());

    return ret;
}

tf::Stamped<tf::Pose> RobotArm::getTransformIn(const char target_frame[], tf::Stamped<tf::Pose>src)
{
    tf::Stamped<tf::Pose> transform;

    listener_->waitForTransform(src.frame_id_, target_frame,
                                ros::Time(0), ros::Duration(10.0));
    try
    {
        listener_->transformPose("base_link", src, transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("getTransformIn %s",ex.what());
    }
    tf::Stamped<tf::Pose> ret;
    ret.frame_id_ = transform.frame_id_;
    ret.stamp_ = transform.stamp_;
    ret.setOrigin(transform.getOrigin());
    ret.setRotation(transform.getRotation());

    return ret;
}

tf::TransformListener *RobotArm::listener_=0;

tf::Stamped<tf::Pose> RobotArm::scaleStampedPose(const tf::Stamped<tf::Pose> &in, double scale)
{
    tf::Stamped<tf::Pose>  ret;
    ret.setRotation(btQuaternion::getIdentity().slerp(in.getRotation(), scale));
    ret.setOrigin(in.getOrigin() * scale);
    return ret;
}

tf::Stamped<tf::Pose> RobotArm::scaleStampedTransform(const tf::Stamped<tf::Pose> &in, double scale)
{
    tf::Stamped<tf::Pose>  ret;
    ret.setRotation(btQuaternion::getIdentity().slerp(in.getRotation(), scale));
    ret.setOrigin(in.getOrigin() * scale);
    return ret;
}

tf::Stamped<tf::Pose> RobotArm::getPoseIn(const char target_frame[], tf::Stamped<tf::Pose>src)
{

    if (!listener_)
        listener_ = new tf::TransformListener();

    // tf::Stamped<tf::Pose>
    tf::Stamped<tf::Pose> transform;
    //this shouldnt be here TODO
    src.stamp_ = ros::Time(0);

    listener_->waitForTransform(src.frame_id_, target_frame,
                                ros::Time(0), ros::Duration(30.0));
    bool transformOk = false;
    while (!transformOk)
    {
        try
        {
            transformOk = true;
            listener_->transformPose(target_frame, src, transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("getPoseIn %s",ex.what());
            // dirty:
            src.stamp_ = ros::Time(0);
            transformOk = false;
        }
        ros::spinOnce();
    }
    return transform;
}

//run inverse kinematics on a PoseStamped (7-dof pose
//(position + quaternion orientation) + header specifying the
//frame of the pose)
//tries to stay close to double start_angles[7]
//returns the solution angles in double solution[7]
bool RobotArm::run_ik(geometry_msgs::PoseStamped pose, double start_angles[7],
                      double solution[7], std::string link_name)
{

    if (excludeBaseProjectionFromWorkspace)
    {

        //bool poseInsideFootprint = false;
        float padding = 0.05;
        float x = pose.pose.position.x;
        float y = pose.pose.position.y;
        // does target pose lie inside projection of base footprint?
        if ((x < .325 + padding) && (x > -.325 - padding) && (y < .325 + padding) && (y > -.325 - padding))
        {
            //poseInsideFootprint = true;
            return false;
        }
    }


    kinematics_msgs::GetPositionIK::Request  ik_request;
    kinematics_msgs::GetPositionIK::Response ik_response;

    ik_request.timeout = ros::Duration(5.0);
    if (side_ == 0)
    {
        ik_request.ik_request.ik_seed_state.joint_state.name.push_back("r_shoulder_pan_joint");
        ik_request.ik_request.ik_seed_state.joint_state.name.push_back("r_shoulder_lift_joint");
        ik_request.ik_request.ik_seed_state.joint_state.name.push_back("r_upper_arm_roll_joint");
        ik_request.ik_request.ik_seed_state.joint_state.name.push_back("r_elbow_flex_joint");
        ik_request.ik_request.ik_seed_state.joint_state.name.push_back("r_forearm_roll_joint");
        ik_request.ik_request.ik_seed_state.joint_state.name.push_back("r_wrist_flex_joint");
        ik_request.ik_request.ik_seed_state.joint_state.name.push_back("r_wrist_roll_joint");

        ik_request.ik_request.robot_state.joint_state.name.push_back("r_shoulder_pan_joint");
        ik_request.ik_request.robot_state.joint_state.name.push_back("r_shoulder_lift_joint");
        ik_request.ik_request.robot_state.joint_state.name.push_back("r_upper_arm_roll_joint");
        ik_request.ik_request.robot_state.joint_state.name.push_back("r_elbow_flex_joint");
        ik_request.ik_request.robot_state.joint_state.name.push_back("r_forearm_roll_joint");
        ik_request.ik_request.robot_state.joint_state.name.push_back("r_wrist_flex_joint");
        ik_request.ik_request.robot_state.joint_state.name.push_back("r_wrist_roll_joint");
    }
    else
    {
        ik_request.ik_request.ik_seed_state.joint_state.name.push_back("l_shoulder_pan_joint");
        ik_request.ik_request.ik_seed_state.joint_state.name.push_back("l_shoulder_lift_joint");
        ik_request.ik_request.ik_seed_state.joint_state.name.push_back("l_upper_arm_roll_joint");
        ik_request.ik_request.ik_seed_state.joint_state.name.push_back("l_elbow_flex_joint");
        ik_request.ik_request.ik_seed_state.joint_state.name.push_back("l_forearm_roll_joint");
        ik_request.ik_request.ik_seed_state.joint_state.name.push_back("l_wrist_flex_joint");
        ik_request.ik_request.ik_seed_state.joint_state.name.push_back("l_wrist_roll_joint");

        ik_request.ik_request.robot_state.joint_state.name.push_back("l_shoulder_pan_joint");
        ik_request.ik_request.robot_state.joint_state.name.push_back("l_shoulder_lift_joint");
        ik_request.ik_request.robot_state.joint_state.name.push_back("l_upper_arm_roll_joint");
        ik_request.ik_request.robot_state.joint_state.name.push_back("l_elbow_flex_joint");
        ik_request.ik_request.robot_state.joint_state.name.push_back("l_forearm_roll_joint");
        ik_request.ik_request.robot_state.joint_state.name.push_back("l_wrist_flex_joint");
        ik_request.ik_request.robot_state.joint_state.name.push_back("l_wrist_roll_joint");
    }

    ik_request.ik_request.ik_link_name = link_name;

    ik_request.ik_request.pose_stamped = pose;
    ik_request.ik_request.ik_seed_state.joint_state.position.resize(7);
    ik_request.ik_request.robot_state.joint_state.position.resize(7);

    for (int i=0; i<7; i++)
    {
        ik_request.ik_request.ik_seed_state.joint_state.position[i] = start_angles[i];
        ik_request.ik_request.robot_state.joint_state.position[i] = start_angles[i];
    }

    //ROS_INFO("request pose: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);

    bool ik_service_call = ik_client.call(ik_request,ik_response);
    if (!ik_service_call)
    {
        ROS_ERROR("IK service call failed! is ik service running? /pr2_right_arm_kinematics/get_ik");
        return 0;
    }
    if (ik_response.error_code.val == ik_response.error_code.SUCCESS)
    {
        for (int i=0; i<7; i++)
        {
            solution[i] = ik_response.solution.joint_state.position[i];
        }
        //ROS_INFO("solution angles: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f",solution[0],solution[1], solution[2],solution[3],solution[4],solution[5],solution[6]);
        //ROS_INFO("IK service call succeeded");
        return 1;
    }
    //ROS_INFO("IK service call error code: %d", ik_response.error_code.val);
    return 0;
}


//! Sends the command to start a given trajectory
void RobotArm::startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal, bool wait)
{

    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now(); //; + ros::Duration(0.1);

    traj_client_->sendGoal(goal);

    if (wait)
    {

        try
        {
            traj_client_->waitForResult();
        }
        catch ( boost::thread_interrupted ti )
        {
            traj_client_->cancelAllGoals();
            ROS_ERROR("RobotArm startTrajectory side %i Interrupted: Thread killed. Cancelling all arm ac goals", this->side_);
            throw ti;
        }
    }

    //if (traj_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//      ROS_INFO("traj action succ");
    //else
//      ROS_INFO("traj action failed");
}

pr2_controllers_msgs::JointTrajectoryGoal RobotArm::goalTraj(float *poseA, float dur)
{

    //ROS_INFO("JOINT TRAJECTORY CONTROL %s ARM", side_ == 0 ? "right" : "left");

    //our goal variable
    pr2_controllers_msgs::JointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    if (side_==0)
    {
        goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
        goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
        goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
        goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
        goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
        goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
        goal.trajectory.joint_names.push_back("r_wrist_roll_joint");
    }
    else
    {
        goal.trajectory.joint_names.push_back("l_shoulder_pan_joint");
        goal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
        goal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
        goal.trajectory.joint_names.push_back("l_elbow_flex_joint");
        goal.trajectory.joint_names.push_back("l_forearm_roll_joint");
        goal.trajectory.joint_names.push_back("l_wrist_flex_joint");
        goal.trajectory.joint_names.push_back("l_wrist_roll_joint");
    }

    // We will have two waypoints in this goal trajectory
    goal.trajectory.points.resize(1);

    // First trajectory point
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
        goal.trajectory.points[ind].positions[j] = poseA[j];
        goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 1 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(dur);

    //we are done; return the goal
    return goal;
}

pr2_controllers_msgs::JointTrajectoryGoal RobotArm::goalTraj(float *poseA, float *vel)
{

    //ROS_INFO("JOINT TRAJECTORY CONTROL %s ARM", side_ == 0 ? "right" : "left");

    //our goal variable
    pr2_controllers_msgs::JointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    if (side_==0)
    {
        goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
        goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
        goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
        goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
        goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
        goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
        goal.trajectory.joint_names.push_back("r_wrist_roll_joint");
    }
    else
    {
        goal.trajectory.joint_names.push_back("l_shoulder_pan_joint");
        goal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
        goal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
        goal.trajectory.joint_names.push_back("l_elbow_flex_joint");
        goal.trajectory.joint_names.push_back("l_forearm_roll_joint");
        goal.trajectory.joint_names.push_back("l_wrist_flex_joint");
        goal.trajectory.joint_names.push_back("l_wrist_roll_joint");
    }

    // We will have two waypoints in this goal trajectory
    goal.trajectory.points.resize(1);

    // First trajectory point

    int ind = 0;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
        goal.trajectory.points[ind].positions[j] = poseA[j];
        goal.trajectory.points[ind].velocities[j] = vel[j];
    }
    // To be reached 1 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(0.25);

    //we are done; return the goal
    return goal;
}


//! Generates a simple trajectory with two waypoints, used as an example
/*! Note that this trajectory contains two waypoints, joined together
    as a single trajectory. Alternatively, each of these waypoints could
    be in its own trajectory - a trajectory can have one or more waypoints
    depending on the desired application.
*/
pr2_controllers_msgs::JointTrajectoryGoal RobotArm::lookAtMarker(float *poseA, float *poseB)
{

    ROS_INFO("JOINT TRAJECTORY CONTROL %s ARM", side_ == 0 ? "right" : "left");

    //tucked = (poseB == Poses::tuckPose);
    //our goal variable
    pr2_controllers_msgs::JointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    if (side_==0)
    {
        goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
        goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
        goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
        goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
        goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
        goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
        goal.trajectory.joint_names.push_back("r_wrist_roll_joint");
    }
    else
    {
        goal.trajectory.joint_names.push_back("l_shoulder_pan_joint");
        goal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
        goal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
        goal.trajectory.joint_names.push_back("l_elbow_flex_joint");
        goal.trajectory.joint_names.push_back("l_forearm_roll_joint");
        goal.trajectory.joint_names.push_back("l_wrist_flex_joint");
        goal.trajectory.joint_names.push_back("l_wrist_roll_joint");
    }

    // We will have two waypoints in this goal trajectory
    goal.trajectory.points.resize(2);

    // First trajectory point

    int ind = 0;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
        goal.trajectory.points[ind].positions[j] = poseA[j];
        goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 1 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(2.0);

    // Second trajectory point

    ind += 1;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
        goal.trajectory.points[ind].positions[j] = poseB[j];
        goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 2 seconds after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(4.0);

    //we are done; return the goal
    return goal;
}

// Returns the current state of the action
actionlib::SimpleClientGoalState RobotArm::getState()
{
    return traj_client_->getState();
}

void RobotArm::printPose(tf::Stamped<tf::Pose> &toolTargetPose)
{
    ROS_INFO("%f %f %f %f %f %f %f\n",
             toolTargetPose.getOrigin().x(),  toolTargetPose.getOrigin().y(),  toolTargetPose.getOrigin().z(),toolTargetPose.getRotation().x(),
             toolTargetPose.getRotation().y(),toolTargetPose.getRotation().z(),toolTargetPose.getRotation().w());
    ROS_INFO("Frame: %s", toolTargetPose.frame_id_.c_str());
}

/*
void RobotArm::getToolPose(tf::Stamped<tf::Pose> &marker)
{
    //tf::TransformListener listener;
    ros::Rate rate(100.0);

    tf::StampedTransform transform;
    transform.setOrigin(btVector3(0,0,0));
    bool transformOk = false;
    while (ros::ok() && ((!transformOk) || (transform.getOrigin().z() < 0.01)))
    {
        transformOk = true;
        try
        {
            listener_->lookupTransform("base_link", (side_==0) ? "r_gripper_tool_frame" : "l_gripper_tool_frame",ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("getToolPose %s",ex.what());
            transformOk = false;
        }
        rate.sleep();
    }
    marker.frame_id_ = transform.frame_id_;
    marker.setOrigin(transform.getOrigin());
    marker.setRotation(transform.getRotation());
    marker.stamp_ = transform.stamp_;
}
*/



tf::Stamped<tf::Pose> RobotArm::getTransform(const char baseframe[],  const char toolframe[])
{
    //tf::TransformListener listener;
    ros::Rate rate(100.0);

    tf::StampedTransform transform;
    bool transformOk = false;
    while (ros::ok() && (!transformOk))
    {
        transformOk = true;
        try
        {
            listener_->lookupTransform(baseframe, toolframe,ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("getTransform %s",ex.what());
            transformOk = false;
        }
        rate.sleep();
    }
    tf::Stamped<tf::Pose> ret;
    ret.frame_id_ = transform.frame_id_;
    ret.stamp_ = transform.stamp_;
    ret.setOrigin(transform.getOrigin());
    ret.setRotation(transform.getRotation());

    return ret;
}


tf::Stamped<tf::Pose> RobotArm::getToolPose(const char frame[])
{
    //tf::TransformListener listener;
    ros::Rate rate(100.0);

    tf::StampedTransform transform;
    bool transformOk = false;
    while (ros::ok() && (!transformOk))
    {
        transformOk = true;
        try
        {
            listener_->lookupTransform(frame, (side_==0) ? "r_gripper_tool_frame" : "l_gripper_tool_frame" ,ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("getToolPose s f %s",ex.what());
            transformOk = false;
        }
        rate.sleep();
    }
    tf::Stamped<tf::Pose> ret;
    ret.frame_id_ = transform.frame_id_;
    ret.stamp_ = transform.stamp_;
    ret.setOrigin(transform.getOrigin());
    ret.setRotation(transform.getRotation());

    return ret;
}



void RobotArm::getToolPose(tf::Stamped<tf::Pose> &marker, const char frame[])
{
    //tf::TransformListener listener;
    ros::Rate rate(100.0);

    tf::StampedTransform transform;
    bool transformOk = false;
    while (ros::ok() && (!transformOk))
    {
        transformOk = true;
        try
        {
            listener_->lookupTransform(frame, (side_==0) ? "r_gripper_tool_frame" : "l_gripper_tool_frame" ,ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("getToolPose s f %s",ex.what());
            transformOk = false;
        }
        rate.sleep();
    }
    marker.frame_id_ = transform.frame_id_;
    marker.setOrigin(transform.getOrigin());
    marker.setRotation(transform.getRotation());
    marker.stamp_ = transform.stamp_;
}


void RobotArm::getWristPose(tf::Stamped<tf::Pose> &marker, const char frame[])
{
    //tf::TransformListener listener;
    ros::Rate rate(100.0);

    tf::StampedTransform transform;
    bool transformOk = false;
    while (ros::ok() && (!transformOk))
    {
        transformOk = true;
        try
        {
            listener_->lookupTransform(frame, (side_==0) ? "r_wrist_roll_link" : "l_wrist_roll_link" ,ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("getWristPose s f %s",ex.what());
            transformOk = false;
        }
        rate.sleep();
    }
    marker.frame_id_ = transform.frame_id_;
    marker.setOrigin(transform.getOrigin());
    marker.setRotation(transform.getRotation());
    marker.stamp_ = transform.stamp_;
}



tf::Stamped<tf::Pose> RobotArm::rotateAroundBaseAxis(tf::Stamped<tf::Pose> toolPose, float r_x,float r_y,float r_z)
{
    tf::Stamped<tf::Pose> toolRotPlus;
    toolRotPlus.setOrigin(btVector3(0,0,0));
    toolRotPlus.setRotation(btQuaternion(r_x,r_y,r_z));
    btVector3 orig = toolPose.getOrigin();
    toolPose.setOrigin(btVector3(0,0,0));
    toolRotPlus *= toolPose;
    toolPose.setOrigin(orig);
    toolPose.setRotation(toolRotPlus.getRotation());
    return toolPose;
}

tf::Stamped<tf::Pose> RobotArm::rotateAroundToolframeAxis(tf::Stamped<tf::Pose> toolPose, float r_x,float r_y,float r_z)
{
    tf::Stamped<tf::Pose> toolRotPlus;
    toolRotPlus.setOrigin(btVector3(0,0,0));
    toolRotPlus.setRotation(btQuaternion(r_x,r_y,r_z));
    btVector3 orig = toolPose.getOrigin();
    toolPose *= toolRotPlus;
    return toolPose;
}


tf::Stamped<tf::Pose> RobotArm::rotateAroundPose(tf::Stamped<tf::Pose> toolPose, tf::Stamped<tf::Pose> pivot, float r_x, float r_y, float r_z)
{
    btTransform curr = toolPose;
    btTransform pivo = pivot;

    curr = pivo.inverseTimes(curr);

    btQuaternion qa;
    qa.setEulerZYX(r_z,r_y,r_x);

    btTransform rot;
    rot.setOrigin(btVector3(0,0,0));
    rot.setRotation(qa);
    curr = rot * curr;
    curr = pivo * curr;

    tf::Stamped<tf::Pose> act;
    act.frame_id_ = toolPose.frame_id_;
    act.setOrigin(curr.getOrigin());
    act.setRotation(curr.getRotation());

    return act;
}


tf::Stamped<tf::Pose> RobotArm::rotateAroundPose(tf::Stamped<tf::Pose> toolPose, tf::Stamped<tf::Pose> pivot, btQuaternion qa)
{
    btTransform curr = toolPose;
    btTransform pivo = pivot;

    curr = pivo.inverseTimes(curr);

    btTransform rot;
    rot.setOrigin(btVector3(0,0,0));
    rot.setRotation(qa);
    curr = rot * curr;
    curr = pivo * curr;

    tf::Stamped<tf::Pose> act;
    act.frame_id_ = toolPose.frame_id_;
    act.setOrigin(curr.getOrigin());
    act.setRotation(curr.getRotation());

    return act;
}


bool RobotArm::rotate_toolframe_ik(float r_x, float r_y, float r_z)
{
    //ROS_INFO("ROTATING TOOLFRAME %f %f %f", r_x, r_y, r_z);
    tf::Stamped<tf::Pose> toolTargetPose;
    tf::Stamped<tf::Pose> trans;
    getToolPose(trans);
    toolTargetPose.setOrigin(trans.getOrigin());
    toolTargetPose.setRotation(trans.getRotation());

    tf::Stamped<tf::Pose> wrist2tool;

    if (side_==0)
        wrist2tool.frame_id_ = "r_wrist_roll_link";
    else
        wrist2tool.frame_id_ = "l_wrist_roll_link";
    wrist2tool.stamp_ = ros::Time();
    wrist2tool.setOrigin(btVector3(-.18,0,0));
    wrist2tool.setRotation(btQuaternion(0,0,0,1));

    tf::Stamped<tf::Pose> toolTargetPoseRotPlus = toolTargetPose;
    toolTargetPose *= wrist2tool;

    // printPose(toolTargetPose);

    // init with some yaw pitch roll
    tf::Stamped<tf::Pose> toolRotPlus;
    toolRotPlus.setOrigin(btVector3(0,0,0));
    toolRotPlus.setRotation(btQuaternion(r_x,r_y,r_z));

    toolTargetPoseRotPlus*=toolRotPlus;
    toolTargetPoseRotPlus*=wrist2tool;

    return move_ik(toolTargetPoseRotPlus.getOrigin().x(),toolTargetPoseRotPlus.getOrigin().y(),toolTargetPoseRotPlus.getOrigin().z(),
                   toolTargetPoseRotPlus.getRotation().x(),toolTargetPoseRotPlus.getRotation().y(),toolTargetPoseRotPlus.getRotation().z(),toolTargetPoseRotPlus.getRotation().w(),0.1);

    return true;
}


bool RobotArm::move_toolframe_ik_pose(tf::Stamped<tf::Pose> toolTargetPose)
{
    tf::Stamped<tf::Pose> toolTargetPoseWristBase;

    toolTargetPoseWristBase = getPoseIn("base_link", toolTargetPose);
    toolTargetPoseWristBase = tool2wrist(toolTargetPoseWristBase);

    //printPose(toolTargetPoseWristBase);

    return move_ik(toolTargetPoseWristBase.getOrigin().x(),toolTargetPoseWristBase.getOrigin().y(),toolTargetPoseWristBase.getOrigin().z(),
                   toolTargetPoseWristBase.getRotation().x(),toolTargetPoseWristBase.getRotation().y(),toolTargetPoseWristBase.getRotation().z(),toolTargetPoseWristBase.getRotation().w(),time_to_target);


    return true;
}

//moves the toolframe to the given position
bool RobotArm::move_toolframe_ik(float x, float y, float z, float ox, float oy, float oz, float ow)
{
    tf::Stamped<tf::Pose> toolTargetPose;

    toolTargetPose.frame_id_ = "base_link";
    toolTargetPose.stamp_ = ros::Time();
    toolTargetPose.setOrigin(btVector3( x,y,z));
    toolTargetPose.setRotation(btQuaternion(ox,oy,oz,ow));

    return move_toolframe_ik_pose(toolTargetPose);

    /*tf::Stamped<tf::Pose> wrist2tool;

    if (side_==0)
        wrist2tool.frame_id_ = "r_wrist_roll_link";
    else
        wrist2tool.frame_id_ = "l_wrist_roll_link";
    wrist2tool.stamp_ = ros::Time();
    wrist2tool.setOrigin(btVector3(-.18,0,0));
    wrist2tool.setRotation(btQuaternion(0,0,0,1));

    toolTargetPose *= wrist2tool;

    printPose(toolTargetPose);

    return move_ik(toolTargetPose.getOrigin().x(),toolTargetPose.getOrigin().y(),toolTargetPose.getOrigin().z(),
                   toolTargetPose.getRotation().x(),toolTargetPose.getRotation().y(),toolTargetPose.getRotation().z(),toolTargetPose.getRotation().w(),time_to_target);


    return true;*/
}


void RobotArm::stabilize_grip()
{
    RobotArm *arm = this;

    float oldtime = time_to_target;
    time_to_target = 1.2;

    Pressure *p = Pressure::getInstance(side_);

    ros::Rate rate(25.0);
    double pGain = 0.02; //actually seomthing p-gainy and not an increment
    double limit = 0.25;
    long cnt_ok = 0;
    long cnt_bad = 0;
    double lastval = 0;
    double lastdiff = 100;
    double lastchange = 0; // what did we do
    double lasteffect = 0; // how did it change the pressure center difference?
    while (ros::ok() && (cnt_ok < 5) && (cnt_bad < 100))
    {
        rate.sleep();
        ros::spinOnce();
        float r[2],l[2];
        p->getCenter(r,l);

        float d[2];
        d[0] = r[0] - l[0];
        d[1] = r[1] - l[1];

        ROS_INFO("SIDE %i CENTERS %f %f , %f %f DIFFS %f %f", side_, r[0], r[1], l[0], l[1], d[0], d[1]);

        lasteffect = d[1] - lastdiff;
        if (lastchange != 0)
        {
            double optpGain = fabs(pGain * (d[1] / lasteffect)); // for now only positive gains, although there are some strange cases where the effect seems reversed
            if (optpGain > 0.1)
                optpGain = 0.1;
            if (optpGain < -0.1)
                optpGain = -0.1;
            ROS_INFO("change %f effect %f change in pGains %f, new pGain %f", lastchange, lasteffect, lastchange / pGain,optpGain);
            pGain = optpGain;
        }

        float k = d[1];
        double current = 0;
        if (k > 1)
            k = 1;
        if (k < -1)
            k = -1;

        lastchange = 0;
        if (fabs(d[1]) > limit)
        {
            if (d[1] > 0)
                ROS_INFO("pos DIFF %f %f limit %f k %f", d[0], d[1], limit, k);
            else
                ROS_INFO("neg DIFF %f %f limit %f k %f", d[0], d[1], limit, k);

            arm->rotate_toolframe_ik(pGain * k,0,0);
            lastchange = pGain * k;
            ros::spinOnce();
            current = +1;
        }

        if (fabs(d[1]) > fabs(lastdiff))
        {

            ROS_INFO("SOMETHINGS WRONG, GET OUT OF STABIL %f %f", fabs(d[1]), fabs(lastdiff));
            cnt_bad = 1000;
        }
        lastdiff = d[1];

        if (fabs(current - lastval) > 1.5)
        {
            ROS_INFO("LET ME OUT limit %f", limit);
            limit *= 2;
            pGain *= 0.5;
        }

        if (fabs(d[1]) < limit)
            cnt_ok++;
        else
            cnt_bad++;
    }

    time_to_target = oldtime;
}


// rosrun tf tf_echo /base_link /r_wrist_roll_link -> position
bool RobotArm::move_ik(float x, float y, float z, float ox, float oy, float oz, float ow, float time)
{

    ROS_INFO("%s arm move ik action started %f %f %f %f %f %f %f (time %f)",((side_==0) ? "right" : "left"), x, y, z, ox, oy, oz, ow, time);
    //actionlib::SimpleActionClient<pr2_common_action_msgs::ArmMoveIKAction> ac("r_arm_ik", true);

    //ac.waitForServer(); //will wait for infinite time

    //ROS_INFO("server found.");

    pr2_common_action_msgs::ArmMoveIKGoal goal;
    goal.pose.header.frame_id = "base_link";
    goal.pose.header.stamp = ros::Time(0);

    goal.pose.pose.orientation.x = ox;
    goal.pose.pose.orientation.y = oy;
    goal.pose.pose.orientation.z = oz;
    goal.pose.pose.orientation.w = ow;
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.position.z = z;

    goal.ik_timeout = ros::Duration(5.0);

    if (side_==0)
    {
        goal.ik_seed.name.push_back("r_shoulder_pan_joint");
        goal.ik_seed.name.push_back("r_shoulder_lift_joint");
        goal.ik_seed.name.push_back("r_upper_arm_roll_joint");
        goal.ik_seed.name.push_back("r_elbow_flex_joint");
        goal.ik_seed.name.push_back("r_forearm_roll_joint");
        goal.ik_seed.name.push_back("r_wrist_flex_joint");
        goal.ik_seed.name.push_back("r_wrist_roll_joint");
    }
    else
    {
        goal.ik_seed.name.push_back("l_shoulder_pan_joint");
        goal.ik_seed.name.push_back("l_shoulder_lift_joint");
        goal.ik_seed.name.push_back("l_upper_arm_roll_joint");
        goal.ik_seed.name.push_back("l_elbow_flex_joint");
        goal.ik_seed.name.push_back("l_forearm_roll_joint");
        goal.ik_seed.name.push_back("l_wrist_flex_joint");
        goal.ik_seed.name.push_back("l_wrist_roll_joint");
    }

    ros::Rate rate(20);
    bool good = true;
    while (!good)
    {
        mutex_.lock();
        // start from current joint states
        good = true;
        for (int i=0; i<7; ++i)
        {
            //sumofj += jointState[i];
            if (jointState[i] == 0.0f)
                good = false;
        }
        //ROS_INFO("Joint States %f %f %f %f %f %f %f", jointState[0],jointState[1],jointState[2],jointState[3],jointState[4],jointState[5],jointState[6]);
        mutex_.unlock();
        ros::spinOnce();
        rate.sleep();
    }

    mutex_.lock(); // for accessing current joint state
    for (int i=0; i<7; ++i)
    {
        if ((i != 2) || (!raise_elbow))
            goal.ik_seed.position.push_back(jointState[i]);
        else
            goal.ik_seed.position.push_back((side_==0) ? -preset_angle : preset_angle);
        //goal.ik_seed.position.push_back((side_==0) ? -3.1 : 3.1);

        //ROS_INFO("JOINT %i : %f", i, goal.ik_seed.position[i]);
    }


    mutex_.unlock();
    // somewhat close to what we use for looking at the drawer
    goal.move_duration = ros::Duration(time);

    ac_->sendGoal(goal);

    if (!evil_switch)
    {

        bool finished_before_timeout = false;
        try
        {
            finished_before_timeout = ac_->waitForResult(ros::Duration(time_to_target + 3.0f));
        }
        catch ( boost::thread_interrupted ti)
        {
            ac_->cancelAllGoals();
            ROS_ERROR("MOVE_IK side %i Interrupted: Thread killed. Cancelling all arm ac goals", this->side_);
            throw ti;
        }
        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac_->getState();
            ROS_INFO("%s Arm Action finished: %s",((side_==0) ? "right" : "left"), state.toString().c_str());
            if ((state == actionlib::SimpleClientGoalState::PREEMPTED) || (state == actionlib::SimpleClientGoalState::SUCCEEDED))
                return true;
            else
            {
                ROS_ERROR("Action finished: %s", state.toString().c_str());
                ROS_ERROR("In move_ik, with arm %i", this->side_);
                ROS_ERROR("failed goal (in base): %f %f %f %f %f %f %f (time %f)", x, y, z, ox, oy, oz, ow, time);
                //if (--retries > 0)
                //    return move_ik(x, y, z, ox, oy, oz, ow, time);
                //else
                return false;
            }
        }
        else
            ROS_INFO("Action did not finish before the time out.");
        return false;
    }
    else return true;
}


RobotArm *RobotArm::instance[] = {0,0};

tf::Stamped<tf::Pose> RobotArm::tool2wrist(tf::Stamped<tf::Pose> toolPose)
{
    tf::Stamped<tf::Pose> wrist2tool;
    wrist2tool.frame_id_ = (side_==0) ? "r_wrist_roll_link" : "l_wrist_roll_link";
    wrist2tool.stamp_ = ros::Time();
    wrist2tool.setOrigin(btVector3(-.18,0,0));
    wrist2tool.setRotation(btQuaternion(0,0,0,1));
    tf::Stamped<tf::Pose> ret;
    ret = toolPose;
    ret *= wrist2tool;
    return ret;
}

tf::Stamped<tf::Pose> RobotArm::wrist2tool(tf::Stamped<tf::Pose> toolPose)
{
    tf::Stamped<tf::Pose> wrist2tool;
    wrist2tool.frame_id_ = (side_==0) ? "r_wrist_roll_link" : "l_wrist_roll_link";
    wrist2tool.stamp_ = ros::Time();
    wrist2tool.setOrigin(btVector3(.18,0,0));
    wrist2tool.setRotation(btQuaternion(0,0,0,1));
    tf::Stamped<tf::Pose> ret;
    ret = toolPose;
    ret *= wrist2tool;
    return ret;
}

//bool RobotArm::checkHypothesis()

#include <stdio.h>
#include <stdlib.h>

bool RobotArm::findBaseMovement(btVector3 &result, std::vector<int> arm, std::vector<tf::Stamped<tf::Pose> > goal, bool drive, bool reach)
{

    //system("rosrun dynamic_reconfigure dynparam set  /camera_synchronizer_node projector_mode 1");

    ros::NodeHandle node_handle;
    //ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "robot_arm_marker", 1000 );

    ROS_INFO("looking for base movement to satisfy ik constraints : ");

    if (goal.size() == 0)
        return btVector3(0,0,0);
    std::vector<tf::Stamped<tf::Pose > > goalInBase;
    goalInBase.resize(goal.size());

    for (size_t k = 0; k < goal.size(); ++k)
    {
        //tf::Stamped<tf::Pose> wrist2tool;
        //wrist2tool.frame_id_ = arm[k] ? "r_wrist_roll_link" : "l_wrist_roll_link";
        //wrist2tool.stamp_ = ros::Time();
        //wrist2tool.setOrigin(btVector3(-.18,0,0));
        //wrist2tool.setRotation(btQuaternion(0,0,0,1));
        //ROS_INFO("TOOL");
        //RobotArm::getInstance(0)->printPose(goalInBase[k]);
        //goalInBase [k] *= wrist2tool;
        //goalInBase[k] = getPoseIn("base_link",goal[k]);
        //ROS_INFO("WRIST");
        //RobotArm::getInstance(0)->printPose(goalInBase[k]);

        ROS_INFO("TOOL IN MAP");
        RobotArm::getInstance(0)->printPose(goal[k]);

        //put it in base
        goalInBase[k] = getPoseIn("base_link",goal[k]);;
        //get the wrist pose from it
        goalInBase[k] = RobotArm::getInstance(arm[k])->tool2wrist(goalInBase[k]);

        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "base_link";
            marker.header.stamp = ros::Time::now();
            marker.ns = "goalpoints";
            marker.id = k + 10000;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.x = goalInBase[k].getRotation().x();
            marker.pose.orientation.y = goalInBase[k].getRotation().y();
            marker.pose.orientation.z = goalInBase[k].getRotation().z();
            marker.pose.orientation.w = goalInBase[k].getRotation().w();
            marker.pose.position.x = goalInBase[k].getOrigin().x();
            marker.pose.position.y = goalInBase[k].getOrigin().y();
            marker.pose.position.z = goalInBase[k].getOrigin().z();
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker.lifetime = ros::Duration();

            ROS_INFO("PUBLISH MARKER %i", marker.id);

            vis_pub.publish( marker );

            ros::spinOnce();
        }
    }

    // from now on we work in wrist coordinates

    tf::Stamped<tf::Pose> wristPose[2];
    RobotArm::getInstance(0)->getWristPose(wristPose[0], "base_link");
    RobotArm::getInstance(1)->getWristPose(wristPose[1], "base_link");

    tf::Stamped<tf::Pose> a = RobotArm::getInstance(0)->getPoseIn("map", wristPose[0]);
    tf::Stamped<tf::Pose> b = RobotArm::getInstance(0)->getPoseIn("map", wristPose[1]);
    ROS_INFO("CURRENT WRIST R");
    RobotArm::getInstance(0)->printPose(a);
    ROS_INFO("CURRENT WRIST L");
    RobotArm::getInstance(1)->printPose(b);

    float s[] = {0,0};
    for (size_t k = 0; k < goal.size(); ++k)
    {
        s[0] += wristPose[arm[k]].getOrigin().x() - goalInBase[k].getOrigin().x();
        s[1] += wristPose[arm[k]].getOrigin().y() - goalInBase[k].getOrigin().y();
    }

    s[0] /= (float) goal.size();
    s[1] /= (float) goal.size();

    // minimum movement 5 cm
    s[0] = ceil(s[0] * 20) / 20.0f;
    s[1] = ceil(s[1] * 20) / 20.0f;

    ROS_INFO("SPEEDUP X %f Y %f" , s[0], s[1]);

    // daft punk
    //if (sqrtf(s[0]*s[0] + s[1]*s[1]) <= 0.75)
    {
        s[0] = 0;
        s[1] = 0;
    }

    //if (sqrt(s[0] * s[0] + s[1] * s[1]) < 0.1) {
    //s[0] = 0;
    //s[1] = 0;
    //}

    ROS_INFO("SPEEDUP X %f Y %f" , s[0], s[1]);

    double start_angles[2][7];
    RobotArm::getInstance(0)->getJointState(start_angles[0]);
    RobotArm::getInstance(1)->getJointState(start_angles[1]);

    double minDist = 10000;
    double best_x = 0;
    double best_y = 0;
    double range = 0.75;
    double maxrange = 1.5;
    double step = 0.05;
    int cnt = 0;

    int markerCnt = 0;

    double rangeIncrement = 0.75;

    if (sqrtf(s[0]*s[0] + s[1]*s[1]) < 0.5)
        rangeIncrement = 0.25;

    ROS_INFO("before loop");
    for (range = rangeIncrement; (range <= maxrange) && (minDist == 10000); range += rangeIncrement)
        for (double dx = -range + s[0]; (dx <= range + s[0]) && (ros::ok()); dx += step)
        {
            for (double dy = -range + s[1]; dy <= range + s[1]; dy += step)
            {
                btVector3 distVec(dx,dy,0);
                double dist = distVec.length();
                btVector3 rangeDistVec(dx - s[0],dy - s[1],0);
                double rangeDist = rangeDistVec.length();
                size_t found = 0;
                float dr[2];
                dr[0] = dx;
                dr[1] = dy;
                if ((dist < minDist) && (rangeDist < range) && (rangeDist >= range - rangeIncrement) && RobotDriver::getInstance()->checkCollision(dr))
                {
                    bool good = true;
                    for (size_t k = 0; (k < goal.size()) && good; ++k)
                    {
                        btVector3 curGoalPos = goalInBase[k].getOrigin() + btVector3(dx,dy,0);

                        geometry_msgs::PoseStamped stamped_pose;
                        stamped_pose.header.frame_id = "base_link";
                        stamped_pose.header.stamp = ros::Time::now();
                        stamped_pose.pose.position.x=curGoalPos.x();
                        stamped_pose.pose.position.y=curGoalPos.y();
                        stamped_pose.pose.position.z=curGoalPos.z();
                        stamped_pose.pose.orientation.x=goalInBase[k].getRotation().x();
                        stamped_pose.pose.orientation.y=goalInBase[k].getRotation().y();
                        stamped_pose.pose.orientation.z=goalInBase[k].getRotation().z();
                        stamped_pose.pose.orientation.w=goalInBase[k].getRotation().w();
                        double new_state[7];
                        bool f = RobotArm::getInstance(arm[k])->run_ik(stamped_pose,start_angles[arm[k]],new_state,(arm[k] == 0) ? "r_wrist_roll_link" : "l_wrist_roll_link");
                        //ROS_INFO("DXDY %f %f", dx, dy);
                        cnt++;
                        if (cnt && !(cnt % 25))
                            ROS_INFO("Cnt %i", cnt);
                        if (f)
                            found++;
                        else
                            good = false; // for now,get out when one fails.
                    }
                }
                bool col = true;
                bool isGood = (found==goal.size());
                if (found==goal.size())
                {
                    //if (RobotDriver::getInstance()->checkCollision(dr))
                    //{
                    printf("DIST %f found %f %f %s \n", dist, dx, dy, "FOUND");
                    best_x = dx;
                    best_y = dy;
                    minDist = dist;
                    col = false;
                    //}
                }
                if (isGood)
                {
                    visualization_msgs::Marker marker;
                    marker.header.frame_id = "base_link";
                    marker.header.stamp = ros::Time::now();
                    marker.ns = "target base footprints";
                    marker.id = ++markerCnt;
                    marker.type = visualization_msgs::Marker::LINE_STRIP;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.pose.position.x = 0;
                    marker.pose.position.y = 0;
                    marker.pose.position.z = 0;
                    marker.pose.orientation.x = 0.0;
                    marker.pose.orientation.y = 0.0;
                    marker.pose.orientation.z = 0.0;
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x = 0.01;
                    marker.scale.y = 0.01;
                    marker.scale.z = 0.01;
                    marker.color.a = 1.0;
                    marker.color.r = (col ? 1.0 : 0.0);
                    marker.color.g = (col ? 0.0 : 1.0);
                    marker.color.b = 0.0;
                    float si = .325;
                    if (!isGood)
                        si = .3;
                    if (!isGood)
                        marker.color.b = markerCnt / 1000.0f;
                    {
                        geometry_msgs::Point p;
                        p.x =  -si -  dx;
                        p.y =  -si -  dy;
                        p.z = 0.1;
                        marker.points.push_back(p);
                    }
                    {
                        geometry_msgs::Point p;
                        p.x =  si -  dx;
                        p.y =  -si -  dy;
                        p.z = 0.1;
                        marker.points.push_back(p);
                    }
                    {
                        geometry_msgs::Point p;
                        p.x =  si -  dx;
                        p.y =  si -  dy;
                        p.z = 0.1;
                        marker.points.push_back(p);
                    }
                    {
                        geometry_msgs::Point p;
                        p.x =  -si -  dx;
                        p.y =  si -  dy;
                        p.z = 0.1;
                        marker.points.push_back(p);
                    }
                    {
                        geometry_msgs::Point p;
                        p.x =  -si -  dx;
                        p.y =  -si -  dy;
                        p.z = 0.1;
                        marker.points.push_back(p);
                    }

                    vis_pub.publish( marker );
                }
            }
        }
    if (minDist < 10000)
    {

        float ps[4];
        ps[0] = -best_x;
        ps[1] = -best_y;
        ps[2] = 0;
        ps[3] = 1;

        ROS_ERROR("FOUND BASE MOVEMENT %f %f Query Count %i", -best_x, -best_y, cnt);

        RobotDriver *driver = RobotDriver::getInstance();
        // todo: chck for localization, since move base is not exact! pose -> mappose -> pose
        //move_ik(x + best_x,y + best_y,z,ox,oy,oz,ow);
        if (drive && ((fabs(best_x) > 0.01) || (fabs(best_y) > 0.01)))
        {
            //driver->driveInOdom(ps,true);
            //boost::thread t2(&RobotDriver::driveInOdom,driver,ps,true);
            boost::thread t2(&RobotDriver::driveInOdom,driver,ps,false);
            ros::Rate rate(5.0);
            while (!t2.timed_join(boost::posix_time::seconds(0.01)))
            {
                //ROS_ERROR("not yet joinable");
                tf::Stamped<tf::Pose> firstGoalBase = RobotArm::getInstance(arm[0])->getPoseIn("base_link",goal[0]);
                if (reach)
                    RobotArm::getInstance(arm[0])->move_toolframe_ik(firstGoalBase.getOrigin().x(), firstGoalBase.getOrigin().y(), firstGoalBase.getOrigin().z(),
                            firstGoalBase.getRotation().x(),firstGoalBase.getRotation().y(),firstGoalBase.getRotation().z(),firstGoalBase.getRotation().w());
                //RobotArm::getInstance(arm[0])->move_toolframe_ik(x + best_x,y + best_y,z,ox,oy,oz,ow);
                rate.sleep();
            }
            t2.join();

        }

        result = btVector3(best_x, best_y, 0);
        return true;
    }
    return false;

}

//  move the toolframe including base motions when necessary
btVector3 RobotArm::universal_move_toolframe_ik(float x, float y, float z, float ox, float oy, float oz, float ow, const char target_frame[])
{
    tf::Stamped<tf::Pose> toolTargetPose;

    toolTargetPose.frame_id_ = target_frame;
    toolTargetPose.stamp_ = ros::Time(0);
    toolTargetPose.setOrigin(btVector3( x,y,z));
    toolTargetPose.setRotation(btQuaternion(ox,oy,oz,ow));

    return universal_move_toolframe_ik_pose(toolTargetPose);
}


btVector3 RobotArm::universal_move_toolframe_ik_pose(tf::Stamped<tf::Pose> toolTargetPose)
{
    //return universal_move_toolframe_ik(toolTargetPose.getOrigin().x(),toolTargetPose.getOrigin().y(),toolTargetPose.getOrigin().z(),
    //toolTargetPose.getRotation().x(),toolTargetPose.getRotation().y(),toolTargetPose.getRotation().z(),toolTargetPose.getRotation().w(),toolTargetPose.frame_id_.c_str());



    //ROS_INFO("in map");
    //printPose(toolTargetPoseInMap);

    tf::Stamped<tf::Pose> toolTargetPoseInBase = getPoseIn("base_link",toolTargetPose);
    //ROS_INFO("in base_link");
    //printPose(toolTargetPoseInBase);

    tf::Stamped<tf::Pose> toolPose;
    getToolPose(toolPose, "base_link");
    //ROS_INFO("tool pose in base link");
    //printPose(toolPose);

    //double sx = toolPose.getOrigin().x() - toolTargetPoseInBase.getOrigin().x();
    //double sy = toolPose.getOrigin().y() - toolTargetPoseInBase.getOrigin().y();
    //sx = 0;
    //sy = 0;
    //toolTargetPoseInMap = getPoseIn("map", toolTargetPoseInBase);


    //x = toolTargetPoseInBase.getOrigin().x();
    //y = toolTargetPoseInBase.getOrigin().y();
    //z = toolTargetPoseInBase.getOrigin().z();
    //ox = toolTargetPoseInBase.getRotation().x();
    //oy = toolTargetPoseInBase.getRotation().y();
    //oz = toolTargetPoseInBase.getRotation().z();
    //ow = toolTargetPoseInBase.getRotation().w();

    tf::Stamped<tf::Pose> wristTargetBase = tool2wrist(toolTargetPoseInBase);

    geometry_msgs::PoseStamped stamped_pose;
    stamped_pose.header.frame_id = "base_link";
    stamped_pose.header.stamp = ros::Time::now();
    stamped_pose.pose.position.x=wristTargetBase.getOrigin().x();
    stamped_pose.pose.position.y=wristTargetBase.getOrigin().y();
    stamped_pose.pose.position.z=wristTargetBase.getOrigin().z();
    stamped_pose.pose.orientation.x=wristTargetBase.getRotation().x();
    stamped_pose.pose.orientation.y=wristTargetBase.getRotation().y();
    stamped_pose.pose.orientation.z=wristTargetBase.getRotation().z();
    stamped_pose.pose.orientation.w=wristTargetBase.getRotation().w();

    double new_state[7];

    double start_angles[7];
    getJointState(start_angles);

    if (raise_elbow)
        start_angles[2] = ((side_==0) ? -preset_angle : preset_angle);

    bool foundInitially = run_ik(stamped_pose,start_angles,new_state,(side_ == 0) ? "r_wrist_roll_link" : "l_wrist_roll_link");

    //if (poseInsideFootprint && excludeBaseProjectionFromWorkspace)
    // foundInitially = false;
    bool poseInsideFootprint = false;
    if (excludeBaseProjectionFromWorkspace)
    {
        ROS_WARN("excludeBaseProjectionFromWorkspace == true");

        float padding = 0.05;
        float x = stamped_pose.pose.position.x;
        float y = stamped_pose.pose.position.y;
        // does target pose lie inside projection of base footprint?
        if ((x < .325 + padding) && (x > -.325 - padding) && (y < .325 + padding) && (y > -.325 - padding))
            poseInsideFootprint = true;
    }


//if (!move_ik(x,y,z,ox,oy,oz,ow,time_to_target))
    if (!poseInsideFootprint && move_ik(wristTargetBase.getOrigin().x(),wristTargetBase.getOrigin().y(),wristTargetBase.getOrigin().z(),
                                        wristTargetBase.getRotation().x(),wristTargetBase.getRotation().y(),wristTargetBase.getRotation().z(),wristTargetBase.getRotation().w(),time_to_target))
    {
        if (!foundInitially)
            ROS_ERROR("RUN IK SAYS NOT FOUND INITIALLY BUT WE REACHED IT WITH MOVE_IK");

        return btVector3(0,0,0);
    }
    else
    {
        if (foundInitially)
        {
            ROS_ERROR("RobotArm::universal_move_toolframe_ik run_ik says reachable move_ik says not!");
        }

        ROS_ERROR("NOT FOUND INITIALLY");
        ROS_INFO("looking for base movement to reach tf %f %f %f %f %f %f %f", toolTargetPose.getOrigin().x(),toolTargetPose.getOrigin().y(),toolTargetPose.getOrigin().z(),
                 toolTargetPose.getRotation().x(),toolTargetPose.getRotation().y(),toolTargetPose.getRotation().z(),toolTargetPose.getRotation().w());

        std::vector<int> arm;
        std::vector<tf::Stamped<tf::Pose> > goal;
        btVector3 result;
        arm.push_back(side_);
        //toolTargetPose *= wrist2tool;
        tf::Stamped<tf::Pose> toolTargetPoseInMap = getPoseIn("map",toolTargetPose);
        goal.push_back(toolTargetPoseInMap);
        RobotArm::findBaseMovement(result, arm, goal, true, true);
        return universal_move_toolframe_ik_pose(toolTargetPoseInMap);
    }


    return btVector3(0,0,0);
}



tf::Stamped<tf::Pose> RobotArm::runFK(double jointAngles[], tf::Stamped<tf::Pose> *elbow)
{

    tf::Stamped<tf::Pose> res;
    // define the service messages
    kinematics_msgs::GetKinematicSolverInfo::Request request;
    kinematics_msgs::GetKinematicSolverInfo::Response response;

    ros::service::waitForService((side_==0) ? "pr2_right_arm_kinematics/get_fk_solver_info" : "pr2_left_arm_kinematics/get_fk_solver_info");

    if (query_client.call(request,response))
    {

        //for (unsigned int i=0; i < response.kinematic_solver_info.joint_names.size(); i++)
        // {
        //   ROS_INFO("Joint: %d %s", i,response.kinematic_solver_info.joint_names[i].c_str());
        // }
    }
    else
    {
        ROS_ERROR("Could not call query service");
        //ros::shutdown();
        //exit(1);
        return res;
    }
    // define the service messages
    kinematics_msgs::GetPositionFK::Request  fk_request;
    kinematics_msgs::GetPositionFK::Response fk_response;
    //fk_request.header.frame_id = "torso_lift_link";
    //fk_request.header.frame_id = "torso_lift_joint";
    fk_request.header.frame_id = "base_link";
    fk_request.fk_link_names.resize(2);
    //for (int i = 0; i < 7; ++i) {
    //ROS_INFO("LINK %i : %s", i, joint_names[i].c_str());
    //fk_request.fk_link_names[i] = joint_names[i];
    //}
    fk_request.fk_link_names[0] = (side_==0) ? "r_wrist_roll_link" : "l_wrist_roll_link";
    fk_request.fk_link_names[1] = (side_==0)? "r_elbow_flex_link" : "l_elbow_flex_link" ;
    //fk_request.fk_link_names[0] = "r_wrist_roll_link";
    //fk_request.fk_link_names[1] = "r_elbow_flex_link";
    //fk_request.robot_state.joint_state.position.resize(response.kinematic_solver_info.joint_names.size());
    fk_request.robot_state.joint_state.position.resize(7);
    fk_request.robot_state.joint_state.name=response.kinematic_solver_info.joint_names;
    for (unsigned int i=0;
            i< response.kinematic_solver_info.joint_names.size(); i++)
    {
        //ROS_INFO("LINK %i : %f", i, jointAngles[i]);
        fk_request.robot_state.joint_state.position[i] = jointAngles[i];
    }

    ros::service::waitForService((side_==0) ? "pr2_right_arm_kinematics/get_fk" : "pr2_left_arm_kinematics/get_fk");
    if (fk_client.call(fk_request, fk_response))
    {
        //if (0)
        if (fk_response.error_code.val == fk_response.error_code.SUCCESS)
        {
            if (0)
                for (unsigned int i=0; i < fk_response.pose_stamped.size(); i ++)
                {
                    ROS_INFO_STREAM("Link    : " << fk_response.fk_link_names[i].c_str());
                    ROS_INFO_STREAM("Position: " <<
                                    fk_response.pose_stamped[i].pose.position.x << "," <<
                                    fk_response.pose_stamped[i].pose.position.y << "," <<
                                    fk_response.pose_stamped[i].pose.position.z);
                    ROS_INFO("Orientation: %f %f %f %f",
                             fk_response.pose_stamped[i].pose.orientation.x,
                             fk_response.pose_stamped[i].pose.orientation.y,
                             fk_response.pose_stamped[i].pose.orientation.z,
                             fk_response.pose_stamped[i].pose.orientation.w);
                }


            res.frame_id_ = fk_request.header.frame_id;
            int o = 0;
            res.setOrigin(btVector3(fk_response.pose_stamped[o].pose.position.x, fk_response.pose_stamped[o].pose.position.y, fk_response.pose_stamped[o].pose.position.z));
            res.setRotation(btQuaternion(fk_response.pose_stamped[o].pose.orientation.x,fk_response.pose_stamped[o].pose.orientation.y,fk_response.pose_stamped[o].pose.orientation.z,fk_response.pose_stamped[o].pose.orientation.w));

            if (elbow)
            {
                res.frame_id_ = fk_request.header.frame_id;
                int o = 1;
                elbow->setOrigin(btVector3(fk_response.pose_stamped[o].pose.position.x, fk_response.pose_stamped[o].pose.position.y, fk_response.pose_stamped[o].pose.position.z));
                elbow->setRotation(btQuaternion(fk_response.pose_stamped[o].pose.orientation.x,fk_response.pose_stamped[o].pose.orientation.y,fk_response.pose_stamped[o].pose.orientation.z,fk_response.pose_stamped[o].pose.orientation.w));
            }
        }
        else
        {
            ROS_ERROR("Forward kinematics failed");
        }
    }
    else
    {
        ROS_ERROR("Forward kinematics service call failed");
    }


    //fk_request.fk_link_names[1] = "r_elbow_flex_link";

    return res;
}


btVector3 RobotArm::cartError()
{
    double state[7];
    getJointState(state);
    double stateDes[7];
    getJointStateDes(stateDes);
    tf::Stamped<tf::Pose> posAct = runFK(state);
    tf::Stamped<tf::Pose> posDes = runFK(stateDes);
    btVector3 diff = posDes.getOrigin() - posAct.getOrigin();
    //ROS_INFO("DIFF %f %f %f DISTANCE %f", diff.x(), diff.y(), diff.y(), diff.length());
    return diff;
}


void RobotArm::moveElbowOutOfWay(tf::Stamped<tf::Pose> toolTargetPose)
{

    RobotArm *arm = this;
    double state[7];
    arm->getJointState(state);

    tf::Stamped<tf::Pose> actPose;
    actPose = tool2wrist(getPoseIn("base_link", toolTargetPose));


    //arm->getWristPose(actPose,"base_link");

    geometry_msgs::PoseStamped stamped_pose;
    stamped_pose.header.frame_id = "base_link";
    stamped_pose.header.stamp = ros::Time::now();
    stamped_pose.pose.position.x=actPose.getOrigin().x();
    stamped_pose.pose.position.y=actPose.getOrigin().y();
    stamped_pose.pose.position.z=actPose.getOrigin().z();
    stamped_pose.pose.orientation.x=actPose.getRotation().x();
    stamped_pose.pose.orientation.y=actPose.getRotation().y();
    stamped_pose.pose.orientation.z=actPose.getRotation().z();
    stamped_pose.pose.orientation.w=actPose.getRotation().w();

    ros::Rate rate(50.0);

    //for (double add = 0; ros::ok(); add+= 0.01)
    while (ros::ok())
    {
        rate.sleep();
        ros::spinOnce();

        double jErr[7];
        double jDes[7];

        arm->getJointStateErr(jErr);
        arm->getJointStateDes(jDes);

        float increment = .1;

        ROS_INFO("jERR[1]=%f", jErr[1]);

        if (fabs(jErr[1]) > 0.0001)
        {
            // {

            double stateP[7];
            double statePK[7];
            double stateM[7];
            //double solStateP[7];
            //double solStatePK[7];
            //double solStateM[7];

            arm->getJointState(stateP);
            arm->getJointState(statePK);
            arm->getJointState(stateM);

            double stA[7];
            double stB[7];
            double stC[7];
            arm->getJointState(stA);
            arm->getJointState(stB);
            arm->getJointState(stC);

            stB[2] += increment;

            double stAs[7];
            double stBs[7];
            double stCs[7];

            arm->run_ik(stamped_pose,stA,stAs,(side_ == 0) ? "r_wrist_roll_link" : "l_wrist_roll_link");
            arm->run_ik(stamped_pose,stB,stBs,(side_ == 0) ? "r_wrist_roll_link" : "l_wrist_roll_link");

            double newinc = (jErr[1] / (stBs[1] - stAs[1])) * increment;

            stC[2] += newinc;

            arm->run_ik(stamped_pose,stC,stCs,(side_ == 0) ? "r_wrist_roll_link" : "l_wrist_roll_link");

            ROS_INFO("curr (sta[1]) %f  stCs %f", stA[1], stCs[1]);

            float pose[7];
            float sum = 0;
            for (int k = 0; k < 7; ++k)
            {
                pose[k] = stCs[k];
                sum += stCs[k] * stCs[k];
            }

            if (sum > 0.01)
                arm->startTrajectory(arm->goalTraj(pose));

            if (stBs[2]==0)
            {
                increment = -increment;
                ROS_INFO("HIT LIMIT?");
            }
        }
        else
        {
            break;
        }
    }
}



