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

#ifndef __ROBOTARM_H__
#define __ROBOTARM_H__

// roslaunch arm_ik.launch
#include <ros/ros.h>

#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pr2_common_action_msgs/ArmMoveIKAction.h>
#include <tf/transform_listener.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <pr2_msgs/PressureState.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetPositionFK.h>


#include <find_base_pose/FindBasePoseAction.h>


#include <boost/thread/mutex.hpp>


//l_arm_tucked = [0.06024, 1.248526, 1.789070, -1.683386, -1.7343417, -0.0962141, -0.0864407]
//r_arm_tucked = [-0.023593, 1.1072800, -1.5566882, -2.124408, -1.4175, -1.8417, 0.21436]


typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

class RobotArm
{
private:
    // Action client for the joint trajectory action
    // used to trigger the arm movement action

    int side_;

    TrajClient* traj_client_;
    static tf::TransformListener *listener_;
    actionlib::SimpleActionClient<pr2_common_action_msgs::ArmMoveIKAction> *ac_;
    static actionlib::SimpleActionClient<find_base_pose::FindBasePoseAction> *ac_fbp_;
    ros::Subscriber jointStateSubscriber_;
    ros::NodeHandle n_;
    boost::mutex mutex_;

    ros::ServiceClient query_client;
    ros::ServiceClient fk_client;
    ros::ServiceClient ik_client;

    volatile bool haveJointState;
    double jointState[7];
    double jointStateDes[7];
    double jointStateErr[7];
    std::string joint_names[7];

    void jointStateCallback(const  pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr& msg);

    static RobotArm *instance[];

    //! Initialize the action client and wait for action server to come up
    RobotArm(int side);

    //! Clean up the action client
    ~RobotArm();

public:
    //will be private again one day
    bool tucked;

    int retries;

    bool raise_elbow;
    double preset_angle;

    actionlib::SimpleActionClient<pr2_common_action_msgs::ArmMoveIKAction> *getActionClient()
    {
        return ac_;
    }

    static RobotArm* getInstance(int side = 0);

    bool isTucked()
    {
        return tucked;
    }

    void getJointState(double state[]);
    void getJointStateDes(double state[]);
    void getJointStateErr(double state[]);

    static void init();

    //tf::Stamped<tf::Pose> getTransformIn(const char target_frame[], tf::Stamped<tf::Pose>src);

  // static helpers
    //scales the transform, e.g. scale .10 = apply returned transform 10 times on the right side to get the same result as applying in once
    //static tf::Stamped<tf::Pose> scaleStampedPose(const tf::Stamped<tf::Pose> &in, double scale);
    //static tf::Stamped<tf::Pose> scaleStampedTransform(const tf::Stamped<tf::Pose> &in, double scale);

    //static tf::Stamped<tf::Pose> getPoseIn(const char target_frame[], tf::Stamped<tf::Pose> src);

    //run inverse kinematics on a PoseStamped (7-dof pose
    //(position + quaternion orientation) + header specifying the
    //frame of the pose)
    //tries to stay close to double start_angles[7]
    //returns the solution angles in double solution[7]
    bool run_ik(geometry_msgs::PoseStamped pose, double start_angles[7],double solution[7], std::string link_name);

    bool run_ik(tf::Stamped<tf::Pose> pose, double start_angles[7],double solution[7], std::string link_name);

    tf::Stamped<tf::Pose> runFK(double jointAngles[], tf::Stamped<tf::Pose> *elbow = 0);


    //! cartesian difference at end effector resulting from difference between desired and actual joint state
    btVector3 cartError();

    //! Sends the command to start a given trajectory
    void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal,bool wait = true);

    // Generates a simple trajectory with two waypoints, used as an example
    pr2_controllers_msgs::JointTrajectoryGoal twoPointTrajectory(double *poseA, double *poseB);

    pr2_controllers_msgs::JointTrajectoryGoal multiPointTrajectory(const std::vector<std::vector<double> > &poses, const double &duration = 1.0);

    pr2_controllers_msgs::JointTrajectoryGoal multiPointTrajectory(const std::vector<std::vector<double> > &poses, const std::vector<double> &duration);

    pr2_controllers_msgs::JointTrajectoryGoal multiPointTrajectory(const std::vector<tf::Stamped<tf::Pose> > &poses,  const std::vector<double> &duration);

    pr2_controllers_msgs::JointTrajectoryGoal goalTraj(double a0, double a1, double a2, double a3, double a4, double a5, double a6, double dur=1.0);
    // 0 velocity at goal point
    pr2_controllers_msgs::JointTrajectoryGoal goalTraj(double *poseA, double dur=1.0);
    // some velocity at goal point
    pr2_controllers_msgs::JointTrajectoryGoal goalTraj(double *poseA, double *vel);

    //! Returns the current state of the action
    actionlib::SimpleClientGoalState getState();

    static void printPose(const tf::Stamped<tf::Pose> &toolTargetPose);

    //void getToolPose(tf::Stamped<tf::Pose> &marker);

    //static tf::Stamped<tf::Pose> getPose(const char target_frame[],const char lookup_frame[]);

    tf::Stamped<tf::Pose> getToolPose(const char frame[] = "base_link");

    //static tf::Stamped<tf::Pose> getRelativeTransform(const char source_frameid[], const char target_frameid[]);

    //tf::Stamped<tf::Pose> getTransform(const char baseframe[], const char toolframe[]);

    void getToolPose(tf::Stamped<tf::Pose> &marker, const char frame[] = "base_link");

    void getWristPose(tf::Stamped<tf::Pose> &marker, const char frame[] = "base_link");

    //tf::Stamped<tf::Pose>  rotateAroundBaseAxis(tf::Stamped<tf::Pose> toolPose, double r_x,double r_y,double r_z);
    //tf::Stamped<tf::Pose>  rotateAroundToolframeAxis(tf::Stamped<tf::Pose> toolPose, double r_x,double r_y,double r_z);
    //static tf::Stamped<tf::Pose>  rotateAroundPose(tf::Stamped<tf::Pose> toolPose, tf::Stamped<tf::Pose> pivot, double r_x, double r_y, double r_z);
    //static tf::Stamped<tf::Pose>  rotateAroundPose(tf::Stamped<tf::Pose> toolPose, tf::Stamped<tf::Pose> pivot, btQuaternion qa);

    // rotate gripper around gripper tool frame
    bool rotate_toolframe_ik(double r_x, double r_y, double r_z);
    tf::Stamped<tf::Pose> rotate_toolframe_ik_p(double r_x, double r_y, double r_z);

    // rotate gripper arounr some frame down from wrist
    //bool rotate_toolframe_ik(double r_x, double r_y, double r_z, const char frame_id[]);
    //tf::Stamped<tf::Pose> rotate_toolframe_ik(tf::Stamped<tf::Pose> current,double r_x, double r_y, double r_z);

    //moves the toolframe to the given position
    bool move_toolframe_ik_pose(tf::Stamped<tf::Pose> toolTargetPose);
    bool move_toolframe_ik(double x, double y, double z, double ox, double oy, double oz, double ow);

    void stabilize_grip();

    // rosrun tf tf_echo /base_link /r_wrist_roll_link -> position
    bool move_ik(double x, double y, double z, double ox, double oy, double oz, double ow, double time = 1.0);
    bool move_ik(tf::Stamped<tf::Pose> targetPose, double time = 0.0);

    tf::Stamped<tf::Pose> tool2wrist(tf::Stamped<tf::Pose> toolPose);
    tf::Stamped<tf::Pose> wrist2tool(tf::Stamped<tf::Pose> toolPose);

    //static tf::Stamped<tf::Pose> approach(tf::Stamped<tf::Pose> toolPose, double dist = 0.1);

    static bool findBaseMovement(tf::Stamped<tf::Pose> &result, std::vector<int> arm, std::vector<tf::Stamped<tf::Pose> > goal, bool drive, bool reach);

    //!moves the toolframe to an ik position given in any frame, moving the base without rotating when necessary
    btVector3 universal_move_toolframe_ik_pose(tf::Stamped<tf::Pose> toolTargetPose);

    //!move to the tool to the target pose with a goal tolerance, returns but does not stop arm when tolerance or timeout is reached
    btVector3 universal_move_toolframe_ik_pose_tolerance(tf::Stamped<tf::Pose> toolTargetPose, double tolerance, double timeout = 5.0);

    btVector3 universal_move_toolframe_ik(double x, double y, double z, double ox, double oy, double oz, double ow, const char target_frame[]="base_link");

    //static tf::Stamped<tf::Pose> make_pose(double x, double y, double z, double ox, double oy, double oz, double ow, const char target_frame[]);

    //static tf::Stamped<tf::Pose> make_pose(const btTransform &trans, const char target_frame[]);

    void bring_into_reach(tf::Stamped<tf::Pose> toolTargetPose);

    bool reachable(tf::Stamped<tf::Pose> target);

    void moveElbowOutOfWay(tf::Stamped<tf::Pose> toolTargetPose);

    bool executeViaJointControl(const std::vector<tf::Stamped<tf::Pose> > &poses, int start = -1, int end = -1);

    bool pose2Joint(const std::vector<tf::Stamped<tf::Pose> > &poses, std::vector<std::vector<double> > &joints);

    static void moveBothArms(tf::Stamped<tf::Pose> leftArm, tf::Stamped<tf::Pose> rightArm, double tolerance = 0, bool wait = true);

    // given relative pose is relative to the root_frame, return its pose in the frame the root frame is defined in
    //static tf::Stamped<tf::Pose> getRel(const tf::Stamped<tf::Pose> &root_frame,  const tf::Stamped<tf::Pose> &relative_pose);

    //static tf::Stamped<tf::Pose> getRelInBase(const tf::Stamped<tf::Pose> &root_frame,  const btVector3 &dist);

    double time_to_target;

    bool evil_switch;

    bool excludeBaseProjectionFromWorkspace;

    tf::Stamped<tf::Pose> wrist2tool_;
    tf::Stamped<tf::Pose> tool2wrist_;

    std::string wrist_frame;
    std::string tool_frame;
};


#endif
