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

// roslaunch arm_ik.launch
#include <ros/ros.h>

#include <ias_drawer_executive/RobotArm.h>

ros::Publisher pubR, pubL;

void printPose(const char title[], tf::Stamped<tf::Pose> pose)
{
    ROS_INFO("%s %f %f %f %f %f %f %f %s", title, pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z()
             , pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z(), pose.getRotation().w(), pose.frame_id_.c_str());
}


void poseCallbackR(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

    double start_angles[7];
    double solutions[7];
    RobotArm::getInstance(0)->getJointState(start_angles);

    tf::Stamped<tf::Pose> nextPose;
    tf::poseStampedMsgToTF(*msg, nextPose);

    nextPose = RobotArm::getInstance(0)->tool2wrist(nextPose);
    geometry_msgs::PoseStamped inWrist;
    tf::poseStampedTFToMsg(nextPose,inWrist);

    bool ik_res = RobotArm::getInstance(0)->run_ik(inWrist, start_angles,solutions,"r_wrist_roll_link");

    if (!ik_res)  {
        ROS_ERROR("outside bounds");
        printPose("right arm outside workspace, for wrist", nextPose);
        return;
    }

    pubR.publish(msg);

}



void poseCallbackL(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

    double start_angles[7];
    double solutions[7];
    RobotArm::getInstance(1)->getJointState(start_angles);

    tf::Stamped<tf::Pose> nextPose;
    tf::poseStampedMsgToTF(*msg, nextPose);

    nextPose = RobotArm::getInstance(1)->tool2wrist(nextPose);
    geometry_msgs::PoseStamped inWrist;
    tf::poseStampedTFToMsg(nextPose,inWrist);

    bool ik_res = RobotArm::getInstance(1)->run_ik(inWrist, start_angles,solutions,"l_wrist_roll_link");

    if (!ik_res)  {
        ROS_ERROR("outside bounds");
        printPose("left arm outside workspace, for wrist", nextPose);
        return;
    }

    pubL.publish(msg);

}


int main(int argc, char** argv)
{
    // Init the ROS node
    ros::init(argc, argv, "WorkSpaceFilter");
    ros::NodeHandle node_handle;

    ros::Subscriber subsRight = node_handle.subscribe("/r_workspace_filter", 1, &poseCallbackR);
    ros::Subscriber subsLeft = node_handle.subscribe("/l_workspace_filter", 1, &poseCallbackL);

    ros::Publisher pubR = node_handle.advertise<geometry_msgs::PoseStamped>( "/r_out_workspace_filter", 100, true );
    ros::Publisher pubL = node_handle.advertise<geometry_msgs::PoseStamped>( "/l_out_workspace_filter", 100, true );

    ROS_INFO("READY");

    ros::spin();
}

