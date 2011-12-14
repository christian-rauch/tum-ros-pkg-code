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

void printPose(const char title[], tf::Stamped<tf::Pose> pose)
{
    ROS_INFO("%s %f %f %f %f %f %f %f %s", title, pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z()
             , pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z(), pose.getRotation().w(), pose.frame_id_.c_str());
}

ros::Time lastMessageR;
ros::Time lastMessageL;

void poseCallbackRMoveArm(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

    if ((ros::Time::now() - lastMessageR) < ros::Duration(0.3))
      return;
    lastMessageR = ros::Time::now();

    tf::Stamped<tf::Pose> nextPose;
    tf::poseStampedMsgToTF(*msg, nextPose);
    printPose("target pose: ", nextPose);
    RobotArm::getInstance(0)->time_to_target = 0;
    RobotArm::getInstance(0)->move_ik(nextPose,0);
}

void poseCallbackR(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

    //ROS_INFO("seq %i stamp %i.%i", msg->header.seq,msg->header.stamp.sec, msg->header.stamp.nsec );
    std::cout << "." << std::flush;
    if ((msg->header.stamp - lastMessageR) < ros::Duration(.1))
    {
      //  ROS_ERROR("too young");
        return;
    }

    if ((ros::Time::now() - msg->header.stamp) > ros::Duration(0.5))
    {
        ROS_ERROR("too old %i.%i", (ros::Time::now() - msg->header.stamp).sec, (ros::Time::now() - msg->header.stamp).nsec );
        return;
    }

    lastMessageR = msg->header.stamp;

    double start_angles[7];
    double solutions[7];
    RobotArm::getInstance(0)->getJointState(start_angles);

    tf::Stamped<tf::Pose> nextPose;
    tf::poseStampedMsgToTF(*msg, nextPose);

    tf::Stamped<tf::Pose> actPose = RobotArm::getInstance(0)->getToolPose("map");
    tf::Stamped<tf::Pose> nxtPose = RobotArm::getInstance(0)->getPoseIn("map",nextPose);
    float dist = (actPose.getOrigin() - nxtPose.getOrigin()).length();

    float dist_limit = 0.5;

    if (dist > dist_limit) {
        btVector3 act = actPose.getOrigin();
        btVector3 rel = nxtPose.getOrigin() - actPose.getOrigin();
        ROS_INFO("BFR REL LENGTH : %f", rel.length());
        float dfactor = dist_limit / rel.length();
        rel *= dfactor;
        ROS_INFO("AFT REL LENGTH : %f", rel.length());
        nxtPose.setOrigin(rel + act);
        dist = dist_limit;
    }

    nextPose = nxtPose;

    float timeToTarget = dist / 0.1; // 50 cm per sec

    //ROS_INFO(". dist %f timetotarg %f", dist, timeToTarget);

    nextPose = RobotArm::getInstance(0)->tool2wrist(nextPose);
    geometry_msgs::PoseStamped inWrist;
    tf::poseStampedTFToMsg(nextPose,inWrist);

    bool ik_res = RobotArm::getInstance(0)->run_ik(inWrist, start_angles,solutions,"r_wrist_roll_link");

    printPose("target pose: ", nextPose);

    if (!ik_res)  {
        ROS_ERROR("outside bounds");
        return;
    }

    float solutionsf[7]; for (int i=0;i<7;++i) solutionsf[i] = solutions[i];
    RobotArm::getInstance(0)->startTrajectory(RobotArm::getInstance(0)->goalTraj(solutionsf, timeToTarget),false);

    //tf::Stamped<tf::Pose> nextPose;
    //tf::poseStampedMsgToTF(*msg, nextPose);
    //RobotArm::getInstance(0)->time_to_target = 0;
    //RobotArm::getInstance(0)->move_ik(nextPose,0);
}


void poseCallbackL(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

    //ROS_INFO("seq %i stamp %i.%i", msg->header.seq,msg->header.stamp.sec, msg->header.stamp.nsec );
    std::cout << "." << std::flush;
    if ((msg->header.stamp - lastMessageL) < ros::Duration(.1))
    {
      //  ROS_ERROR("too young");
        return;
    }

    if ((ros::Time::now() - msg->header.stamp) > ros::Duration(0.5))
    {
        ROS_ERROR("too old %i.%i", (ros::Time::now() - msg->header.stamp).sec, (ros::Time::now() - msg->header.stamp).nsec );
        return;
    }

    lastMessageL = msg->header.stamp;

    double start_angles[7];
    double solutions[7];
    RobotArm::getInstance(1)->getJointState(start_angles);

    tf::Stamped<tf::Pose> nextPose;
    tf::poseStampedMsgToTF(*msg, nextPose);

    tf::Stamped<tf::Pose> actPose = RobotArm::getInstance(1)->getToolPose("map");
    tf::Stamped<tf::Pose> nxtPose = RobotArm::getInstance(1)->getPoseIn("map",nextPose);
    float dist = (actPose.getOrigin() - nxtPose.getOrigin()).length();

    float dist_limit = 0.5;

    if (dist > dist_limit) {
        btVector3 act = actPose.getOrigin();
        btVector3 rel = nxtPose.getOrigin() - actPose.getOrigin();
        ROS_INFO("BFR REL LENGTH : %f", rel.length());
        float dfactor = dist_limit / rel.length();
        rel *= dfactor;
        ROS_INFO("AFT REL LENGTH : %f", rel.length());
        nxtPose.setOrigin(rel + act);
        dist = dist_limit;
    }

    nextPose = nxtPose;

    float timeToTarget = dist / 0.1; // 50 cm per sec

    //ROS_INFO(". dist %f timetotarg %f", dist, timeToTarget);

    nextPose = RobotArm::getInstance(1)->tool2wrist(nextPose);
    geometry_msgs::PoseStamped inWrist;
    tf::poseStampedTFToMsg(nextPose,inWrist);

    bool ik_res = RobotArm::getInstance(1)->run_ik(inWrist, start_angles,solutions,"l_wrist_roll_link");

    printPose("target pose: ", nextPose);

    if (!ik_res)  {
        ROS_ERROR("outside bounds");
        return;
    }

    float solutionsf[7]; for (int i=0;i<7;++i) solutionsf[i] = solutions[i];
    RobotArm::getInstance(1)->startTrajectory(RobotArm::getInstance(1)->goalTraj(solutionsf, timeToTarget),false);

    //tf::Stamped<tf::Pose> nextPose;
    //tf::poseStampedMsgToTF(*msg, nextPose);
    //RobotArm::getInstance(0)->time_to_target = 0;
    //RobotArm::getInstance(0)->move_ik(nextPose,0);
}


int main(int argc, char** argv)
{
    // Init the ROS node
    ros::init(argc, argv, "CartesianViaJoint");



    ros::NodeHandle node_handle;

    lastMessageR = ros::Time::now();
    lastMessageL = ros::Time::now();

    ros::Subscriber subsRight = node_handle.subscribe("/r_cartesian_via_joint/command", 1, &poseCallbackR);
    ros::Subscriber subsLeft = node_handle.subscribe("/l_cartesian_via_joint/command", 1, &poseCallbackL);

    ROS_INFO("READY");

    ros::spin();
}

