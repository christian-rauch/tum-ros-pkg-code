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

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <ias_drawer_executive/Head.h>

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

RobotHead *RobotHead::instance_ = 0;

RobotHead::RobotHead()
{
    //Initialize the client for the Action interface to the head controller
    point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);

    //wait for head controller action server to come up
    while (!point_head_client_->waitForServer(ros::Duration(5.0)) && ros::ok())
    {
        ROS_INFO("Waiting for the point_head_action server to come up");
    }

    stop = true;

    t1 = 0;
}

RobotHead::~RobotHead()
{
    delete point_head_client_;
}

//! Points the high-def camera frame at a point in a given frame
void RobotHead::lookAt(std::string frame_id, double x, double y, double z, bool waitfor)
{

    if (t1)
       stopThread();
    //the goal message we will be sending
    pr2_controllers_msgs::PointHeadGoal goal;

    //the target point, expressed in the requested frame
    geometry_msgs::PointStamped point;
    point.header.frame_id = frame_id;
    point.point.x = x;
    point.point.y = y;
    point.point.z = z;
    goal.target = point;

    //we are pointing the wide_stereo camera frame
    //(pointing_axis defaults to X-axis)
    goal.pointing_frame = "narrow_stereo_optical_frame";

    //take at least 5 seconds to get there
    goal.min_duration = ros::Duration(2);

    //and go no faster than 0.1 rad/s
    goal.max_velocity = 0.5;

    //send the goal
    point_head_client_->sendGoal(goal);

    //wait for it to get there
    if (waitfor)
        point_head_client_->waitForResult();
}


void RobotHead::spinner(std::string frame_id, double x, double y, double z, double rate)
{
    ros::Rate rt(rate);

    while (!stop && ros::ok())
    {
        rt.sleep();
        //the goal message we will be sending
        pr2_controllers_msgs::PointHeadGoal goal;

        //the target point, expressed in the requested frame
        geometry_msgs::PointStamped point;
        point.header.frame_id = frame_id;
        point.point.x = x;
        point.point.y = y;
        point.point.z = z;
        goal.target = point;

        //we are pointing the wide_stereo camera frame
        //(pointing_axis defaults to X-axis)
        goal.pointing_frame = "narrow_stereo_optical_frame";

        //take at least 5 seconds to get there
        goal.min_duration = ros::Duration(0.3);

        //and go no faster than 0.1 rad/s
        goal.max_velocity = 2.5;

        //send the goal
        point_head_client_->sendGoal(goal);
    }

    ROS_INFO("SPINNER STOPPED");

    t1 = 0;

}

//! Points the high-def camera frame at a point in a given frame
void RobotHead::lookAtThreaded(std::string frame_id, double x, double y, double z, bool waitfor)
{
    if (t1)
       stopThread();
    stop = false;
    t1 = new boost::thread(&RobotHead::spinner, this, frame_id, x, y, z, 15);
}

void RobotHead::stopThread()
{
    ros::Rate rt(10);
    stop = true;
    while (t1) {
        rt.sleep();
    }
}

