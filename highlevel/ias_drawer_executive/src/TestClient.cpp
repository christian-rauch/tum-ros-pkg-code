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
#include <actionlib/client/terminal_state.h>
#include <ias_drawer_executive/OperateHandleAction.h>
#include <ias_drawer_executive/OpenContainerAction.h>
#include <ias_drawer_executive/CloseContainerAction.h>

#include <signal.h>


/*void test_op()
{

    if (!ac_)
       ac_ = new actionlib::SimpleActionClient<ias_drawer_executive::OperateHandleAction>("operate_handle_action", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac_->waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");

    // send a goal to the action
    ias_drawer_executive::OperateHandleGoal goal;
    goal.arm = 0;
    goal.positionIdx= 8;
    goal.heightIdx= 3;
    ac_->sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac_->waitForResult(ros::Duration(1000.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac_->getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

}*/


actionlib::SimpleActionClient<ias_drawer_executive::OpenContainerAction> *ac_ = 0;

actionlib::SimpleActionClient<ias_drawer_executive::CloseContainerAction> *ac_close_ = 0;

void test_open(int arm_, float x, float y, float z, float ox, float oy, float oz, float ow)
{
    if (!ac_)
      ac_ = new actionlib::SimpleActionClient<ias_drawer_executive::OpenContainerAction>("open_container_action", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac_->waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");

    // send a goal to the action
    ias_drawer_executive::OpenContainerGoal goal;
    goal.arm = arm_;
    goal.position.header.frame_id = "map";
    goal.position.pose.position.x = x;
    goal.position.pose.position.y = y;
    goal.position.pose.position.z = z;
    goal.position.pose.orientation.x = ox;
    goal.position.pose.orientation.y = oy;
    goal.position.pose.orientation.z = oz;
    goal.position.pose.orientation.w = ow;

    ac_->sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac_->waitForResult(ros::Duration(1000.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac_->getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
        boost::shared_ptr<const ias_drawer_executive::OpenContainerResult> result = ac_->getResult();
        //ias_drawer_executive::OpenContainerResult result = ac_->getResult();
        ROS_INFO("sucess %i", result->success);
        ROS_INFO("Trajectory: %i Poses", (unsigned int) result->trajectory.poses.size() );
        for (size_t j=0; j < result->trajectory.poses.size(); ++j)
        {
            ROS_INFO(" Pos %f %f %f, %f %f %f %f", result->trajectory.poses[j].position.x,result->trajectory.poses[j].position.y,result->trajectory.poses[j].position.z,
                     result->trajectory.poses[j].orientation.x,result->trajectory.poses[j].orientation.y,result->trajectory.poses[j].orientation.z,result->trajectory.poses[j].orientation.w);
        }

        /*int32 success
        float32 distance
        geometry_msgs/PoseArray trajectory
        Header header
        uint32 seq
        time stamp
        string frame_id
        geometry_msgs/Pose[] poses
        geometry_msgs/Point position
          float64 x
          float64 y
          float64 z
        geometry_msgs/Quaternion orientation
          float64 x
          float64 y
          float64 z
          float64 w*/
        ROS_INFO("TRYING NOW TO CLOSE");

        if (!ac_close_)
           ac_close_ = new actionlib::SimpleActionClient<ias_drawer_executive::CloseContainerAction>("close_container_action", true);

        ROS_INFO("Waiting for action server to start.");
        // wait for the action server to start
        ac_close_->waitForServer(); //will wait for infinite time

        ROS_INFO("Action server started, sending goal.");

        ias_drawer_executive::CloseContainerGoal closeGoal;
        closeGoal.arm = arm_;
        closeGoal.opening_trajectory = result->trajectory;

        ac_close_->sendGoal(closeGoal);

        bool finished_before_timeout = ac_close_->waitForResult(ros::Duration(1000.0));

        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac_close_->getState();
            ROS_INFO("Action finished: %s",state.toString().c_str());
        }
    }
    else
        ROS_INFO("Action did not finish before the time out.");
}

geometry_msgs::Pose *getPose(float x, float y, float z, float ox, float oy, float oz, float ow)
{
        geometry_msgs::Pose *act = new geometry_msgs::Pose();
        act->position.x = x;
        act->position.y = y;
        act->position.z = z;
        act->orientation.x = ox;
        act->orientation.y = oy;
        act->orientation.z = oz;
        act->orientation.w = ow;
        return act;
}




void closeonly() {
      if (!ac_close_)
        ac_close_ = new actionlib::SimpleActionClient<ias_drawer_executive::CloseContainerAction>("close_container_action", true);

     ROS_INFO("Waiting for action server to start.");
        // wait for the action server to start
        ac_close_->waitForServer(); //will wait for infinite time

        ROS_INFO("Action server started, sending goal.");

        ias_drawer_executive::CloseContainerGoal closeGoal;
        closeGoal.arm = 0;

        closeGoal.opening_trajectory.poses.push_back(*getPose( 0.346634, 1.118203, 0.763804, -0.708801, -0.055698, 0.039966, 0.702069));
        closeGoal.opening_trajectory.poses.push_back(*getPose( 0.509877, 1.121905, 0.761193, -0.705817, -0.039952, 0.034062, 0.706447));
        closeGoal.opening_trajectory.poses.push_back(*getPose( 0.710390, 1.128074, 0.761932, -0.705345, -0.038818, 0.035855, 0.706892));

        ac_close_->sendGoal(closeGoal);

        bool finished_before_timeout = ac_close_->waitForResult(ros::Duration(1000.0));

        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac_close_->getState();
            ROS_INFO("Action finished: %s",state.toString().c_str());
        }
}

/*
[ INFO] [1298048988.832717783]: TRAJ PT 10, 0.710390 1.128074 0.761932, -0.705345 -0.038818 0.035855 0.706892
[ INFO] [1298048988.832773299]: TRAJ PT 10, 0.660410 1.126327 0.761844, -0.705351 -0.038682 0.035990 0.706887
[ INFO] [1298048988.832815850]: TRAJ PT 10, 0.613501 1.124731 0.760803, -0.704437 -0.041014 0.035059 0.707713
[ INFO] [1298048988.832854815]: TRAJ PT 10, 0.558759 1.122694 0.760463, -0.704835 -0.040498 0.034604 0.707369
[ INFO] [1298048988.832893431]: TRAJ PT 10, 0.509877 1.121905 0.761193, -0.705817 -0.039952 0.034062 0.706447
[ INFO] [1298048988.832932724]: TRAJ PT 10, 0.459795 1.119149 0.761111, -0.705787 -0.039522 0.033228 0.706540
[ INFO] [1298048988.833147869]: TRAJ PT 10, 0.410610 1.116605 0.760985, -0.706038 -0.039084 0.032569 0.706344
[ INFO] [1298048988.833254194]: TRAJ PT 10, 0.360722 1.114320 0.760970, -0.706072 -0.038935 0.032037 0.706343
[ INFO] [1298048988.833302466]: TRAJ PT 10, 0.346634 1.118203 0.763804, -0.708801 -0.055698 0.039966 0.702069
[ INFO] [1298048988.833347079]: TRAJ PT 10, 0.336214 1.120235 0.764419, -0.699980 -0.071769 0.059650 0.708039
*/

void quit(int sig)
{
    if (ac_)
      ac_->cancelAllGoals();
    if (ac_close_)
      ac_close_->cancelAllGoals();
    exit(0);
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "test_fibonacci");

    signal(SIGINT,quit);

    //closeonly();

    //test_open(atoi(argv[1]),atof(argv[2]),atof(argv[3]),atof(argv[4]),atof(argv[5]),atof(argv[6]),atof(argv[7]),atof(argv[8]));


    // create the action client
    // true causes the client to spin it's own thread
    //exit
    return 0;
}
