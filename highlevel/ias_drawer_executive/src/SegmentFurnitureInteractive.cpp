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
#include <ias_drawer_executive/OpenContainerAction.h>
#include <ias_drawer_executive/CloseContainerAction.h>
#include <ias_drawer_executive/OperateHandleController.h>
#include <ias_drawer_executive/RobotDriver.h>
#include <ias_drawer_executive/RobotArm.h>
#include <ias_drawer_executive/Torso.h>
#include <ias_drawer_executive/Perception3d.h>

//for trajectory publishing
#include <visualization_msgs/MarkerArray.h>

bool getParam(std::string param_name)
{
  ros::NodeHandle nh;
  bool segment;
  nh.getParam(param_name, segment);
  return segment;
}

void setParam(std::string param_name, bool segment)
{
  ros::NodeHandle nh;
  nh.setParam(param_name, segment);
}


void test_open(int arm_, float x, float y, float z, float ox, float oy, float oz, float ow)
{
  //publish trajectory as arrow marker array
  ros::Publisher trajectory_marker_array_publisher, trajectory_marker_publisher;
  ros::NodeHandle nh;
  //advertise as latched
  trajectory_marker_publisher=nh.advertise<visualization_msgs::Marker>("trajectory_marker", 100, true);
  trajectory_marker_array_publisher = nh.advertise<visualization_msgs::MarkerArray>("trajectory_marker_array", 100, true);

    OperateHandleController::plateTuckPose();


    //boost::thread tors(boost::bind(&Torso::pos,Torso::getInstance(),(z < 0.5) ? 0.1f : 0.29f));
    //   Torso::down();

    //go to initial scan pose
    RobotDriver::getInstance()->moveBaseP(x -0.5,y + (arm_  ? -.2 : .2),0,1);
    // sleep(10);
    // //this is to get the first cloud into segment_differences node
    // setParam("/segment_difference_interactive/take_first_cloud", true);
    // while (getParam("/segment_difference_interactive/take_first_cloud"))
    //   {
    // 	sleep(1);
    //   }

    RobotDriver::getInstance()->moveBaseP(x -.6,y + (arm_  ? -.2 : .2),0,1);
    //tors.join();

  //open the drawer
    actionlib::SimpleActionClient<ias_drawer_executive::OpenContainerAction> ac("open_container_action", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    std::cerr << "before action" << std::endl;
    ac.waitForServer(); //will wait for infinite time
    std::cerr << "after action" << std::endl;

    ROS_INFO("Action server started, sending goal.");


    tf::Stamped<tf::Pose> handleHint;
    handleHint.setOrigin(btVector3( x, y, z));
    handleHint.frame_id_ = "/map";
    tf::Stamped<tf::Pose> handlePos = Perception3d::getHandlePoseFromLaser(handleHint);
    //handlePos = OperateHandleController::getHandlePoseFromLaser(handleHint);
    handlePos.setRotation(btQuaternion(ox, oy, oz, ow));

    // send a goal to the action
    ias_drawer_executive::OpenContainerGoal goal;
    goal.arm = arm_;
    goal.position.header.frame_id = "map";
    goal.position.pose.position.x = handlePos.getOrigin().x();
    goal.position.pose.position.y = handlePos.getOrigin().y();
    goal.position.pose.position.z = handlePos.getOrigin().z();
    goal.position.pose.orientation.x = handlePos.getRotation().x();
    goal.position.pose.orientation.y = handlePos.getRotation().y();
    goal.position.pose.orientation.z = handlePos.getRotation().z();
    goal.position.pose.orientation.w = handlePos.getRotation().w();

    ac.sendGoal(goal);

    //wait if openning of drawer succeeded
    ROS_INFO("waiting for openning of drawer");
    bool finished_before_timeout = ac.waitForResult(ros::Duration(1000.0));


    //move base backward
    ROS_INFO("moving base backward");
    float target[4];
    target[0] = -0.1;
    target[1] = -0.1;
    target[2] = 0;
    target[3] = 1;
    ROS_INFO("POSE IN BASE %f %f %f", target[0], target[1], target[2]);
    RobotDriver::getInstance()->driveInOdom(target, 1);

    //get arm pose
    tf::Stamped<tf::Pose> p0 = RobotArm::getInstance(1)->getToolPose("base_link");

    //park the arm to the side
    ROS_INFO("parking the arm");
    OperateHandleController::plateTuckPose();

    //got to the same initial position and scan
    RobotDriver::getInstance()->moveBaseP(x -1.0,y + (arm_  ? -.2 : .2),0,1);
    //sleep(11);

    //call "segment differences" service
    ROS_INFO("calling the segment-difference service");
    setParam("/segment_difference_interactive/segment", true);
    // while (getParam("/segment_difference_interactive/segment"))
    // {
    //   sleep(1);
    // }

    //publish the marker
    visualization_msgs::MarkerArray marker;
    boost::shared_ptr<const ias_drawer_executive::OpenContainerResult> result = ac.getResult();
    ROS_INFO("Trajectory: %i Poses", (unsigned int) result->trajectory.poses.size() );
    marker.markers.resize(result->trajectory.poses.size());
    for (size_t j=0; j < result->trajectory.poses.size(); ++j)
      {
	    ROS_INFO(" Pos %f %f %f, %f %f %f %f", result->trajectory.poses[j].position.x,result->trajectory.poses[j].position.y,result->trajectory.poses[j].position.z,
		result->trajectory.poses[j].orientation.x,result->trajectory.poses[j].orientation.y,result->trajectory.poses[j].orientation.z,result->trajectory.poses[j].orientation.w);
	    marker.markers[j].header.frame_id = result->trajectory.header.frame_id;
    	marker.markers[j].header.stamp=ros::Time::now();
    	marker.markers[j].type = visualization_msgs::Marker::ARROW;
    	marker.markers[j].action = visualization_msgs::Marker::ADD;
    	marker.markers[j].ns="trajectory";
    	marker.markers[j].pose= result->trajectory.poses[j];
    	marker.markers[j].scale.x=0.1;
    	marker.markers[j].scale.y=1.0;
    	marker.markers[j].id = j;
    	marker.markers[j].color.r = 0.0f;
    	marker.markers[j].color.g = 1.0f;
    	marker.markers[j].color.b = 0.0f;
    	marker.markers[j].color.a = 1.0f;
    	marker.markers[j].lifetime = ros::Duration::Duration();
      }
    ROS_INFO("publishing marker array for goto poses");
    trajectory_marker_array_publisher.publish(marker);

    //move arm to pose
    //RobotArm::getInstance(0)->move_toolframe_ik_pose(p0);
    //int handle = OperateHandleController::operateHandle(0,p0);

    //move base forwward
    // ROS_INFO("moving base forward");
    // target[0] = -target[0];
    // ROS_INFO("POSE IN BASE %f %f %f", target[0], target[1], target[2]);
    // RobotDriver::getInstance()->driveInOdom(target, 1);

    //close the drawer
    ROS_INFO("closing the drawer");
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
        //boost::shared_ptr<const ias_drawer_executive::OpenContainerResult> result = ac.getResult();
        //ias_drawer_executive::OpenContainerResult result = ac.getResult();
        ROS_INFO("sucess %i", result->success);
        ROS_INFO("Trajectory: %i Poses", (unsigned int) result->trajectory.poses.size() );
        for (size_t j=0; j < result->trajectory.poses.size(); ++j)
        {
            ROS_INFO(" Pos %f %f %f, %f %f %f %f", result->trajectory.poses[j].position.x,result->trajectory.poses[j].position.y,result->trajectory.poses[j].position.z,
                     result->trajectory.poses[j].orientation.x,result->trajectory.poses[j].orientation.y,result->trajectory.poses[j].orientation.z,result->trajectory.poses[j].orientation.w);
        }

        ROS_INFO("TRYING NOW TO CLOSE");

        actionlib::SimpleActionClient<ias_drawer_executive::CloseContainerAction> ac_close("close_container_action", true);

        ROS_INFO("Waiting for action server to start.");
        // wait for the action server to start
        ac_close.waitForServer(); //will wait for infinite time

        ROS_INFO("Action server started, sending goal.");

        ias_drawer_executive::CloseContainerGoal closeGoal;
        closeGoal.arm = arm_;
        closeGoal.opening_trajectory = result->trajectory;

        ac_close.sendGoal(closeGoal);

        bool finished_before_timeout = ac_close.waitForResult(ros::Duration(1000.0));

        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac_close.getState();
            ROS_INFO("Action finished: %s",state.toString().c_str());
        }

        OperateHandleController::plateTuckPose();
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
   actionlib::SimpleActionClient<ias_drawer_executive::CloseContainerAction> ac_close("close_container_action", true);

     ROS_INFO("Waiting for action server to start.");
        // wait for the action server to start
        ac_close.waitForServer(); //will wait for infinite time

        ROS_INFO("Action server started, sending goal.");

        ias_drawer_executive::CloseContainerGoal closeGoal;
        closeGoal.arm = 0;

        closeGoal.opening_trajectory.poses.push_back(*getPose( 0.346634, 1.118203, 0.763804, -0.708801, -0.055698, 0.039966, 0.702069));
        closeGoal.opening_trajectory.poses.push_back(*getPose( 0.509877, 1.121905, 0.761193, -0.705817, -0.039952, 0.034062, 0.706447));
        closeGoal.opening_trajectory.poses.push_back(*getPose( 0.710390, 1.128074, 0.761932, -0.705345, -0.038818, 0.035855, 0.706892));

        ac_close.sendGoal(closeGoal);

        bool finished_before_timeout = ac_close.waitForResult(ros::Duration(1000.0));

        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac_close.getState();
            ROS_INFO("Action finished: %s",state.toString().c_str());
        }
}

void test_open(int arm, btTransform &t)
{
    test_open(arm, t.getOrigin().x(), t.getOrigin().y(), t.getOrigin().z(), t.getRotation().x(),t.getRotation().y(),t.getRotation().z(),t.getRotation().w());
}

int main (int argc, char **argv)
{

    ros::init(argc, argv, "segment_difference_interactive");

    //closeonly();

    std::vector<btTransform> hints(100);
    //hints.push_back(btTransform(btQuaternion(1.000, 0.008, -0.024, -0.004) ,btVector3(0.762, 2.574, 0.791)); // 00

    hints[0] = btTransform(btQuaternion(1.000, 0.008, -0.024, -0.004), btVector3(0.762, 2.574, 0.791)); // 0
    hints[10] = btTransform(btQuaternion(-0.691, -0.039, 0.040, 0.721), btVector3(0.799, 2.137, 1.320)); // 10
    hints[11] = btTransform(btQuaternion(-0.715, -0.017, -0.003, 0.699), btVector3(0.777, 2.126, 0.804)); // 11
    hints[12] = btTransform(btQuaternion(-0.741, -0.029, -0.020, 0.671), btVector3(0.777, 2.125, 0.662)); // 12
    hints[20] = btTransform(btQuaternion(0.025, -0.002, 0.039, 0.999), btVector3(0.798, 1.678, 0.805)); // 20
    hints[30] = btTransform(btQuaternion(0.712, 0.003, 0.025, 0.702), btVector3(0.629, 1.126, 0.756)); // 30
    hints[31] = btTransform(btQuaternion(0.712, 0.027, 0.019, 0.701), btVector3(0.627, 1.140, 0.614)); // 31
    hints[32] = btTransform(btQuaternion(0.718, 0.031, 0.019, 0.695), btVector3(0.623, 1.135, 0.331)); // 32
    hints[40] = btTransform(btQuaternion(0.698, 0.004, 0.056, 0.714) ,btVector3(0.654, 0.442, 0.766)); // 40
    hints[50] = btTransform(btQuaternion(0.704, -0.035, 0.027, 0.709) ,btVector3(0.705, -0.171, 0.614)); // 50
    hints[60] = btTransform(btQuaternion(-0.029, -0.012, 0.020, 0.999) ,btVector3(0.882, -0.520, 0.985)); // 60
    hints[61] = btTransform(btQuaternion(-0.688, -0.015, 0.045, 0.724) ,btVector3(0.881, -0.776, 0.437)); // 61

    //test_open(atoi(argv[1]),atof(argv[2]),atof(argv[3]),atof(argv[4]),atof(argv[5]),atof(argv[6]),atof(argv[7]),atof(argv[8]));
    test_open(0, hints[atoi(argv[1])]);

    // create the action client
    // true causes the client to spin it's own thread
    //exit
    return 0;
}


/*
*/
