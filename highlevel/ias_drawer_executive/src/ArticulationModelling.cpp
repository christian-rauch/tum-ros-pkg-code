


#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <articulation_pr2/ArticulatedObjectAction.h>
#include <actionlib/client/simple_action_client.h>


#include <ias_drawer_executive/ArticulationModelling.h>
#include <ias_drawer_executive/Gripper.h>
#include <ias_drawer_executive/RobotArm.h>

typedef actionlib::SimpleActionClient<articulation_pr2::ArticulatedObjectAction> Client;

int articulate()
{
    Client client("articulate_object", true); // true -> don't need ros::spin()
    client.waitForServer();
    {
        articulation_pr2::ArticulatedObjectGoal goal;
        goal.timeout = ros::Duration(20.00);
        goal.initial_pulling_direction.header.frame_id = "torso_lift_link";
        goal.initial_pulling_direction.vector.x = -1;// pull towards robot
        goal.desired_velocity = 0.04;

        // Fill in goal here
        client.sendGoal(goal);
        client.waitForResult(ros::Duration(5.0));
        while (client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            //if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            printf("Current State: %s\n", client.getState().toString().c_str());
            ros::Duration(0.5).sleep();
        }

        articulation_pr2::ArticulatedObjectResultConstPtr result = client.getResult();

        ROS_INFO("FRAME : %s", result->posterior_model.track.header.frame_id.c_str());
        //std::vector<geometry_msgs::Pose>::iterator pose_it;
        //for (pose_it = result->posterior_model.track.pose_projected.begin(); pose_it != result->posterior_model.track.pose_projected.end(); ++pose_it) {
        tf::Stamped<tf::Pose> lastPose;
        lastPose.setOrigin(btVector3(0,0,10000));
        for (int pos_i = 0; pos_i < result->posterior_model.track.pose_resampled.size(); ++pos_i)
        {
            geometry_msgs::Pose act = result->posterior_model.track.pose_resampled[pos_i];
            ROS_INFO("%f %f %f  %f %f %f %f", act.position.x, act.position.y, act.position.z,
                     act.orientation.x,  act.orientation.y,  act.orientation.z,  act.orientation.w);
            tf::Pose actTF;
            tf::poseMsgToTF(act,actTF);
            tf::Stamped<tf::Pose> actSTF;
            actSTF.setOrigin(actTF.getOrigin());
            actSTF.setRotation(actTF.getRotation());
            actSTF.frame_id_ = result->posterior_model.track.header.frame_id.c_str();
            if (pos_i == 0)
                Gripper::getInstance(0)->open();
            if (((actSTF.getOrigin() - lastPose.getOrigin()).length() >= 0.05) || (pos_i == result->posterior_model.track.pose_resampled.size() - 1))
            {
                RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(actSTF);
                lastPose = actSTF;
            }

            if (pos_i == 0)
                Gripper::getInstance(0)->close();
        }

        for (int pos_i = result->posterior_model.track.pose_resampled.size() - 1; pos_i >= 0 ; --pos_i)
        {
            geometry_msgs::Pose act = result->posterior_model.track.pose_resampled[pos_i];
            ROS_INFO("%f %f %f  %f %f %f %f", act.position.x, act.position.y, act.position.z,
                     act.orientation.x,  act.orientation.y,  act.orientation.z,  act.orientation.w);
            tf::Pose actTF;
            tf::poseMsgToTF(act,actTF);
            tf::Stamped<tf::Pose> actSTF;
            actSTF.setOrigin(actTF.getOrigin());
            actSTF.setRotation(actTF.getRotation());
            actSTF.frame_id_ = result->posterior_model.track.header.frame_id.c_str();
            if (((actSTF.getOrigin() - lastPose.getOrigin()).length() >= 0.05) || (pos_i == result->posterior_model.track.pose_resampled.size() - 1))
            {
                RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(actSTF);
                lastPose = actSTF;
            }

            if (pos_i == 0)
                Gripper::getInstance(0)->open();
        }



        printf("Yay! The dishes are now clean");
    }
    if (0)
    {
        articulation_pr2::ArticulatedObjectGoal goal;
        goal.timeout = ros::Duration(20.00);
        goal.initial_pulling_direction.header.frame_id = "torso_lift_link";
        goal.initial_pulling_direction.vector.x = 1;// pull towards robot
        goal.desired_velocity = 0.04;

        // Fill in goal here
        client.sendGoal(goal);
        client.waitForResult(ros::Duration(5.0));
        while (client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            //if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            printf("Current State: %s\n", client.getState().toString().c_str());
            ros::Duration(0.5).sleep();
        }
        printf("Yay! The dishes are now clean");
    }

    return 0;
}
