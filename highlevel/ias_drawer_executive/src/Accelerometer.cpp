#include <perception_for_manipulation/Accelerometer.h>

Accelerometer::Accelerometer(int side)
{
    side_=side;
    acc_client_=new DetectContactAC(side ? "l_gripper_sensor_controller/event_detector" : "r_gripper_sensor_controller/event_detector",true);
    ROS_INFO("waiting for accelerometer AC server to come up for side %d",side);
    acc_client_->waitForServer(ros::Duration(5.0));
}


Accelerometer::~Accelerometer()
{
    delete acc_client_;
}


Accelerometer *Accelerometer::instance_[] = {0,0};


Accelerometer *Accelerometer::getInstance(int side)
{
    if (!instance_[side]) instance_[side] = new Accelerometer(side);
    return instance_[side];
}


void Accelerometer::detectContact(double magnitude)
{
    pr2_gripper_sensor_msgs::PR2GripperEventDetectorGoal detect_goal;
    detect_goal.command.trigger_conditions=pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC;//detect_goal.command.trigger_conditions.ACC;
    detect_goal.command.acceleration_trigger_magnitude=magnitude;

    ROS_INFO("sending detect_acc goal");
    acc_client_->sendGoal(detect_goal);
    acc_client_->waitForResult(ros::Duration(100.0));
    if (acc_client_->getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("detect_acc succeeded");
        else
        ROS_INFO("detect_acc failed");
}
