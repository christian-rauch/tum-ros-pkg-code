/* Copyright (c) 2011, MATHE Antal Koppany <mathekoppany@yahoo.com>, All rights reserved. */
/* Copyright (c) 2010, Thomas Ruehr <ruehr@cs.tum.edu>, All rights reserved. */

#ifndef __ACCELEROMETER_H__
#define __ACCELEROMETER_H__

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_gripper_sensor_msgs/PR2GripperEventDetectorAction.h>

typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperEventDetectorAction> DetectContactAC;

class Accelerometer{
private:
    int side_;
    DetectContactAC *acc_client_;
    static Accelerometer *instance_[];

    Accelerometer(int side=0);//side 0 means right hand
    ~Accelerometer();

public:
    static Accelerometer *getInstance(int side=0);
    void detectContact(double magnitude=1.6);
};

#endif
