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

#include <boost/thread.hpp>


#include <ias_drawer_executive/OperateHandleController.h>
#include <ias_drawer_executive/RobotArm.h>
#include <ias_drawer_executive/Geometry.h>
#include <ias_drawer_executive/Gripper.h>
#include <ias_drawer_executive/Pressure.h>
#include <ias_drawer_executive/Poses.h>
#include <ias_drawer_executive/AverageTF.h>
#include <ias_drawer_executive/Torso.h>
#include <ias_drawer_executive/Approach.h>

#include <cop_client_cpp/cop_client.h>


void Approach::init(int side, tf::Stamped<tf::Pose> approachStart, tf::Stamped<tf::Pose> plateCenter, Approach::SensorArea sensors)
{
    sensors_ = sensors;

    ros::service::call((side_==0) ? "/r_gripper_sensor_controller/update_zeros" : "/l_gripper_sensor_controller/update_zeros", serv);

    side_ = side;
    arm = RobotArm::getInstance(side);
    pressure = Pressure::getInstance(side);

    touched = false;
    firstTouch = 0;

    approachStart.stamp_=ros::Time();
    plateCenter.stamp_=ros::Time();

    startPose = Geometry::getPoseIn("map",approachStart);

    plateCenter=Geometry::getPoseIn("map",plateCenter);
    //plateCenter.setOrigin(plateCenter.getOrigin() + btVector3(0,side ? -.15 : .15,0.0f));
    diff = plateCenter.getOrigin() - startPose.getOrigin(); // we define a line (pMa, diff) in map frame here that is used for the approach
    //like that: p.setOrigin(pMa.getOrigin() + diff * ap);
    ROS_INFO("DIFF %f %f %f", diff.x(), diff.y(), diff.z());
    // approach
    arm->time_to_target = 10;

    pressureDiff = 1000;
}

double Approach::increment(double st, double ap)
{

    tf::Stamped<tf::Pose> p = startPose;
    p.setOrigin(startPose.getOrigin() + diff * ap);

    tf::Stamped<tf::Pose> curr = startPose;
    curr.setOrigin(startPose.getOrigin() + diff * st);

    //arm->moveElbowOutOfWay(curr);

    ROS_INFO("START SIDE %i , %f %f %f, %f %f %f %f",side_,startPose.getOrigin().x(),startPose.getOrigin().y(),startPose.getOrigin().z(),
             startPose.getRotation().x(),startPose.getRotation().y(),startPose.getRotation().z(),startPose.getRotation().w());
    ROS_INFO("GOAL SIDE %i , %f %f %f, %f %f %f %f",side_,p.getOrigin().x(),p.getOrigin().y(),p.getOrigin().z(),
             p.getRotation().x(),p.getRotation().y(),p.getRotation().z(),p.getRotation().w());

    p = Geometry::getPoseIn("base_link",p);

    //arm->time_to_target = 1;

    //arm->move_toolframe_ik(startPose.getOrigin().x(),startPose.getOrigin().y(),startPose.getOrigin().z(),
    //   startPose.getRotation().x(),startPose.getRotation().y(),startPose.getRotation().z(),startPose.getRotation().w());

    arm->time_to_target = 10;

    arm->retries = 3;

    boost::thread t1(&RobotArm::move_toolframe_ik,arm,p.getOrigin().x(),p.getOrigin().y(),p.getOrigin().z(),
                     p.getRotation().x(),p.getRotation().y(),p.getRotation().z(),p.getRotation().w());

    ros::Rate rate(40.0);
    double pl, pr;
    //pressure->getInside(pl,pr,false);
    switch (sensors_)
    {
    case Approach::inside :
        pressure->getInside(pl,pr, false);
        break;
    case Approach::front :
        pressure->getFront(pl,pr, false);
        break;
    default:
        ROS_ERROR("Approach::init got an unknown value %i for sensors", sensors_);
    }

    while (ros::ok() && arm->getActionClient()->getState() !=  actionlib::SimpleClientGoalState::ACTIVE)
    {
        ros::spinOnce();
        rate.sleep();
    }

    double tl,tr;

    switch (sensors_)
    {
    case Approach::inside :
        pressure->getInsideTouched(tl,tr);
        break;
    case Approach::front :
        pressure->getFrontTouched(tl,tr);
        break;
    default:
        ROS_ERROR("Approach::init got an unknown value %i for sensors", sensors_);
    }


    std::vector<double> distances;

    double sumavgs = 0;
    double numavgs = 0;

    bool touched = false;
    int cart_err_hit = 0;
    while (ros::ok()  && (arm->getActionClient()->getState() ==  actionlib::SimpleClientGoalState::ACTIVE))
    {
        double cl, cr;
        //pressure->getInside(cl,cr,false);
        switch (sensors_)
        {
        case Approach::inside :
            pressure->getInsideTouched(cl,cr);
            break;
        case Approach::front :
            pressure->getFrontTouched(cl,cr);
            break;
        default:
            ROS_ERROR("Approach::init got an unknown value %i for sensors", sensors_);
        }
        // ROS_INFO("STATUS %s", arm->getActionClient()->getState().toString().c_str());
        ROS_INFO("TOUCHES %i %f", side_,  (cl + cr) - (tl + tr));
        btVector3 cartErr = arm->cartError();


        btVector3 direction = (plateCenter.getOrigin() - startPose.getOrigin()).normalize();
        double projected = cartErr.dot(direction);

        distances.push_back(projected);

        double avg = 0;

        if (distances.size() > 5) {
            double distsum = 0;
            for (int k = distances.size() - 6; k < distances.size(); ++k) {
                distsum += distances[k];
            }
            distsum /= 5;
            avg = distsum;
            sumavgs += fabs(avg);
            numavgs += 1;
        }

        double relative = 0;
        if (numavgs > 2) {
            double sumnum = sumavgs / numavgs;
            relative = avg / sumnum;
            ROS_ERROR("%f / \t %f = \t %f", avg ,sumnum, avg / sumnum);
        }


        ROS_INFO("CArt error : magn %f \t %f %f %f proj : \t %f \t avg: %f ", cartErr.length(), cartErr.x(), cartErr.y(), cartErr.z(), projected, avg);

        if (relative < -3)
          cart_err_hit++;

        if ((cl + cr > tl + tr + 5) || (cart_err_hit > 3)) // || cartErr.length() > 0.001)
        {
            ROS_INFO("SIDE %i TOUCHED press %i", side_, pressure->side_);
            ROS_INFO("TOUCHES %i %f", side_,  (cl + cr) - (tl + tr));
            arm->getActionClient()->cancelGoal();
            touched = true;
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    tf::Stamped<tf::Pose> actPose;
    arm->getToolPose(actPose,"map");

    double actDiff = btVector3(startPose.getOrigin() - actPose.getOrigin()).length();
    ROS_INFO("DISTANCE FROM START %f" , actDiff);

    arm->retries = 0;

    if (touched)
        return actDiff;
    else
        return 0;

}

void Approach::move_to(double ap)
{

    tf::Stamped<tf::Pose> p = startPose;
    p.setOrigin(startPose.getOrigin() + diff * ap);

    arm->time_to_target = 1;

    arm->universal_move_toolframe_ik(p.getOrigin().x(),p.getOrigin().y(),p.getOrigin().z(),
                                     p.getRotation().x(),p.getRotation().y(),p.getRotation().z(),p.getRotation().w(),"map");
}


bool Approach::finish()
{
    arm->time_to_target = 1;

    //ros::service::call((side_==0) ? "/r_reactive_grasp/compliant_close" : "/l_reactive_grasp/compliant_close", serv);

    //ros::service::call("/r_reactive_grasp/compliant_close" , serv);
    Gripper::getInstance(side_)->closeCompliant();

    Gripper::getInstance(side_)->close();
    return false;
}

void Lift::init(int side)
{
    side_ = side;
    arm = RobotArm::getInstance(side);

    tf::Stamped<tf::Pose> actPose;
    const char fixed_frame[] = "torso_lift_link";
    arm->getToolPose(actPose,fixed_frame);

    // for the case that we change the elbow angle, reposition the tool at current pose with given elbow angle
    //for popcorn: arm->universal_move_toolframe_ik_pose(actPose);
    //Pressure *pressure = Pressure::getInstance(side);
    //pressure->reset();

    plateCenter= actPose;
    plateCenter.setOrigin(plateCenter.getOrigin() + btVector3(0,0,1));
    diff = plateCenter.getOrigin() - actPose.getOrigin(); // we define a line (pMa, diff) in map frame here that is used for the approach
    diffToRob = btVector3(-1,0,0);
    //like that: p.setOrigin(pMa.getOrigin() + diff * ap);
    startPose = actPose;
    // approach
    // arm->time_to_target = 0.25;
    arm->time_to_target = 1;
}

//void Lift::increment(double up, double back)
void Lift::increment(double x, double y, double z)
{
    ros::spinOnce();
    tf::Stamped<tf::Pose> p = startPose;
    //p.setOrigin(startPose.getOrigin() + diff * z + diffToRob * back);
    p.setOrigin(startPose.getOrigin() + btVector3(x,y,z));
    //arm->universal_move_toolframe_ik_pose(p);
    arm->move_toolframe_ik_pose(p);
}

