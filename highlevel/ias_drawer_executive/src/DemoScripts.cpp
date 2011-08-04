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

#include <ias_drawer_executive/RobotDriver.h>
#include <ias_drawer_executive/Gripper.h>
#include <ias_drawer_executive/Perception3d.h>
#include <ias_drawer_executive/Pressure.h>
#include <ias_drawer_executive/Poses.h>
#include <ias_drawer_executive/RobotArm.h>
#include <ias_drawer_executive/AverageTF.h>
#include <ias_drawer_executive/Approach.h>
#include <ias_drawer_executive/Torso.h>
#include <ias_drawer_executive/Head.h>
#include <ias_drawer_executive/OperateHandleController.h>

#include <boost/thread.hpp>

#include <actionlib/client/simple_client_goal_state.h>
#include <visualization_msgs/Marker.h>

#include <ias_drawer_executive/DemoScripts.h>

int DemoScripts::openFridge(int z)
{

    Torso *torso = Torso::getInstance();
    boost::thread t2(&Torso::up, torso);

    OperateHandleController::plateTuckPose();
    RobotDriver::getInstance()->moveBaseP(0.165, -0.247, -0.044, 0.999, false);
    t2.join();

    tf::Stamped<tf::Pose> handleHint;
    handleHint.setOrigin(btVector3( 0.919, -0.553, 1.177 ));
    handleHint.frame_id_ = "/map";
    tf::Stamped<tf::Pose> handlePos = Perception3d::getHandlePoseFromLaser(handleHint);
    //handlePos = OperateHandleController::getHandlePoseFromLaser(handleHint);
    handlePos.setRotation(btQuaternion(0.999, 0.025, 0.025, -0.007));

    // we wait max 60 seconds for the laser handle, then go for map pose
    if (handlePos.getOrigin().z() < 0)
    {
        ROS_ERROR("--------------------------------------------------------------");
        ROS_ERROR("DIDNT GET LASER DETECTED HANDLE, FALLING BACK TO MAP POSE NOW!");
        ROS_ERROR("--------------------------------------------------------------");
        // fallback : map pose
        OperateHandleController::plateTuckPose();
        RobotDriver::getInstance()->moveBaseP(0.165, -0.247, -0.044, 0.999, false);
        tf::Stamped<tf::Pose> handleHint;
        handleHint.setOrigin(btVector3( 0.919, -0.553, 1.177 ));
        handleHint.frame_id_ = "/map";
        //tf::Stamped<tf::Pose> handle = OperateHandleController::getHandlePoseFromLaser(handleHint);
        //handle = OperateHandleController::getHandlePoseFromLaser(handleHint);
        //handle.setRotation(btQuaternion(0.999, 0.025, 0.025, -0.007));

        tf::Stamped<tf::Pose> handleMap;
        handleMap.frame_id_ = "map";
        handleMap.setOrigin(btVector3(0.920, -0.566, 1.174));
        handleMap.setRotation(btQuaternion(0.999, 0.014, -0.030, 0.008));

        handleMap = RobotArm::getInstance()->getPoseIn("base_link", handleMap);

        return OperateHandleController::operateHandle(0,handleMap);
    }
    else
        return OperateHandleController::operateHandle(0, handlePos);
}


int DemoScripts::takeBottle(int z)
{

    pr2_controllers_msgs::JointTrajectoryGoal goalB = RobotArm::getInstance(1)->lookAtMarker(Poses::prepDishL1,Poses::prepDishL1);
    boost::thread t3(&RobotArm::startTrajectory, RobotArm::getInstance(1), goalB,true);


    Torso *torso = Torso::getInstance();
    boost::thread t2(&Torso::up, torso);


    RobotArm::getInstance(0)->tucked = true;
    float p[] = { 0.255, -0.571, -0.025, 1.000};
    //float p[] = { 0.255, -0.571, -0.108, 0.994 };
    RobotDriver::getInstance()->moveBase(p);

    //t2.join();t3.join();

    RobotHead::getInstance()->lookAtThreaded("/map", 1.243111, -0.728864, 0.9);
    tf::Stamped<tf::Pose> bottle = Perception3d::getBottlePose();


    {
        std::vector<int> arm;
        std::vector<tf::Stamped<tf::Pose> > goal;
        btVector3 result;

        tf::Stamped<tf::Pose> p0;
        p0.frame_id_="map";
        p0.stamp_=ros::Time();
        p0.setOrigin(btVector3(0.8, -0.455, 1.251));
        p0.setRotation(btQuaternion(0.005, -0.053, -0.029, 0.998));
        goal.push_back(p0);
        arm.push_back(1);

        tf::Stamped<tf::Pose> p1;
        p1.frame_id_="map";
        p1.stamp_=ros::Time();
        p1.setOrigin(btVector3(0.8, -0.655, 1.251));
        p1.setRotation(btQuaternion(0.005, -0.053, -0.029, 0.998));
        goal.push_back(p1);
        arm.push_back(1);

        tf::Stamped<tf::Pose> p2;
        p2.frame_id_="map";
        p2.stamp_=ros::Time();
        //p2.setOrigin(btVector3(1.165 - fridgeLink, -0.655, 1.151));
        p2.setOrigin(bottle.getOrigin() + btVector3(-.1,0,.03));
        p2.setRotation(btQuaternion(0.005, -0.053, -0.029, 0.998));
        goal.push_back(p2);
        arm.push_back(1);

        tf::Stamped<tf::Pose> p3;
        p3.frame_id_="map";
        p3.stamp_=ros::Time();
        //p3.setOrigin(btVector3(1.265000 - fridgeLink, -0.655000, 0.951000));
        p3.setOrigin(bottle.getOrigin() + btVector3(.02,0,.03));
        p3.setRotation(btQuaternion(0.005001, -0.053009, -0.029005, 0.998160));
        goal.push_back(p3);
        arm.push_back(1);

        RobotArm::findBaseMovement(result, arm, goal,true, true);
        //RobotArm::findBaseMovement(result, arm, goal,false);
    }

    Gripper::getInstance(1)->open();
    RobotArm::getInstance(1)->universal_move_toolframe_ik(0.8, -0.455, 1.251, 0.005, -0.053, -0.029, 0.998, "map");
    Gripper::getInstance(1)->updatePressureZero();
    RobotArm::getInstance(1)->universal_move_toolframe_ik(0.8, -0.655, 1.251, 0.005, -0.053, -0.029, 0.998, "map");
    //RobotArm::getInstance(1)->universal_move_toolframe_ik(1.065, -0.655, 1.151, 0.005, -0.053, -0.029, 0.998, "map");
    RobotArm::getInstance(1)->universal_move_toolframe_ik(bottle.getOrigin().x() - .1, bottle.getOrigin().y(), bottle.getOrigin().z() + .03, 0.005, -0.053, -0.029005, 0.998160, "map");
    //RobotArm::getInstance(1)->universal_move_toolframe_ik(1.265000, -0.655000, 0.951000, 0.005001, -0.053009, -0.029005, 0.998160, "map");
    //RobotArm::getInstance(1)->universal_move_toolframe_ik(1.2, -0.655000, 0.951000, 0.005001, -0.053009, -0.029005, 0.998160, "map");
    RobotArm::getInstance(1)->universal_move_toolframe_ik(bottle.getOrigin().x(), bottle.getOrigin().y(), bottle.getOrigin().z() + .03, 0.005, -0.053, -0.029005, 0.998160, "map");

    Gripper::getInstance(1)->closeCompliant();
    Gripper::getInstance(1)->close(0.04);
    //RobotArm::getInstance(1)->universal_move_toolframe_ik(1.065, -0.655, 1.151, 0.005, -0.053, -0.029, 0.998, "map");
    RobotArm::getInstance(1)->universal_move_toolframe_ik(bottle.getOrigin().x() - .1, bottle.getOrigin().y(), bottle.getOrigin().z() + .03, 0.005, -0.053, -0.029005, 0.998160, "map");
    RobotArm::getInstance(1)->universal_move_toolframe_ik(0.8, -0.655, 1.251, 0.005, -0.053, -0.029, 0.998, "map");
    RobotArm::getInstance(1)->universal_move_toolframe_ik(0.8, -0.455, 1.251, 0.005, -0.053, -0.029, 0.998, "map");

    return 0;
}


int DemoScripts::closeFridge(int handle)
{
    OperateHandleController::close(0,handle);
    //go to right tuck
    pr2_controllers_msgs::JointTrajectoryGoal goalAT = RobotArm::getInstance(0)->lookAtMarker(Poses::prepDishRT,Poses::prepDishRT);
    boost::thread t2T(&RobotArm::startTrajectory, RobotArm::getInstance(0), goalAT,true);
    RobotArm::getInstance(0)->tucked = true;
    t2T.join();
    return 0;
}


int DemoScripts::takeBottleFromFridge(int z)
{


    //approach fridge and grasp handle
    {

        RobotHead::getInstance()->lookAtThreaded("/map", 0.919, -0.553, 1.35);


        RobotArm::RobotArm *arm = RobotArm::getInstance(0);
        Torso *torso = Torso::getInstance();
        Gripper *gripper = Gripper::getInstance(0);

        boost::thread t2(&Torso::up, torso);

        OperateHandleController::plateTuckPose();

        RobotDriver::getInstance()->moveBaseP(0.305, -0.515, -0.348, 0.938, false);
        t2.join();


        RobotHead::getInstance()->lookAtThreaded("/map", 0.919, -0.553, 1.35);


        tf::Stamped<tf::Pose> handleHint;
        handleHint.setOrigin(btVector3( 0.919, -0.553, 1.35 ));
        handleHint.frame_id_ = "/map";
        tf::Stamped<tf::Pose> handlePos = Perception3d::getHandlePoseFromLaser(handleHint);

        //yes TWO Times
        //handlePos = OperateHandleController::getHandlePoseFromLaser(handleHint);


        handlePos = arm->getPoseIn("map",handlePos);

        ROS_INFO("Handle in Map");
        arm->printPose(handlePos);

        handlePos.setRotation(btQuaternion( 0.998732, 0.0416648, 0.0273237, -0.00716123));

        tf::Stamped<tf::Pose> handlePosApp = handlePos;
        tf::Stamped<tf::Pose> handlePosAppB = handlePos;
        handlePosApp.setOrigin(handlePosApp.getOrigin() + btVector3(-.05,0,1.35 - handlePos.getOrigin().z() ));
        handlePosAppB.setOrigin(handlePosAppB.getOrigin() + btVector3(.05,0,1.35 - handlePos.getOrigin().z() ));
        tf::Stamped<tf::Pose> handlePosAppM = handlePos;
        handlePosAppM.setOrigin(handlePosAppM.getOrigin() + btVector3(-.10,0,1.35 - handlePos.getOrigin().z() ));


        arm->time_to_target = 2;
        arm->move_toolframe_ik_pose(handlePosAppM);
        arm->time_to_target = 1;

        Approach *apr = new Approach();
        apr->init(0,handlePosApp, handlePosAppB, Approach::front);

        gripper->close();

        float distA = (apr->increment(0,0.5));
        if (distA == 0)
        {
            ROS_ERROR("DIDNT TOUCH IN THE FIRST 5 CM OF APPROACH");
            distA = (apr->increment(0.5,1));
        }
        //back up 5 centimeter
        apr->move_to((distA - .5));
        gripper->open();
        //go 2.75 cm forward from touch position
        apr->move_to((distA + .375));

        gripper->closeCompliant();

        gripper->close();

        arm->stabilize_grip();
    }

// open fridge, take bottle and close it againg

    {

        //boost::thread t2(&cart_err_monit, 0);

        int side = 0;
        RobotArm::RobotArm *arm = RobotArm::getInstance(side);

        float pts[][7] =
        {
            {0.478704, -1.0355, 1.18101, 0.767433, 0.639987, 0.022135, 0.0311955},
            {0.489086, -0.984206, 1.17956, 0.797904, 0.601535, 0.01726, 0.0347398},
            {0.494529, -0.937741, 1.1803, 0.830545, 0.555891, 0.0110758, 0.0325103},
            {0.504333, -0.909376, 1.18066, 0.849808, 0.526105, 0.00709967, 0.03147},
            {0.507886, -0.884252, 1.17954, 0.8814, 0.471274, 0.00699274, 0.0313926},
            {0.516993, -0.854729, 1.18006, 0.903026, 0.428457, 0.00859376, 0.0299171},
            {0.527833, -0.832331, 1.1803, 0.920176, 0.390256, 0.0125722, 0.0286066},
            {0.541463, -0.80644, 1.18, 0.931353, 0.362808, 0.0186723, 0.0245782},
            {0.571712, -0.760535, 1.17887, 0.936451, 0.349496, 0.024334, 0.017896},
            {0.608236, -0.715618, 1.17839, 0.944274, 0.327791, 0.0273483, 0.0123364},
            {0.647457, -0.676296, 1.17812, 0.954053, 0.298037, 0.0302956, 0.00623379},
            {0.690692, -0.638766, 1.17999, 0.964469, 0.262043, 0.0336022, 0.00195834},
            {0.734141, -0.609302, 1.18042, 0.974717, 0.220844, 0.0339708, -0.00102721},
            {0.781735, -0.583995, 1.17916, 0.983083, 0.180164, 0.0327274, -0.00426907},
            {0.828575, -0.564397, 1.17937, 0.990023, 0.137179, 0.0315954, -0.00617472},
            {0.870116, -0.550422, 1.17831, 0.995336, 0.0920069, 0.0283872, -0.00586025},
            {0.921693, -0.544899, 1.17853, 0.998734, 0.0415909, 0.0273629, -0.00714236},
            {0.971471, -0.549669, 1.17854, 0.998732, 0.0416648, 0.0273237, -0.00716123}
        };


        //for (float z= 1.3; z <= 1.4; z +=0.025)
        float z = 1.35;
        {
            int numf = 0;
            bool found = true;

            double stA[7];
            arm->getJointState(stA);

            RobotHead::getInstance()->lookAtThreaded("/map", 1.243111, -0.728864, 0.7);

            for (int k = 0; k <= 16; ++k)
            {
                tf::Stamped<tf::Pose> act;
                act.frame_id_ = "map";
                act.setOrigin(btVector3(pts[k][0],pts[k][1],z));
                act.setRotation(btQuaternion(pts[k][3],pts[k][4],pts[k][5],pts[k][6]));
                act = arm->tool2wrist(act);

                geometry_msgs::PoseStamped stamped_pose;
                stamped_pose.header.frame_id = "map";
                stamped_pose.header.stamp = ros::Time::now();
                stamped_pose.pose.position.x=act.getOrigin().x();
                stamped_pose.pose.position.y=act.getOrigin().y();
                stamped_pose.pose.position.z=z;
                stamped_pose.pose.orientation.x=pts[k][3];
                stamped_pose.pose.orientation.y=pts[k][4];
                stamped_pose.pose.orientation.z=pts[k][5];
                stamped_pose.pose.orientation.w=pts[k][6];

                double stAs[7];
                int ret = arm->run_ik(stamped_pose,stA,stAs,side == 0 ? "r_wrist_roll_link" : "l_wrist_roll_link");
                ROS_INFO("       z %f k %i %s", z, k, ret ? "found." : "not found.");
                numf += ret;
                if (ret==0)
                    found = false;
                else
                {
                    for (int i = 0; i < 7; ++i)
                        stA[i] = stAs[i];
                }
            }
            ROS_INFO("FOR HEIGHT z %f %s num %i", z, found ? "found." : "not found.", numf);
            if (found)
            {

                double stA[7];
                arm->getJointState(stA);
                ros::Rate rate(3.0);
                int idx[] = {16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,16,16};
                for (int o = 0; o <= 16+16; ++o)
                    //for (int k = 16; k >= 0; --k)
                {
                    int k = idx[o];
                    tf::Stamped<tf::Pose> act;
                    act.frame_id_ = "map";
                    act.setOrigin(btVector3(pts[k][0],pts[k][1],z));
                    act.setRotation(btQuaternion(pts[k][3],pts[k][4],pts[k][5],pts[k][6]));
                    act = arm->tool2wrist(act);

                    geometry_msgs::PoseStamped stamped_pose;
                    stamped_pose.header.frame_id = "map";
                    stamped_pose.header.stamp = ros::Time::now();
                    stamped_pose.pose.position.x=act.getOrigin().x();
                    stamped_pose.pose.position.y=act.getOrigin().y();
                    stamped_pose.pose.position.z=z;
                    stamped_pose.pose.orientation.x=pts[k][3];
                    stamped_pose.pose.orientation.y=pts[k][4];
                    stamped_pose.pose.orientation.z=pts[k][5];
                    stamped_pose.pose.orientation.w=pts[k][6];

                    double stAs[7];
                    int ret = arm->run_ik(stamped_pose,stA,stAs,side == 0 ? "r_wrist_roll_link" : "l_wrist_roll_link");

                    //ROS_INFO("       z %f k %i %s", z, k, ret ? "found." : "not found.");
                    ROS_INFO("k : %i",k);

                    rate.sleep();
                    if (ret)
                    {
                        float pose[7];
                        float sum = 0;
                        for (int k = 0; k < 7; ++k)
                        {
                            pose[k] = stAs[k];
                            sum += stAs[k] * stAs[k];
                        }
                        if (sum > 0.01)
                            arm->startTrajectory(arm->goalTraj(pose),false);
                    }
                    if (k == 0)
                    {

                        RobotHead::getInstance()->lookAtThreaded("/map", 1.181542, -0.763771, 0.967066);

                        tf::Stamped<tf::Pose> bottle = Perception3d::getBottlePose();

                        Gripper::getInstance(1)->close();

                        float ptA[] = {0.41491862845470812, 1.3468554401788568, 1.501748997727044, -2.0247783614692936, -16.507431415382143, -1.3292235155277217, 15.027356561279952};
                        float ptB[] = {0.040263624618489424, 0.96465557759293075, 0.27150676981727662, -1.6130504582945409, -14.582800985450046, -1.1869058378819473, 14.819427432123987};
                        RobotArm *arml = RobotArm::getInstance(1);
                        arml->startTrajectory(arml->goalTraj(ptA,1.5));
                        arml->startTrajectory(arml->goalTraj(ptB,1.5));
                        Gripper::getInstance(1)->open();
                        RobotArm::getInstance(1)->universal_move_toolframe_ik(bottle.getOrigin().x() - .1, bottle.getOrigin().y(), bottle.getOrigin().z(), 0.005, -0.053, -0.029005, 0.998160, "map");
                        RobotArm::getInstance(1)->universal_move_toolframe_ik(bottle.getOrigin().x(), bottle.getOrigin().y(), bottle.getOrigin().z(), 0.005, -0.053, -0.029005, 0.998160, "map");

                        //Gripper::getInstance(1)->closeCompliant();
                        Gripper::getInstance(1)->close(0.02);
                        RobotArm::getInstance(1)->universal_move_toolframe_ik(bottle.getOrigin().x() - .1, bottle.getOrigin().y(), bottle.getOrigin().z(), 0.005, -0.053, -0.029005, 0.998160, "map");
                        arml->startTrajectory(arml->goalTraj(ptB,1.5));
                        RobotArm::getInstance(1)->universal_move_toolframe_ik(0.208, 0.120, 0.785 , -0.020, -0.101, -0.662, 0.742, "base_link");
                    }
                }
            }
        }

        Gripper::getInstance(0)->open();

        float target[4];
        target[0] = -0.3;
        target[1] = -0.15;
        target[2] = 0;
        target[3] = 1;
        ROS_INFO("POSE IN BASE %f %f %f", target[0],target[1], target[2]);
        RobotDriver::getInstance()->driveInOdom(target, 1);

        pr2_controllers_msgs::JointTrajectoryGoal goalAT = RobotArm::getInstance(0)->lookAtMarker(Poses::prepDishRT,Poses::prepDishRT);
        boost::thread t2T(&RobotArm::startTrajectory, RobotArm::getInstance(0), goalAT,true);
        t2T.join();

        //RobotArm::getInstance(1)->universal_move_toolframe_ik(0.308, 0.120, 0.91 , -0.020, -0.101, -0.662, 0.742, "base_link");

        //[1.1811415860679706, 0.40573692064179873, 1.772267589813042, -2.0337541455750725, 0.15224039094452749, -0.61844366806403794, 5.0541238004106495]

    }

    return 0;
}


int DemoScripts::serveBottle(int z)
{
    RobotArm::getInstance(0)->tucked =true;
    //float b1[] = {-0.016, -0.440, 0.013, 1.000};

    //RobotDriver::getInstance()->moveBase(b1, false);

    //RobotArm::getInstance(0)->startTrajectory(RobotArm::getInstance(0)->lookAtMarker(Poses::untuckPoseB, Poses::untuckPoseB));

    RobotHead::getInstance()->lookAtThreaded("/map",  -1.7, 2.2, 1);

    float b3[] = {-0.496, 2.203, 0.963, 0.270};
    RobotDriver::getInstance()->moveBase(b3, false);


    float b2[] = {-1.115, 2.683, 0.977, 0.214};
    boost::thread t1 (&RobotDriver::moveBase,RobotDriver::getInstance(), b2, false);

    RobotHead::getInstance()->lookAtThreaded("/l_gripper_tool_frame", 0 , 0, 0);

    RobotArm::getInstance(1)->universal_move_toolframe_ik(.466, 0.486, 0.95, -0.020, -0.101, -0.662, 0.742, "base_link");
    RobotArm::getInstance(1)->universal_move_toolframe_ik(.466, 0.486, 1.15, -0.020, -0.101, -0.662, 0.742, "base_link");

    //RobotArm::getInstance(1)->universal_move_toolframe_ik(0.5, 0.608, 0.94, -0.020, -0.101, -0.662, 0.742, "base_link");

    t1.join();

    RobotArm::getInstance(1)->universal_move_toolframe_ik(0.5, 0.608, 0.94, -0.059, 0.008, 0.131, 0.990, "base_link");

    Gripper::getInstance(1)->open();

    RobotArm::getInstance(1)->universal_move_toolframe_ik(0.3, 0.608, 1.0, -0.059, 0.008, 0.131, 0.990, "base_link");
    return 0;
}

int DemoScripts::openDrawer(int z)
{

    RobotHead::getInstance()->lookAtThreaded("/map", 1.212, 1.240, 1.0);

    RobotDriver::getInstance()->moveBaseP(-0.020, 1.818,0.051, 0.999,false);

    tf::Stamped<tf::Pose> p0;
    p0.frame_id_="map";
    p0.stamp_=ros::Time();
    p0.setOrigin(btVector3(0.64, 1.321, 0.762 + 0.03));
    p0.setRotation(btQuaternion(-0.714, -0.010, 0.051, 0.698));
    p0 = RobotArm::getInstance(0)->getPoseIn("base_link",p0);

    //tf::Stamped<tf::Pose> p0 = RobotArm::getInstance(0)->getToolPose("base_link");

    int handle = OperateHandleController::operateHandle(0,p0);

    RobotDriver::getInstance()->moveBaseP(-0.220, 1.818,0.051, 0.999,false);

    return handle;
}


int DemoScripts::takePlate(int z)
{
    RobotHead::getInstance()->lookAtThreaded("/map", .44 ,1.1, .7);

    Torso *torso = Torso::getInstance();
    boost::thread t2u(&Torso::up, torso);
    float target[] = {-0.329, 1.017, 0.018, 1.000};
    ROS_INFO("POSE IN BASE %f %f %f", target[0],target[1], target[2]);

    OperateHandleController::plateTuckPose();

    RobotArm::getInstance(0)->tucked = true;
    RobotDriver::getInstance()->moveBase(target,false);

    OperateHandleController::plateAttackPose();

    t2u.join();

    //in drawer
    RobotHead::getInstance()->lookAtThreaded("/map", .44 ,1.1, .7, true);

    OperateHandleController::getPlate(0,0.79 - 0.035 + 0.03);

    RobotDriver::getInstance()->moveBaseP(-0.437, 1.053, 0.315, .949,false);

    float cl[] = {0.12, 1.053, 0.315, 0.949};

    RobotDriver::getInstance()->moveBase(cl,false);

    return 0;
}


int DemoScripts::servePlateToIsland(int z)
{

    RobotHead::getInstance()->lookAtThreaded("/map", -1.85, 2.1, 1);

    RobotArm::getInstance(0)->tucked = true;

    //float b4[] = {-1.066, 1.564, 0.970, 0.244};

    float b4[] = {-0.960, 2.161,  0.999, -0.037};

    //OperateHandleController::plateCarryPose();

    RobotDriver::getInstance()->moveBase(b4, false);

    OperateHandleController::spinnerL(0.25,0,-.11);

    OperateHandleController::openGrippers();

    OperateHandleController::plateCarryPose();

    return 0;
}


int DemoScripts::takeSilverware(int z)
{
    // get silverware
//        [0.157, 1.154, 0.051] - Rotation: in Quaternion [0.001, -0.001, 0.044, 0.999]
    pr2_controllers_msgs::JointTrajectoryGoal goalA = RobotArm::getInstance(0)->lookAtMarker(Poses::prepDishR1,Poses::prepDishR1);
    pr2_controllers_msgs::JointTrajectoryGoal goalB = RobotArm::getInstance(1)->lookAtMarker(Poses::prepDishL1,Poses::prepDishL1);
    boost::thread t2(&RobotArm::startTrajectory, RobotArm::getInstance(0), goalA,true);
    boost::thread t3(&RobotArm::startTrajectory, RobotArm::getInstance(1), goalB,true);

    RobotHead::getInstance()->lookAtThreaded("/map", 0.812, 1.240, 1.0);

    RobotDriver::getInstance()->moveBaseP(0.157, 1.154,  0.044, 0.999);
    t2.join();
    t3.join();

    float zAdj = -.015;
    /* R
     Translation: [0.940, 1.032, 0.861]
    - Rotation: in Quaternion [-0.026, 0.733, 0.025, 0.679]
    rosrun tf tf_echo map l_gripper_tool_frame
    At time 1286912134.641
    - Translation: [0.922, 1.354, 0.864]
    - Rotation: in Quaternion [0.003, 0.723, 0.006, 0.690]
                in RPY [2.856, 1.522, 2.860]
                */

    float xR =  .940;
    float xL =  .922;
    float yR =  1.032;
    float yL =  1.354;
    float adj = 0.025;

    OperateHandleController::openGrippers();

    RobotArm::getInstance(1)->universal_move_toolframe_ik(0.812, 1.240, 1.0, 0.009, 0.679, 0.002, 0.734, "map");
    RobotArm::getInstance(1)->universal_move_toolframe_ik(xL, yL, 1.0, 0.009, 0.679, 0.002, 0.734, "map");
    RobotArm::getInstance(1)->universal_move_toolframe_ik(xL, yL, 0.860 + zAdj, 0.009, 0.679, 0.002, 0.734, "map");
    Gripper::getInstance(1)->closeCompliant();
    Gripper::getInstance(1)->close();
    RobotArm::getInstance(1)->universal_move_toolframe_ik(xL, yL, 1.0, 0.009, 0.679, 0.002, 0.734, "map");

    if (Gripper::getInstance(1)->getAmountOpen() <  0.001)
    {
        Gripper::getInstance(1)->open();
        RobotArm::getInstance(1)->universal_move_toolframe_ik(0.812, 1.240, 1.0, 0.009, 0.679, 0.002, 0.734, "map");
        RobotArm::getInstance(1)->universal_move_toolframe_ik(xL, yL, 1.0, 0.009, 0.679, 0.002, 0.734, "map");
        RobotArm::getInstance(1)->universal_move_toolframe_ik(xL, yL, 0.860 + zAdj, 0.009, 0.679, 0.002, 0.734, "map");
        Gripper::getInstance(1)->closeCompliant();
        Gripper::getInstance(1)->close();
        RobotArm::getInstance(1)->universal_move_toolframe_ik(xL, yL, 1.0, 0.009, 0.679, 0.002, 0.734, "map");

        if (Gripper::getInstance(1)->getAmountOpen() <  0.001)
        {
            Gripper::getInstance(1)->open();
            RobotArm::getInstance(1)->universal_move_toolframe_ik(0.812, 1.240, 1.0, 0.009, 0.679, 0.002, 0.734, "map");
            RobotArm::getInstance(1)->universal_move_toolframe_ik(xL, yL - adj, 1.0, 0.009, 0.679, 0.002, 0.734, "map");
            RobotArm::getInstance(1)->universal_move_toolframe_ik(xL, yL - adj, 0.860 + zAdj, 0.009, 0.679, 0.002, 0.734, "map");
            Gripper::getInstance(1)->closeCompliant();
            Gripper::getInstance(1)->close();
            RobotArm::getInstance(1)->universal_move_toolframe_ik(0.912, 1.340, 1.0, 0.009, 0.679, 0.002, 0.734, "map");
            if (Gripper::getInstance(1)->getAmountOpen() <  0.001)
            {
                Gripper::getInstance(1)->open();
                RobotArm::getInstance(1)->universal_move_toolframe_ik(0.812, 1.240, 1.0, 0.009, 0.679, 0.002, 0.734, "map");
                RobotArm::getInstance(1)->universal_move_toolframe_ik(xL, yL + adj, 1.0, 0.009, 0.679, 0.002, 0.734, "map");
                RobotArm::getInstance(1)->universal_move_toolframe_ik(xL, yL + adj, 0.860 + zAdj, 0.009, 0.679, 0.002, 0.734, "map");
                Gripper::getInstance(1)->closeCompliant();
                Gripper::getInstance(1)->close();
                RobotArm::getInstance(1)->universal_move_toolframe_ik(0.912, 1.340, 1.0, 0.009, 0.679, 0.002, 0.734, "map");
            }
        }
    }

    RobotArm::getInstance(0)->universal_move_toolframe_ik(xR, yR, 1.0, 0.044, 0.691, -0.040, 0.720, "map");
    RobotArm::getInstance(0)->universal_move_toolframe_ik(xR, yR, 0.851 + zAdj, 0.044, 0.691, -0.040, 0.720, "map");
    Gripper::getInstance(0)->closeCompliant();
    Gripper::getInstance(0)->close();
    RobotArm::getInstance(0)->universal_move_toolframe_ik(0.911, 0.95, 1.0, 0.044, 0.691, -0.040, 0.720, "map");

    if (Gripper::getInstance(0)->getAmountOpen() <  0.001)
    {
        Gripper::getInstance(0)->open();
        RobotArm::getInstance(0)->universal_move_toolframe_ik(xR, yR, 1.0, 0.044, 0.691, -0.040, 0.720, "map");
        RobotArm::getInstance(0)->universal_move_toolframe_ik(xR, yR, 0.851 + zAdj, 0.044, 0.691, -0.040, 0.720, "map");
        Gripper::getInstance(0)->closeCompliant();
        Gripper::getInstance(0)->close();
        RobotArm::getInstance(0)->universal_move_toolframe_ik(0.911, 0.95, 1.0, 0.044, 0.691, -0.040, 0.720, "map");


        if (Gripper::getInstance(0)->getAmountOpen() <  0.001)
        {
            Gripper::getInstance(0)->open();
            RobotArm::getInstance(0)->universal_move_toolframe_ik(xR, yR - adj, 1.0, 0.044, 0.691, -0.040, 0.720, "map");
            RobotArm::getInstance(0)->universal_move_toolframe_ik(xR, yR - adj, 0.851 + zAdj, 0.044, 0.691, -0.040, 0.720, "map");
            Gripper::getInstance(0)->closeCompliant();
            Gripper::getInstance(0)->close();
            RobotArm::getInstance(0)->universal_move_toolframe_ik(0.911, 0.95, 1.0, 0.044, 0.691, -0.040, 0.720, "map");
            if (Gripper::getInstance(0)->getAmountOpen() <  0.001)
            {
                Gripper::getInstance(0)->open();
                RobotArm::getInstance(0)->universal_move_toolframe_ik(xR, yR + adj, 1.0, 0.044, 0.691, -0.040, 0.720, "map");
                RobotArm::getInstance(0)->universal_move_toolframe_ik(xR, yR + adj, 0.851 + zAdj, 0.044, 0.691, -0.040, 0.720, "map");
                Gripper::getInstance(0)->closeCompliant();
                Gripper::getInstance(0)->close();
                RobotArm::getInstance(0)->universal_move_toolframe_ik(0.911, 0.95, 1.0, 0.044, 0.691, -0.040, 0.720, "map");
            }
        }
    }

    OperateHandleController::plateCarryPose();

    return 0;
}


int DemoScripts::serveToTable(int z)
{

    //OperateHandleController::plateCarryPose();

    RobotArm::getInstance(0)->tucked = true;


    float p[] = {-0.461, -0.836, 0.878, -0.479};
    //float p[] = {-0.428, -0.786, 0.870, -0.493};
    RobotDriver::getInstance()->moveBase(p, false);
    OperateHandleController::spinnerL(0.26, -0.08, -.16);

    OperateHandleController::openGrippers();


    pr2_controllers_msgs::JointTrajectoryGoal goalA = RobotArm::getInstance(0)->lookAtMarker(Poses::prepDishR1,Poses::prepDishR1);
    pr2_controllers_msgs::JointTrajectoryGoal goalB = RobotArm::getInstance(1)->lookAtMarker(Poses::prepDishL1,Poses::prepDishL1);
    boost::thread t2(&RobotArm::startTrajectory, RobotArm::getInstance(0), goalA,true);
    boost::thread t3(&RobotArm::startTrajectory, RobotArm::getInstance(1), goalB,true);

    t2.join();
    t3.join();

    float p2[] = {-0.261, -0.636, 0.878, -0.479};
    RobotDriver::getInstance()->moveBase(p2, false);

    return 0;
}


int DemoScripts::takePlateFromIsland(int z)
{
    Torso *torso = Torso::getInstance();
    boost::thread t2u(&Torso::up, torso);


    OperateHandleController::plateTuckPose();
    //RobotArm::getInstance(1)->startTrajectory(RobotArm::getInstance(1)->lookAtMarker(Poses::prepDishL0,Poses::prepDishL1));

    //RobotArm::getInstance(0)->tucked = true;
    //RobotDriver::getInstance()->moveBaseP(-0.866, 1.564, 0.970, 0.244,false);


    //RobotDriver::getInstance()->moveBaseP(-1.066, 1.564, 0.970, 0.244,false);


    //RobotDriver::getInstance()->moveBaseP(-0.957, 2.178, 0.999, -0.049,false);
    RobotDriver::getInstance()->moveBaseP(-1.03, 2.2, 0.999, -0.049,false);

    OperateHandleController::plateAttackPose();

    t2u.join();

    RobotHead::getInstance()->lookAtThreaded("/map", -1.712, 2.088, 0.865);


    btVector3 pl(-1.679, 1.904, 0.865);

    pl = btVector3(-1.712, 2.088, 0.865);
    pl = btVector3(-1.686, 2.1, 0.865);

    //OperateHandleController::getPlate(0);
    OperateHandleController::pickPlate(pl,0.27);

    OperateHandleController::plateCarryPose();

    return 0;
}

int DemoScripts::putObjectIntoFridge(int z)
{


    //approach fridge and grasp handle
    {

        RobotArm::RobotArm *arm = RobotArm::getInstance(0);
        Torso *torso = Torso::getInstance();
        Gripper *gripper = Gripper::getInstance(0);

        boost::thread t2(&Torso::up, torso);

        OperateHandleController::plateTuckPose();

        RobotDriver::getInstance()->moveBaseP(0.305, -0.515, -0.348, 0.938, false);
        t2.join();


        RobotHead::getInstance()->lookAt("/map", 0.919, -0.553, 1.35);


        tf::Stamped<tf::Pose> handleHint;
        handleHint.setOrigin(btVector3( 0.919, -0.553, 1.35 ));
        handleHint.frame_id_ = "/map";
        tf::Stamped<tf::Pose> handlePos = Perception3d::getHandlePoseFromLaser(handleHint);
        handlePos = Perception3d::getHandlePoseFromLaser(handleHint);
        handlePos = Perception3d::getHandlePoseFromLaser(handleHint);

        //yes TWO Times
        //handlePos = OperateHandleController::getHandlePoseFromLaser(handleHint);


        handlePos = arm->getPoseIn("map",handlePos);

        ROS_INFO("Handle in Map");
        arm->printPose(handlePos);

        handlePos.setRotation(btQuaternion( 0.998732, 0.0416648, 0.0273237, -0.00716123));

        tf::Stamped<tf::Pose> handlePosApp = handlePos;
        tf::Stamped<tf::Pose> handlePosAppB = handlePos;
        handlePosApp.setOrigin(handlePosApp.getOrigin() + btVector3(-.05,0,1.35 - handlePos.getOrigin().z() ));
        handlePosAppB.setOrigin(handlePosAppB.getOrigin() + btVector3(.05,0,1.35 - handlePos.getOrigin().z() ));
        tf::Stamped<tf::Pose> handlePosAppM = handlePos;
        handlePosAppM.setOrigin(handlePosAppM.getOrigin() + btVector3(-.10,0,1.35 - handlePos.getOrigin().z() ));


        arm->time_to_target = 2;
        arm->move_toolframe_ik_pose(handlePosAppM);
        arm->time_to_target = 1;

        Approach *apr = new Approach();
        apr->init(0,handlePosApp, handlePosAppB, Approach::front);

        gripper->close();

        float distA = (apr->increment(0,0.5));
        if (distA == 0)
        {
            ROS_ERROR("DIDNT TOUCH IN THE FIRST 5 CM OF APPROACH");
            distA = (apr->increment(0.5,1));
        }
        //back up 5 centimeter
        apr->move_to((distA - .5));
        gripper->open();
        //go 2.75 cm forward from touch position
        apr->move_to((distA + .375));

        gripper->closeCompliant();

        gripper->close();

        arm->stabilize_grip();
    }

// open fridge, take bottle and close it againg

    {

        //boost::thread t2(&cart_err_monit, 0);

        int side = 0;
        RobotArm::RobotArm *arm = RobotArm::getInstance(side);

        float pts[][7] =
        {
            {0.478704, -1.0355, 1.18101, 0.767433, 0.639987, 0.022135, 0.0311955},
            {0.489086, -0.984206, 1.17956, 0.797904, 0.601535, 0.01726, 0.0347398},
            {0.494529, -0.937741, 1.1803, 0.830545, 0.555891, 0.0110758, 0.0325103},
            {0.504333, -0.909376, 1.18066, 0.849808, 0.526105, 0.00709967, 0.03147},
            {0.507886, -0.884252, 1.17954, 0.8814, 0.471274, 0.00699274, 0.0313926},
            {0.516993, -0.854729, 1.18006, 0.903026, 0.428457, 0.00859376, 0.0299171},
            {0.527833, -0.832331, 1.1803, 0.920176, 0.390256, 0.0125722, 0.0286066},
            {0.541463, -0.80644, 1.18, 0.931353, 0.362808, 0.0186723, 0.0245782},
            {0.571712, -0.760535, 1.17887, 0.936451, 0.349496, 0.024334, 0.017896},
            {0.608236, -0.715618, 1.17839, 0.944274, 0.327791, 0.0273483, 0.0123364},
            {0.647457, -0.676296, 1.17812, 0.954053, 0.298037, 0.0302956, 0.00623379},
            {0.690692, -0.638766, 1.17999, 0.964469, 0.262043, 0.0336022, 0.00195834},
            {0.734141, -0.609302, 1.18042, 0.974717, 0.220844, 0.0339708, -0.00102721},
            {0.781735, -0.583995, 1.17916, 0.983083, 0.180164, 0.0327274, -0.00426907},
            {0.828575, -0.564397, 1.17937, 0.990023, 0.137179, 0.0315954, -0.00617472},
            {0.870116, -0.550422, 1.17831, 0.995336, 0.0920069, 0.0283872, -0.00586025},
            {0.921693, -0.544899, 1.17853, 0.998734, 0.0415909, 0.0273629, -0.00714236},
            {0.971471, -0.549669, 1.17854, 0.998732, 0.0416648, 0.0273237, -0.00716123}
        };


        //for (float z= 1.3; z <= 1.4; z +=0.025)
        float z = 1.35;
        {
            int numf = 0;
            bool found = true;

            double stA[7];
            arm->getJointState(stA);

            RobotHead::getInstance()->lookAt("/map", 1.243111, -0.728864, 0.7);

            for (int k = 0; k <= 16; ++k)
            {
                tf::Stamped<tf::Pose> act;
                act.frame_id_ = "map";
                act.setOrigin(btVector3(pts[k][0],pts[k][1],z));
                act.setRotation(btQuaternion(pts[k][3],pts[k][4],pts[k][5],pts[k][6]));
                act = arm->tool2wrist(act);

                geometry_msgs::PoseStamped stamped_pose;
                stamped_pose.header.frame_id = "map";
                stamped_pose.header.stamp = ros::Time::now();
                stamped_pose.pose.position.x=act.getOrigin().x();
                stamped_pose.pose.position.y=act.getOrigin().y();
                stamped_pose.pose.position.z=z;
                stamped_pose.pose.orientation.x=pts[k][3];
                stamped_pose.pose.orientation.y=pts[k][4];
                stamped_pose.pose.orientation.z=pts[k][5];
                stamped_pose.pose.orientation.w=pts[k][6];

                double stAs[7];
                int ret = arm->run_ik(stamped_pose,stA,stAs,side == 0 ? "r_wrist_roll_link" : "l_wrist_roll_link");
                ROS_INFO("       z %f k %i %s", z, k, ret ? "found." : "not found.");
                numf += ret;
                if (ret==0)
                    found = false;
                else
                {
                    for (int i = 0; i < 7; ++i)
                        stA[i] = stAs[i];
                }
            }
            ROS_INFO("FOR HEIGHT z %f %s num %i", z, found ? "found." : "not found.", numf);
            if (found)
            {

                double stA[7];
                arm->getJointState(stA);
                ros::Rate rate(3.0);
                int idx[] = {16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,16,16};
                for (int o = 0; o <= 16+16; ++o)
                    //for (int k = 16; k >= 0; --k)
                {
                    int k = idx[o];
                    tf::Stamped<tf::Pose> act;
                    act.frame_id_ = "map";
                    act.setOrigin(btVector3(pts[k][0],pts[k][1],z));
                    act.setRotation(btQuaternion(pts[k][3],pts[k][4],pts[k][5],pts[k][6]));
                    act = arm->tool2wrist(act);

                    geometry_msgs::PoseStamped stamped_pose;
                    stamped_pose.header.frame_id = "map";
                    stamped_pose.header.stamp = ros::Time::now();
                    stamped_pose.pose.position.x=act.getOrigin().x();
                    stamped_pose.pose.position.y=act.getOrigin().y();
                    stamped_pose.pose.position.z=z;
                    stamped_pose.pose.orientation.x=pts[k][3];
                    stamped_pose.pose.orientation.y=pts[k][4];
                    stamped_pose.pose.orientation.z=pts[k][5];
                    stamped_pose.pose.orientation.w=pts[k][6];

                    double stAs[7];
                    int ret = arm->run_ik(stamped_pose,stA,stAs,side == 0 ? "r_wrist_roll_link" : "l_wrist_roll_link");

                    //ROS_INFO("       z %f k %i %s", z, k, ret ? "found." : "not found.");
                    ROS_INFO("k : %i",k);

                    rate.sleep();
                    if (ret)
                    {
                        float pose[7];
                        float sum = 0;
                        for (int k = 0; k < 7; ++k)
                        {
                            pose[k] = stAs[k];
                            sum += stAs[k] * stAs[k];
                        }
                        if (sum > 0.01)
                            arm->startTrajectory(arm->goalTraj(pose),false);
                    }

                    //this here shall actually get the bottle's pose and grasp it
                    if (k == 0)
                    {

                        RobotHead::getInstance()->lookAt("/map", 1.181542, -0.763771, 0.967066);

                        tf::Stamped<tf::Pose> bottle = Perception3d::getFridgePlaneCenterPose();
                        //bottle.pose.position.z = bottle.getOrigin().z() + 0.1;
                        Gripper::getInstance(1)->close();

                        //                        float ptA[] = {0.41491862845470812, 1.3468554401788568, 1.501748997727044, -2.0247783614692936, -16.507431415382143, -1.3292235155277217, 15.027356561279952};
                        //                        float ptB[] = {0.040263624618489424, 0.96465557759293075, 0.27150676981727662, -1.6130504582945409, -14.582800985450046, -1.1869058378819473, 14.819427432123987};
                        float ptA[] = {0.68376502811441964, 1.2012096482630152, 1.8365364116753793, -2.2751645879302225, -46.069969252840536, -1.5540038123036684, 33.476251846428482};
                        float ptB[] = {-0.076488653049139876, 0.79236238489918576, -0.066073603203320896, -1.5513110310125839, -46.176928516612122, -1.1200023343102063, 33.886323418367155};

                        RobotArm *arml = RobotArm::getInstance(1);
                        arml->startTrajectory(arml->goalTraj(ptA,1.5));
                        arml->startTrajectory(arml->goalTraj(ptB,1.5));
                        //Gripper::getInstance(1)->open();
                        double offset_above_plane = 0.1;
                        RobotArm::getInstance(1)->universal_move_toolframe_ik(bottle.getOrigin().x() - .1, bottle.getOrigin().y(), bottle.getOrigin().z() + offset_above_plane, 0.005, -0.053, -0.029005, 0.998160, "map");
                        RobotArm::getInstance(1)->universal_move_toolframe_ik(bottle.getOrigin().x(), bottle.getOrigin().y(), bottle.getOrigin().z() + offset_above_plane, 0.005, -0.053, -0.029005, 0.998160, "map");

                        //Gripper::getInstance(1)->closeCompliant();
                        //Gripper::getInstance(1)->close(0.04);
                        Gripper::getInstance(1)->open();
                        RobotArm::getInstance(1)->universal_move_toolframe_ik(bottle.getOrigin().x() - .1, bottle.getOrigin().y(), bottle.getOrigin().z() + offset_above_plane, 0.005, -0.053, -0.029005, 0.998160, "map");
                        arml->startTrajectory(arml->goalTraj(ptB,1.5));
                        arml->startTrajectory(arml->goalTraj(ptA,1.5));
                        Gripper::getInstance(1)->close();
                        RobotArm::getInstance(1)->universal_move_toolframe_ik(0.208, 0.120, 0.785 , -0.020, -0.101, -0.662, 0.742, "base_link");
                    }
                }
            }
        }
        Gripper::getInstance(0)->open();


        float target[4];
        target[0] = -0.3;
        target[1] = -0.15;
        target[2] = 0;
        target[3] = 1;
        ROS_INFO("POSE IN BASE %f %f %f", target[0],target[1], target[2]);
        RobotDriver::getInstance()->driveInOdom(target, 1);

        pr2_controllers_msgs::JointTrajectoryGoal goalAT = RobotArm::getInstance(0)->lookAtMarker(Poses::prepDishRT,Poses::prepDishRT);
        boost::thread t2T(&RobotArm::startTrajectory, RobotArm::getInstance(0), goalAT,true);
        t2T.join();

        //RobotArm::getInstance(1)->universal_move_toolframe_ik(0.308, 0.120, 0.91 , -0.020, -0.101, -0.662, 0.742, "base_link");

        //[1.1811415860679706, 0.40573692064179873, 1.772267589813042, -2.0337541455750725, 0.15224039094452749, -0.61844366806403794, 5.0541238004106495]

    }
    return 0;
}

// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX


int DemoScripts::sliceTheBread(int numslices_)
{
    float numslices = numslices_;

    Torso::getInstance()->up();

    //float numslices = atoi(argv[2]); // 0 = one slice for computer scientists! -1 = none
    float slicethickness = 0.02;

    boost::thread t0(&OperateHandleController::plateTuckPose);

    RobotHead::getInstance()->lookAtThreaded("/slicer",0,0,-0.3);

    tf::Stamped<tf::Pose> newBasePose;
    newBasePose.frame_id_ = "/map";
    newBasePose.setOrigin(btVector3(-2.979, 1.397, 0.052));
    newBasePose.setRotation(btQuaternion(-0.001, 0.000, 0.046, 0.999));

    RobotDriver::getInstance()->driveInMap(newBasePose);

    t0.join();
    boost::thread t0b(&OperateHandleController::plateAttackPose);

    newBasePose.setOrigin(btVector3(-2.679, 1.397, 0.052));

    RobotDriver::getInstance()->driveInMap(newBasePose);

    t0b.join();

    //base
    //Translation: [-2.679, 1.397, 0.052]
    //- Rotation: in Quaternion [-0.001, 0.000, 0.046, 0.999]
    // in RPY [-0.002, 0.000, 0.092]
    //r_gripper_tool_frame // right of
    //- Translation: [-2.106, 1.153, 0.901]
    //- Rotation: in Quaternion [0.521, -0.491, -0.510, -0.477]
    //  in RPY [3.030, 1.546, -1.619]
    // l_gripper_tool_frame = button
    //- Translation: [-2.032, 1.389, 1.121]
    //- Rotation: in Quaternion [-0.503, 0.487, 0.502, 0.507]
    ///    in RPY [-0.850, 1.543, 0.732]


    tf::Stamped<tf::Pose> leftEdge;
    leftEdge.setOrigin(btVector3(-2.032, 1.389, 1.121));
    leftEdge.frame_id_ = "map";

    tf::Stamped<tf::Pose> rightEdge;
    rightEdge.setOrigin(btVector3(-2.106, 1.153, 0.901));
    rightEdge.frame_id_ = "map";

    btVector3 rel = leftEdge.getOrigin() - rightEdge.getOrigin();

    float at2 = atan2(rel.y(), rel.x());

    float analog_synthesizer_tb = .303; // just to make it straight

    btQuaternion ori(btVector3(0,0,1), at2 + analog_synthesizer_tb);

    tf::Stamped<tf::Pose> midEdge;

    midEdge.frame_id_ = "map";

    rightEdge.setRotation(ori);
    leftEdge.setRotation(ori);
    midEdge.setRotation(ori);

    midEdge.setOrigin((rightEdge.getOrigin() + leftEdge.getOrigin()) * .5f);

    //Translation: [-0.229, 0.029, -0.109]
    //- Rotation: in Quaternion [-0.014, 0.692, -0.007, 0.722]
    //   in RPY [-0.613, 1.520, -0.603]

    //publish "midedge" as position of the slicer
    //rosrun tf static_transform_publisher -2.069000 1.271000 1.011000 0.000000 0.000000 0.706806 0.707407 map slicer 100

    tf::Stamped<tf::Pose> pur; //pick up the bread
    pur.setOrigin(btVector3(-0.2, 0.029, -0.109));
    pur.setRotation(btQuaternion(-0.014, 0.692, -0.007, 0.722));
    pur.frame_id_ = "slicer";

    //Translation: [-0.045, 0.191, -0.079]
    //- Rotation: in Quaternion [0.004, 0.720, 0.022, 0.693]
    //    in RPY [2.379, 1.518, 2.404]
    tf::Stamped<tf::Pose> pre; //preslicing pose
    pre.setOrigin(btVector3(-0.005, 0.191, -0.079));
    pre.setRotation(btQuaternion(0.004, 0.720, 0.022, 0.693));
    pre.frame_id_ = "slicer";


    //- Translation: [-0.061, -0.017, -0.074]
    //- Rotation: in Quaternion [-0.022, 0.738, 0.001, 0.675]
    //    in RPY [-2.835, 1.477, -2.802]
    tf::Stamped<tf::Pose> post; //after slicing
    post.setOrigin(btVector3(-0.005, -0.017, -0.079));
    post.setRotation(btQuaternion(-0.022, 0.738, 0.001, 0.675));
    post.frame_id_ = "slicer";


    //- Translation: [0.118, -0.037, 0.110]
    //- Rotation: in Quaternion [-0.011, 0.705, 0.000, 0.709]
    //    in RPY [-1.261, 1.555, -1.245]
    tf::Stamped<tf::Pose> butup;
    butup.setOrigin(btVector3(0.12, -0.03, 0.12));
    butup.setRotation(btQuaternion(-0.011, 0.705, 0.000, 0.709));
    butup.frame_id_ = "slicer";

    tf::Stamped<tf::Pose> prebutup;
    prebutup.setOrigin(btVector3(0.18, -0.03, 0.12));
    prebutup.setRotation(btQuaternion(-0.011, 0.705, 0.000, 0.709));
    prebutup.frame_id_ = "slicer";

    tf::Stamped<tf::Pose> butdown;
    //butdown.setOrigin(btVector3(0.118, -0.037, 0.08));
    //butdown.setOrigin(btVector3(0.118, -0.037, 0.07));
    butdown.setOrigin(btVector3(0.12, -0.03, 0.085));
    butdown.setRotation(btQuaternion(-0.011, 0.705, 0.000, 0.709));
    butdown.frame_id_ = "slicer";

    RobotArm *rarm = RobotArm::getInstance(0);
    Gripper *rgrip = Gripper::getInstance(0);
    RobotArm *larm = RobotArm::getInstance(1);
    Gripper *lgrip = Gripper::getInstance(1);

    ros::Rate onesec(1);

    boost::thread prebuttona(&RobotArm::move_toolframe_ik_pose, larm, prebutup);

    tf::Stamped<tf::Pose> nextPoseR = pur;
    nextPoseR.setOrigin(pur.getOrigin() + btVector3(-numslices * slicethickness,0,0.15));
    rarm->universal_move_toolframe_ik_pose(nextPoseR);
    rgrip->open();

    nextPoseR.setOrigin(pur.getOrigin() + btVector3(-numslices * slicethickness,0,0));
    rarm->universal_move_toolframe_ik_pose(nextPoseR);
    rgrip->close(0.04);

    //nextPoseR.setOrigin(pur.getOrigin() + btVector3(-numslices * slicethickness -0.05,0,0.02));
    //rarm->universal_move_toolframe_ik_pose(nextPoseR);
    prebuttona.join();

    boost::thread t1(&Gripper::close, Gripper::getInstance(1), 0.0);

    boost::thread buttona(&RobotArm::move_toolframe_ik_pose, larm, butup);

    nextPoseR.setOrigin(pur.getOrigin() + btVector3(-numslices * slicethickness -0.05,0,0.05));
    rarm->universal_move_toolframe_ik_pose(nextPoseR);

    buttona.join();

    //lgrip->close(); xxx t

    boost::thread button(&RobotArm::move_toolframe_ik_pose, larm, butdown);

    RobotHead::getInstance()->lookAtThreaded("/r_gripper_tool_frame",0,0,0,false);

    nextPoseR = pre;
    nextPoseR.setOrigin(pre.getOrigin() + btVector3(-numslices * slicethickness, 0,0.1));
    rarm->universal_move_toolframe_ik_pose(nextPoseR);

    for (float nums = numslices; nums >= -1.0; nums-=1.0)
    {

        nextPoseR = pre;
        nextPoseR.setOrigin(pre.getOrigin() + btVector3(-nums * slicethickness, 0,0));
        rarm->universal_move_toolframe_ik_pose(nextPoseR);

        button.join();

        onesec.sleep();
        onesec.sleep();

        nextPoseR = post;
        nextPoseR.setOrigin(post.getOrigin() + btVector3(-nums * slicethickness, 0,0));

        RobotArm::getInstance(0)->time_to_target = 2.7;

        rarm->universal_move_toolframe_ik_pose(nextPoseR);

        RobotArm::getInstance(0)->time_to_target = 1;


        if (nums > -1)
        {
            nextPoseR = pre;
            nextPoseR.setOrigin(pre.getOrigin()  + btVector3(-(nums + 1) * slicethickness, 0,0));
            rarm->universal_move_toolframe_ik_pose(nextPoseR);
        }
    }

    boost::thread t2(&RobotArm::universal_move_toolframe_ik_pose,larm,butup);

    nextPoseR.setOrigin(pur.getOrigin() + btVector3(0,0,0.15));
    rarm->universal_move_toolframe_ik_pose(nextPoseR);
    nextPoseR.setOrigin(pur.getOrigin() + btVector3(-0.02,0,0.02));
    rarm->universal_move_toolframe_ik_pose(nextPoseR);
    nextPoseR.setOrigin(pur.getOrigin() + btVector3(0.04,0,0.0));
    rarm->universal_move_toolframe_ik_pose(nextPoseR);
    rgrip->open();
    nextPoseR.setOrigin(pur.getOrigin() + btVector3(0,0,0.15));
    rarm->universal_move_toolframe_ik_pose(nextPoseR);

    //OperateHandleController::plateAttackPose();
    return 0;
}


int DemoScripts::takeBreadPlate(int zee)
{
    Pressure::getInstance(0);
    Pressure::getInstance(1);

    double zoffs = 0.03;

    RobotHead::getInstance()->lookAtThreaded("/map",-2.15,1.6,0.5,false);

    //OperateHandleController::plateTuckPose();

    tf::Stamped<tf::Pose> newBasePose;
    newBasePose.frame_id_ = "/map";
    newBasePose.setOrigin(btVector3(-2.679, 1.397, 0.052));
    newBasePose.setRotation(btQuaternion(-0.001, 0.000, 0.046, 0.999));

    RobotDriver::getInstance()->driveInMap(newBasePose);

    OperateHandleController::plateAttackPose();

    tf::Stamped<tf::Pose> start;
    start.frame_id_ = "/map";
    start.setOrigin(btVector3(-2.144, 1.69, 0.889 + zoffs));
    start.setRotation(btQuaternion(-0.270, 0.666, -0.247, 0.650));

    tf::Stamped<tf::Pose> end;
    end.frame_id_ = "/map";
    end.setOrigin(btVector3(-2.153, 1.573, 0.898 + zoffs));
    end.setRotation(btQuaternion(-0.270, 0.666, -0.247, 0.650));

    OperateHandleController::singleSidedPick(1,start,end);

    float offset = -.05;

    tf::Stamped<tf::Pose> larm = RobotArm::getInstance(1)->getToolPose("/map");
    btVector3 pos = larm.getOrigin();
    pos.setY(2.15);
    pos.setX(pos.getX() + offset -.1);
    pos.setZ(pos.getZ() - .025);
    larm.setOrigin(pos);

    //RobotHead::getInstance()->lookAtThreaded("/map",-2.3,1.9,.5,false);
    RobotHead::getInstance()->lookAtThreaded("/l_gripper_tool_frame",0,0,0,false);


    RobotArm::getInstance(1)->time_to_target = 3;
    RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(larm);
    RobotArm::getInstance(1)->time_to_target = 1;


    //Translation: [-2.278, 1.979, 0.868] - Rotation: in Quaternion [-0.671, 0.187, 0.279, 0.661]
    larm.setOrigin(btVector3(-2.278 + offset, 1.979, 0.87 + zoffs));
    larm.setRotation(btQuaternion(-0.671, 0.187, 0.279, 0.661));

    // [-2.226, 2.135, 0.891]
    //- Rotation: in Quaternion [-0.376, 0.550, -0.068, 0.742]

    larm.setOrigin(btVector3(-2.226 + offset, 2.135, 0.87 + zoffs));
    larm.setRotation(btQuaternion(-0.376, 0.550, -0.068, 0.742));
    RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(larm);


    //Translation: [-2.300, 2.051, 0.892]
    //- Rotation: in Quaternion [-0.533, 0.468, 0.116, 0.695]


    //RobotHead::getInstance()->lookAtThreaded("/map",-2.3,2.05,.5,false);

    larm.setOrigin(btVector3(-2.300 + offset, 2.051, 0.87 + zoffs));
    larm.setRotation(btQuaternion(-0.533, 0.468, 0.116, 0.695));
    RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(larm);


    //larm.setOrigin(btVector3(-2.375 + offset, 1.939, 0.873));
    larm.setOrigin(btVector3(-2.375 + offset + 0.05, 1.939, 0.873 + zoffs));
    larm.setRotation(btQuaternion(-0.691, 0.138, 0.192, 0.683));
    RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(larm);


    //RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(larm);

    Gripper::getInstance(1)->open();

    //larm.setOrigin(btVector3(-2.37 + offset, 2.051, 0.87));
    larm.setOrigin(btVector3(-2.375 + offset - .05, 1.939, 0.873 + zoffs));
    larm.setRotation(btQuaternion(-0.691, 0.138, 0.192, 0.683));
    RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(larm);

    //exit(0);

    RobotHead::getInstance()->lookAtThreaded("/map",-2.26, 1.92,.5,false);

    OperateHandleController::plateAttackPose();

    //drive to grasp plate pose
    //RobotDriver::getInstance()->moveBaseP(-2.844, 1.403,0.055, 0.998);
    RobotDriver::getInstance()->moveBaseP(-2.904, 1.95,  0.055, 0.998);

    //OperateHandleController::pickPlate(btVector3(-2.18+ offset, 1.95, 0.865),.26);
    //OperateHandleController::pickPlate(btVector3(-2.18 + offset - 0.025, 1.95, 0.865),.26);
    OperateHandleController::pickPlate(btVector3(-2.18 + offset - 0.025, 1.9, 0.865 + zoffs),.26);

    RobotHead::getInstance()->lookAtThreaded("/map",-3.260, -1.76,.5,false);

    tf::Stamped<tf::Pose> basePos;
    RobotDriver::getInstance()->getRobotPose(basePos);
    basePos.setOrigin(basePos.getOrigin() + btVector3(-.15,0,0));

    RobotDriver::getInstance()->driveInMap(basePos);

    RobotDriver::getInstance()->moveBaseP(-3.290, -1.027, -0.697, 0.717);

    OperateHandleController::spinnerL(0.25,0,-.25);

    OperateHandleController::openGrippers();

    OperateHandleController::plateAttackPose();

    return 0;
}


int DemoScripts::takeBowl(int zee)
{
    Pressure::getInstance(0);
    Pressure::getInstance(1);
    OperateHandleController::plateTuckPose();

    double zoffs = 0.03;

    RobotHead::getInstance()->lookAtThreaded("/map",-1.8,1.6,.5);


    RobotDriver::getInstance()->moveBaseP(-2.785, 2.022,0,1);

    boost::thread t0(&OperateHandleController::plateAttackPose);

    ros::Rate rt(10);
    int looper = 50;
    while (--looper > 0)
        rt.sleep();


    tf::Stamped<tf::Pose> bowlPose_ = OperateHandleController::getBowlPose();
    btVector3 bowlPose = bowlPose_.getOrigin();// = OperateHandleController::getBowlPose();

    //-0.008453 -0.075438 0.013157  -0.687616 -0.104624 0.714747 0.073313

    tf::Stamped<tf::Pose> pur; //pick up the bread
    pur.setOrigin(btVector3(-0.008453, -0.075438 ,0.013157  ));
    pur.setRotation(btQuaternion(-0.687616, -0.104624, 0.714747, 0.073313));
    pur.frame_id_ = "bowly";
    btTransform pur_ = bowlPose_ * pur;//tf::Pose btrightGrasp =  midEdge * rRel ;


    tf::Stamped<tf::Pose> pickup;
    pickup.frame_id_ = "map";
    pickup.setOrigin(pur_.getOrigin());
    pickup.setRotation(pur_.getRotation());

    tf::Stamped<tf::Pose> prepickup;
    prepickup.frame_id_ = "map";
    prepickup.setOrigin(pur_.getOrigin() + btVector3(0,0,0.05));
    prepickup.setRotation(pur_.getRotation());

    t0.join();

    boost::thread t1(&Gripper::open, Gripper::getInstance(0), 0.08);

    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(prepickup);

    t1.join();

    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(pickup);

    Gripper::getInstance(0)->close();


    /*
    if (bowlPose.getZ() < 0.05)
    {
        ROS_ERROR("SOMETHING WENT WRONG WITH BOWL DETECTION");
        bowlPose = btVector3(-1.87,1.85,.909);
    }

    bowlPose.setZ(.909 + zoffs);

    tf::Stamped<tf::Pose> start;
    start.frame_id_ = "/map";
    //start.setOrigin(btVector3(-2.215, 1.955, 0.909));
    start.setOrigin(bowlPose - btVector3(.12,0,0.01));
    start.setRotation(btQuaternion(-0.631, 0.304, 0.379, 0.605));

    tf::Stamped<tf::Pose> end;
    end.frame_id_ = "/map";
    //end.setOrigin(btVector3(-2.07, 1.955, 0.909));
    end.setOrigin(bowlPose - btVector3(0.05,0,0.01));
    end.setRotation(btQuaternion(-0.631, 0.304, 0.379, 0.605));

    //bool RobotArm::findBaseMovement(btVector3 &result, std::vector<int> arm, std::vector<tf::Stamped<tf::Pose> > goal, bool drive, bool reach)

    std::vector<int> arm;
    std::vector<tf::Stamped<tf::Pose> > goal;
    btVector3 result;

    arm.push_back(0);
    goal.push_back(end);
    arm.push_back(0);
    goal.push_back(start);

    RobotArm::findBaseMovement(result, arm, goal,true, false);

    // XXXXXXXXXXXXXXXXXXX


    OperateHandleController::singleSidedPick(0,start,end);
    //- Translation: [-2.130, 1.967, 0.883]     -  Rotation: in Quaternion [-0.505, 0.472, 0.540, 0.480]

    //[-2.215, 1.955, 0.909] Rotation: in Quaternion [-2.215, 1.955, 0.909]

    tf::Stamped<tf::Pose> pitch;
    pitch = RobotArm::getInstance(0)->getToolPose("/map");
    pitch.setRotation(btQuaternion(-0.615489,0.343613,0.456079,0.543226));

    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(pitch);

    Gripper::getInstance(0)->close();

    //btTransform bpt = bowlPose;
    //btTransform hpt = pitch;
    btTransform rel = bowlPose_.inverseTimes(pitch);//bpt.inverseTimes(pitch);

    ROS_INFO("RELATIVE POSE AFTER ADJUSTMENT : %f %f %f  %f %f %f %f", rel.getOrigin().x(), rel.getOrigin().y(), rel.getOrigin().z(),
             rel.getRotation().x(), rel.getRotation().y(), rel.getRotation().z(), rel.getRotation().w());

             */

    RobotHead::getInstance()->stopThread();

    tf::Stamped<tf::Pose> carry;
    carry.frame_id_ = "/base_link";
    carry.setOrigin(btVector3(0.347, -0.045, 0.986));
    carry.setRotation(btQuaternion(-0.610, -0.094, 0.737, 0.276));
    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(carry);

    Gripper::getInstance(0)->close();


    RobotHead::getInstance()->lookAtThreaded("/map",-3.085, -1.792,.5);

    RobotDriver::getInstance()->moveBaseP(-2.981, 2.031, 0.002, 1.000);

    OperateHandleController::plateTuckPoseLeft();

    RobotDriver::getInstance()->moveBaseP(-3.180, -1.117,  -0.725, 0.689);

    //RobotArm::getInstance(0)->universal_move_toolframe_ik(-3.085, -1.792, 0.768, -0.371, 0.651, 0.132, 0.649,"/map");
    RobotArm::getInstance(0)->universal_move_toolframe_ik(-3.643, -1.772, 0.815, -0.371, 0.651, 0.132, 0.649,"/map");

    Gripper::getInstance(0)->open();

    OperateHandleController::plateAttackPose();
    OperateHandleController::plateTuckPose();

    RobotHead::getInstance()->stopThread();

    return 0;

}


