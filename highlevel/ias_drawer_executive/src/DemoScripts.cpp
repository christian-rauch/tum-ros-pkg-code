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
    tf::Stamped<tf::Pose> handlePos = OperateHandleController::getHandlePoseFromLaser(handleHint);
    handlePos = OperateHandleController::getHandlePoseFromLaser(handleHint);
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
    boost::thread t3(&RobotArm::startTrajectory, RobotArm::getInstance(1), goalB);


    Torso *torso = Torso::getInstance();
    boost::thread t2(&Torso::up, torso);


    RobotArm::getInstance(0)->tucked = true;
    float p[] = { 0.255, -0.571, -0.025, 1.000};
    //float p[] = { 0.255, -0.571, -0.108, 0.994 };
    RobotDriver::getInstance()->moveBase(p);

    //t2.join();t3.join();

    RobotHead::getInstance()->lookAt("/map", 1.243111, -0.728864, 0.9);
    tf::Stamped<tf::Pose> bottle = OperateHandleController::getBottlePose();


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
    Gripper::getInstance(1)->close(0.5);
    //RobotArm::getInstance(1)->universal_move_toolframe_ik(1.065, -0.655, 1.151, 0.005, -0.053, -0.029, 0.998, "map");
    RobotArm::getInstance(1)->universal_move_toolframe_ik(bottle.getOrigin().x() - .1, bottle.getOrigin().y(), bottle.getOrigin().z() + .03, 0.005, -0.053, -0.029005, 0.998160, "map");
    RobotArm::getInstance(1)->universal_move_toolframe_ik(0.8, -0.655, 1.251, 0.005, -0.053, -0.029, 0.998, "map");
    RobotArm::getInstance(1)->universal_move_toolframe_ik(0.8, -0.455, 1.251, 0.005, -0.053, -0.029, 0.998, "map");

    return 0;
}


int DemoScripts::closeFridge(int handle)
{
    OperateHandleController::close(handle);
    //go to right tuck
    pr2_controllers_msgs::JointTrajectoryGoal goalAT = RobotArm::getInstance(0)->lookAtMarker(Poses::prepDishRT,Poses::prepDishRT);
    boost::thread t2T(&RobotArm::startTrajectory, RobotArm::getInstance(0), goalAT);
    RobotArm::getInstance(0)->tucked = true;
    t2T.join();
    return 0;
}

int DemoScripts::serveBottle(int z)
{
    RobotArm::getInstance(0)->tucked =true;
    float b1[] = {-0.016, -0.440, 0.013, 1.000};


    //RobotDriver::getInstance()->moveBase(b1, false);

    //RobotArm::getInstance(0)->startTrajectory(RobotArm::getInstance(0)->lookAtMarker(Poses::untuckPoseB, Poses::untuckPoseB));

    float b3[] = {-0.496, 2.203, 0.963, 0.270};
    RobotDriver::getInstance()->moveBase(b3, false);

    float b2[] = {-1.115, 2.683, 0.977, 0.214};
    RobotDriver::getInstance()->moveBase(b2, false);

    RobotArm::getInstance(1)->universal_move_toolframe_ik(0.5, 0.608, 0.91, -0.059, 0.008, 0.131, 0.990, "base_link");

    Gripper::getInstance(1)->open();

    RobotArm::getInstance(1)->universal_move_toolframe_ik(0.3, 0.608, 0.91, -0.059, 0.008, 0.131, 0.990, "base_link");
    return 0;
}

int DemoScripts::openDrawer(int z)
{
    RobotDriver::getInstance()->moveBaseP(-0.020, 1.818,0.051, 0.999,false);

    tf::Stamped<tf::Pose> p0;
    p0.frame_id_="map";
    p0.stamp_=ros::Time();
    p0.setOrigin(btVector3(0.64, 1.321, 0.762));
    p0.setRotation(btQuaternion(-0.714, -0.010, 0.051, 0.698));
    p0 = RobotArm::getInstance(0)->getPoseIn("base_link",p0);

    //tf::Stamped<tf::Pose> p0 = RobotArm::getInstance(0)->getToolPose("base_link");

    int handle = OperateHandleController::operateHandle(0,p0);

    RobotDriver::getInstance()->moveBaseP(-0.220, 1.818,0.051, 0.999,false);

    return handle;
}


int DemoScripts::takePlate(int z)
{

    RobotHead::getInstance()->lookAt("/map", .44 ,1.1, .7);

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
    RobotHead::getInstance()->lookAt("/map", .44 ,1.1, .7);

    OperateHandleController::getPlate(0,0.79 - 0.035);

    RobotDriver::getInstance()->moveBaseP(-0.437, 1.053, 0.315, .949,false);

    float cl[] = {0.12, 1.053, 0.315, 0.949};

    RobotDriver::getInstance()->moveBase(cl,false);

    return 0;
}


int DemoScripts::servePlateToIsland(int z)
{

    RobotArm::getInstance(0)->tucked = true;

    //float b4[] = {-1.066, 1.564, 0.970, 0.244};

    float b4[] = {-0.960, 2.161,  0.999, -0.037};

    OperateHandleController::plateCarryPose();

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
    boost::thread t2(&RobotArm::startTrajectory, RobotArm::getInstance(0), goalA);
    boost::thread t3(&RobotArm::startTrajectory, RobotArm::getInstance(1), goalB);


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

    OperateHandleController::plateCarryPose();
    //float p[] = {-0.406, -0.728, 0.869, -0.495};

    //[-0.428, -0.786, 0.051]- Rotation: in Quaternion [-0.001, -0.002, 0.870, -0.493]
    RobotArm::getInstance(0)->tucked = true;


    float p[] = {-0.461, -0.836, 0.878, -0.479};
    //float p[] = {-0.428, -0.786, 0.870, -0.493};
    RobotDriver::getInstance()->moveBase(p, false);

//      - Translation: [-0.461, -0.836, 0.051]
//- Rotation: in Quaternion [-0.000, -0.001, 0.878, -0.479]
    //in RPY [-0.002, 0.002, -2.143]

    OperateHandleController::spinnerL(0.26, -0.08, -.16);

    OperateHandleController::openGrippers();


    pr2_controllers_msgs::JointTrajectoryGoal goalA = RobotArm::getInstance(0)->lookAtMarker(Poses::prepDishR1,Poses::prepDishR1);
    pr2_controllers_msgs::JointTrajectoryGoal goalB = RobotArm::getInstance(1)->lookAtMarker(Poses::prepDishL1,Poses::prepDishL1);
    boost::thread t2(&RobotArm::startTrajectory, RobotArm::getInstance(0), goalA);
    boost::thread t3(&RobotArm::startTrajectory, RobotArm::getInstance(1), goalB);

    t2.join();
    t3.join();

    //no show off pose
    float p2[] = {-0.261, -0.636, 0.878, -0.479};
    RobotDriver::getInstance()->moveBase(p2, false);

    //show off pose
    //float p2[] = {-0.231, -0.408,-0.687, 0.727};
    //RobotDriver::getInstance()->moveBase(p2, false);

    return 0;
}


int DemoScripts::takePlateFromIsland(int z)
{
    Torso *torso = Torso::getInstance();
    boost::thread t2u(&Torso::up, torso);


    OperateHandleController::plateTuckPose();
    //RobotArm::getInstance(1)->startTrajectory(RobotArm::getInstance(1)->lookAtMarker(Poses::prepDishL0,Poses::prepDishL1));

    RobotArm::getInstance(0)->tucked = true;
    RobotDriver::getInstance()->moveBaseP(-0.866, 1.564, 0.970, 0.244,false);

    OperateHandleController::plateAttackPose();

    //RobotDriver::getInstance()->moveBaseP(-1.066, 1.564, 0.970, 0.244,false);

    RobotDriver::getInstance()->moveBaseP(-0.957, 2.178, 0.999, -0.049,false);

    t2u.join();


    //in drawer
    //RobotHead::getInstance()->lookAt("/map", .4 ,1.13, .7);
    //RobotHead::getInstance()->lookAt("/map", -1.61612 ,1.8 ,0.7);
    RobotHead::getInstance()->lookAt("/map", -1.712, 2.088, 0.865);
    //0.437438 1.111197 0.758061

    // at sink
    //RobotHead::getInstance()->lookAt("/map",0.913379, 0.824588, 0.7);

    // 20 up 7 down from drawer to table

    //bin/ias_drawer_executi2 0 0.65, -0.4, 1.15 0.651, 0.295, -0.621, -0.322 ; bin/ias_drawer_executive -2 1 0.65, 0.4, 1.15 -0.295, 0.621, -0.322, 0.651
    //RobotArm::getInstance(0)->startTrajectory(RobotArm::getInstance(0)->lookAtMarker(Poses::prepDishR0,Poses::prepDishR1));
    //boost::thread t2(&RobotArm::startTrajectory, RobotArm::getInstance(0), RobotArm::getInstance(0)->lookAtMarker(Poses::prepDishR0,Poses::prepDishR1));
    //boost::thread t3(&RobotArm::startTrajectory, RobotArm::getInstance(1), RobotArm::getInstance(1)->lookAtMarker(Poses::prepDishL0,Poses::prepDishL1));

    //RobotArm::getInstance(0)->universal_move_toolframe_ik(0.65, -0.4, 1.15, 0.651, 0.295, -0.621, -0.322);
    //RobotArm::getInstance(1)->universal_move_toolframe_ik(0.65, 0.4, 1.15, -0.295, 0.621, -0.322, 0.651);
    //OperateHandleController::getPlate(atoi(argv[2]));

    btVector3 pl(-1.679, 1.904, 0.865);

    pl = btVector3(-1.712, 2.088, 0.865);

    //OperateHandleController::getPlate(0);
    OperateHandleController::pickPlate(pl);

    OperateHandleController::plateCarryPose();

    return 0;
}

