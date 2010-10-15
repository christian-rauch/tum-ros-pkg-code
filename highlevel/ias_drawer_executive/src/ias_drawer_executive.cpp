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

#include <ias_drawer_executive/RobotDriver.h>
#include <ias_drawer_executive/Gripper.h>
#include <ias_drawer_executive/Pressure.h>
#include <ias_drawer_executive/Poses.h>
#include <ias_drawer_executive/RobotArm.h>
#include <ias_drawer_executive/AverageTF.h>
#include <ias_drawer_executive/Torso.h>
#include <ias_drawer_executive/Head.h>
#include <ias_drawer_executive/OperateHandleController.h>

#include <boost/thread.hpp>

#include <actionlib/client/simple_client_goal_state.h>
#include <visualization_msgs/Marker.h>

#include <ias_drawer_executive/DemoScripts.h>

tf::Stamped<tf::Pose> toPose(float x, float y, float z, float ox, float oy, float oz, float ow, const char fixed_frame[])
{
    tf::Stamped<tf::Pose> p0;
    p0.frame_id_=fixed_frame;
    p0.stamp_=ros::Time();
    p0.setOrigin(btVector3(x,y,z));
    p0.setRotation(btQuaternion(ox,oy,oz,ow));
    return p0;
}

tf::Stamped<tf::Pose> toPose(const char text[], const char fixed_frame[])
{
    float x,y,z, ox,oy,oz,ow;
    sscanf(text,"%f %f %f %f %f %f %f", &x, &y, &z, &ox, &oy, &oz, &ow);
    return toPose(x,y,z, ox,oy,oz,ow,fixed_frame);
}

void printPose(const char title[], tf::Stamped<tf::Pose> pose)
{
    ROS_INFO("%s %f %f %f %f %f %f %f %s", title, pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z()
             , pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z(), pose.getRotation().w(), pose.frame_id_.c_str());
}

void printPose(const char title[], btTransform pose)
{
    ROS_INFO("%s %f %f %f %f %f %f %f", title, pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z()
             , pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z(), pose.getRotation().w());
}

int main(int argc, char** argv)
{
    // Init the ROS node
    ros::init(argc, argv, "ias_drawer_executive");

    printf("ias_drawer_executive %i\n", argc);
    printf("usage: ias_drawer_executive <drawer_position> <drawer_height> [<drawer_position> <drawer_height> .. <drawer_position> <drawer_height>] \n");
    printf("drawer_position:-1 .. dont move base \n");
    printf("                 0 .. island left \n");
    printf("                 1 .. island middle \n");
    printf("                 2 .. island right \n");
    printf("                 3 .. under oven \n");
    printf("                 4 .. left of dishwasher \n");
    printf("                 5 .. dishwasher \n");
    printf("                 6 .. right of dishwasher \n");
    printf("                 7 .. right of oven \n");
    printf("                 8 .. fridge \n");
    printf("drawer_height:-1 .. dont grasp/open drawer\n");
    printf("               0 .. low (~34cm) \n");
    printf("               1 .. middle (~64cm) \n");
    printf("               2 .. high (~78cm) \n");
    printf("               3 .. higher (~97cm) \n\n");

    ros::NodeHandle node_handle;


    /*if ((argc > 1) && (atoi(argv[1]) > -2) )
        for (int i = 0; i < (argc - 1) / 2 ; ++i)
        {
            if (argc >= (i * 2 + 3))
            {
                if (atoi(argv[i* 2 + 1]) >= 0)
                {
                    ROS_INFO("TARGET POSE IN MAP %f %f %f %f",Poses::poses[atoi(argv[i* 2 + 1])][0],Poses::poses[atoi(argv[i* 2 + 1])][1],Poses::poses[atoi(argv[i* 2 + 1])][2],Poses::poses[atoi(argv[i* 2 + 1])][3]);
                    RobotDriver::getInstance()->moveBase(Poses::poses[atoi(argv[i* 2 + 1])]);
                }
                if (atoi(argv[i* 2 + 2]) >= 0)
                    OperateHandleController::operateHandle(atoi(argv[i* 2 + 2]));
            }
        }*/



    if ((argc > 1) && (atoi(argv[1]) > -2) )
        for (int i = 0; i < (argc - 1) / 2 ; ++i)
        {
            if (argc >= (i * 2 + 3))
            {
                if (atoi(argv[i* 2 + 1]) >= 0)
                {
                    Torso *torso = Torso::getInstance();
                    boost::thread *t;
                    if (atoi(argv[i* 2 + 1])==8)
                        t = &boost::thread(&Torso::up, torso);
                    else
                        t = &boost::thread(&Torso::down, torso);
                    ROS_INFO("TARGET POSE IN MAP %f %f %f %f",Poses::poses[atoi(argv[i* 2 + 1])][0],Poses::poses[atoi(argv[i* 2 + 1])][1],Poses::poses[atoi(argv[i* 2 + 1])][2],Poses::poses[atoi(argv[i* 2 + 1])][3]);
                    RobotDriver::getInstance()->moveBase(Poses::poses[atoi(argv[i* 2 + 1])]);
                    t->join();
                }
                if (atoi(argv[i* 2 + 2]) >= 0)
                {
                    tf::Stamped<tf::Pose> aM  = OperateHandleController::getHandlePoseFromMarker(0,atoi(argv[i* 2 + 2]));
                    OperateHandleController::operateHandle(0,aM);
                }
            }
        }

    //! drive in map x y oz ow
    if (atoi(argv[1]) == -4)
    {

        RobotArm::getInstance(0)->tucked = true;
        for (int i = 0; i < (argc - 1 ) / 4; ++i)
        {
            float p[4];
            p[0] = atof(argv[2 + i * 4]);
            p[1] = atof(argv[3 + i * 4]);
            p[2] = atof(argv[4 + i * 4]);
            p[3] = atof(argv[5 + i * 4]);
            ROS_INFO("going to %f %f %f %f", p[0], p[1], p[2], p[3]);
            RobotDriver::getInstance()->moveBase(p);
        }
    }

    if (atoi(argv[1]) == -45) {

    //while (ros::ok()) {
    //   rate.sleep();
    //   ros::spinOnce();
    //   float r[2],l[2];
    //   p->getCenter(r,l);
    //   //ROS_INFO("CENTERS %f %f , %f %f", r[0], r[1], l[0], l[1]);
    //   float d[2];
    //   d[0] = r[0] - l[0];
    //   d[1] = r[1] - l[1];
    //   float k = fabs(d[1]);
    //   if (k > 1)
    //     k = 1;
    //   ROS_INFO("DIFF %f %f            argv %f", d[0], d[1], atof(argv[1]));
    //   if (d[1] > limit) {
    //      ROS_INFO("pos");
    //      arm->rotate_toolframe_ik(increment * k,0,0);
    //   }
    //   if (d[1] < -limit) {
    //      ROS_INFO("neg");
    //      arm->rotate_toolframe_ik(-increment * k,0,0);
    //   }
    //
         RobotArm::getInstance(atoi(argv[2]))->rotate_toolframe_ik(atof(argv[3]),atof(argv[4]),atof(argv[5]));
    }

    if (atoi(argv[1]) == -5)
    {
        //RobotArm arm;
        Gripper *gripper = Gripper::getInstance(0);
        gripper->open();
        Pressure::getInstance()->reset();
        gripper->close();
        RobotArm *arm = RobotArm::getInstance();
        arm->stabilize_grip();
    }

    //! put gripper to some pose in map
    if (atoi(argv[1]) == -3)
    {
        //bool move_ik
        double x,y,z,ox,oy,oz,ow;
        x = atof(argv[3]);
        y = atof(argv[4]);
        z = atof(argv[5]);
        ox = atof(argv[6]);
        oy = atof(argv[7]);
        oz = atof(argv[8]);
        ow = atof(argv[9]);
        printf("moving tool to ik goal pos in map %f %f %f or %f %f %f %f \n", x,y,z,ox,oy,oz,ow);
        RobotArm::getInstance(atoi(argv[2]))->universal_move_toolframe_ik(x,y,z,ox,oy,oz,ow,"map");
    }

    //! put gripper to some pose in base
    if (atoi(argv[1]) == -2)
    {
        //bool move_ik
        double x,y,z,ox,oy,oz,ow;
        x = atof(argv[3]);
        y = atof(argv[4]);
        z = atof(argv[5]);
        ox = atof(argv[6]);
        oy = atof(argv[7]);
        oz = atof(argv[8]);
        ow = atof(argv[9]);
        printf("moving tool %s to ik goal pos %f %f %f or %f %f %f %f \n", (atoi(argv[2])==0) ? "right" : "left" , x,y,z,ox,oy,oz,ow);
        RobotArm::getInstance(atoi(argv[2]))->universal_move_toolframe_ik(x,y,z,ox,oy,oz,ow);
        // arm->move_toolframe_ik(x,y,z,ox,oy,oz,ow);

        //btQuaternion myq(ox,oy,oz,ow);

        //btQuaternion qx = myq *     btQuaternion(M_PI / 2,0,0);;
        //btQuaternion qy = myq *     btQuaternion(0,M_PI / 2,0);;
        //btQuaternion qz = myq *     btQuaternion(0,0,M_PI / 2);;

        //ROS_INFO("ROT X %f %f %f %f", qx.x(),qx.y(),qx.z(),qx.w());
        //ROS_INFO("ROT Y %f %f %f %f", qy.x(),qy.y(),qy.z(),qy.w());
        //ROS_INFO("ROT Z %f %f %f %f", qz.x(),qz.y(),qz.z(),qz.w());

        //arm->move_ik(x,y,z,ox,oy,oz,ow);
    }

    if (atoi(argv[1]) == -6)
    {
        while (ros::ok())
        {
            tf::StampedTransform aM = AverageTF::getMarkerTransform("/4x4_1",20);
        }
    }

    if (atoi(argv[1]) == -7)
    {
        RobotArm::getInstance(1)->startTrajectory(RobotArm::getInstance(1)->lookAtMarker(Poses::lf0,Poses::lf1));
        RobotArm::getInstance(1)->startTrajectory(RobotArm::getInstance(1)->lookAtMarker(Poses::lf2,Poses::lf3));
    }


    if (atoi(argv[1]) == -8)
    {
        printf("ias_drawer_executive -8 arm(0=r,1=l) r_x r_y r_z frame_id");
        printf("rotate around frame downstream from wrist");
        RobotArm::getInstance(atoi(argv[2]))->rotate_toolframe_ik(atof(argv[3]),atof(argv[4]),atof(argv[5]), argv[6]);
    }

    //! get plate from open drawer
    if (atoi(argv[1]) == -9)
        DemoScripts::takePlate();

    //! drive relative to base
    if (atoi(argv[1]) == -10)
    {
        float target[4];
        target[0] = atof(argv[2]);
        target[1] = atof(argv[3]);
        target[2] = 0;
        target[3] = 1;
        ROS_INFO("POSE IN BASE %f %f %f", target[0],target[1], target[2]);
        RobotDriver::getInstance()->driveInOdom(target, 1);
    }

    //! print plate pose
    if (atoi(argv[1]) == -11)
    {
        btVector3 vec = OperateHandleController::getPlatePose();
        ROS_INFO("POSE IN MAP %f %f %f", vec.x(), vec.y(), vec.z());
    }

    //! get bottle out of open fridge
    if (atoi(argv[1]) == -12)
        DemoScripts::takeBottle();
    //! move both grippers uniformly in direction x y z in base
    if (atoi(argv[1]) == -13)
    {
        OperateHandleController::spinnerL(atof(argv[2]), atof(argv[3]),atof(argv[4]));
    }

    if (atoi(argv[1]) == -14)
    {

        RobotArm::getInstance(1)->universal_move_toolframe_ik(1.25, -0.7000, 1, 0.005001, -0.053009, -0.029005, 0.998160,"map");
        RobotArm::getInstance(1)->universal_move_toolframe_ik(1.165, -0.655, 1.151, 0.005, -0.053, -0.029, 0.998, "map");
        RobotArm::getInstance(1)->universal_move_toolframe_ik(0.9, -0.655, 1.251, 0.005, -0.053, -0.029, 0.998, "map");
        RobotArm::getInstance(1)->universal_move_toolframe_ik(0.9, -0.255, 1.251, 0.005, -0.053, -0.029, 0.998, "map");
    }

    if (atoi(argv[1]) == -15)   //check for maximum base movement
    {
        RobotDriver *driver = RobotDriver::getInstance();
        float relativePose[2] = {0,0};
        for (relativePose[0] = 0; relativePose[0] < 1; relativePose[0] = relativePose[0] + 0.025)
        {
            ROS_INFO("CAN DRIVE %f %i", relativePose[0], driver->checkCollision(relativePose));
        }
    }

    if (atoi(argv[1]) == -16)
    {
        std::vector<int> arm;
        std::vector<tf::Stamped<tf::Pose> > goal;
        btVector3 result;

        tf::Stamped<tf::Pose> p0;
        p0.frame_id_="map";
        p0.stamp_=ros::Time();
        p0.setOrigin(btVector3(0.9, -0.255, 1.251));
        p0.setRotation(btQuaternion(0.005, -0.053, -0.029, 0.998));
        goal.push_back(p0);
        arm.push_back(1);

        RobotArm::findBaseMovement(result, arm, goal,true, false);

        RobotArm::getInstance(1)->universal_move_toolframe_ik(0.9, -0.255, 1.251, 0.005, -0.053, -0.029, 0.998, "map");
    }

    if (atoi(argv[1]) == -17)
    {
        //    tf::Stamped<tf::Pose> toPose(float x, float y, float z, float ox, float oy, float oz, float ow, const char fixed_frame[])
        char posestrings[][100] = {{"0.542329 -0.981344 1.23612 0.0330708  0.00176235 0.595033  0.803019"},
            {"0.556517 -0.920874 1.23603 0.0298695 -0.00432607 0.545786  0.837381"},
            {"0.572956 -0.861407 1.23635 0.0268959 -0.00695657 0.494586  0.868685"},
            {"0.581633 -0.828156 1.23589 0.0252092 -0.00877769 0.475406  0.879362"},
            {"0.604209 -0.781168 1.23568 0.0179254  0.00291321 0.436192  0.89967"},
            {"0.622422 -0.748086 1.23539 0.0152842  0.00913191 0.340254  0.940165"},
            {"0.639726 -0.721124 1.23581 0.0164386  0.0162648  0.278332  0.960206"},
            {"0.661803 -0.694327 1.23576 0.0219128  0.0176437  0.246738  0.968674"},
            {"0.698809 -0.657863 1.23729 0.0284113  0.0207615  0.238006  0.970626"},
            {"0.740968 -0.625336 1.23853 0.0375375  0.0235445  0.227272  0.972823"},
            {"0.784319 -0.598149 1.23992 0.0450486  0.0276238  0.212765  0.975673"},
            {"0.82972  -0.572236 1.24111 0.0451087  0.0322855  0.169386  0.983987"},
            {"0.875225 -0.553171 1.24195 0.0358857  0.032936   0.122281  0.9913"},
            {"0.956881 -0.532016 1.24365 0.0254018  0.0329301  0.0418438 0.998258"},
            {"0.972319 -0.530781 1.2467  0.0022027  0.0133422  0.0679044 0.9976"},
            {"1.02179  -0.52356  1.24665 0.0022027  0.0133422  0.0679044 0.9976"}
        };
        tf::Stamped<tf::Pose> oldposes[20];
        for (int k = 0; k < 16; ++k)
        {
            oldposes[k] = toPose(posestrings[k],"map");
            printPose("traj : ", oldposes[k]);
        }
        for (int k = 1; k < 14; ++k)
        {
            btTransform diffPose = oldposes[k].inverse() * oldposes[k-1];
            //diffPose.setRotation(diffPose.getRotation().normalize());
            //printPose("difference", diffPose);
            float distance = diffPose.getOrigin().length();
            btQuaternion adjusted = diffPose.getRotation()  * (1 / distance);
            adjusted.normalize();
            ROS_INFO("DIST : %f QUAT ADJ : %f %f %f %f", distance, adjusted.x(), adjusted.y(), adjusted.z(), adjusted.w());
        }
    }


    if (atoi(argv[1]) == -18)
    {
        Gripper::getInstance(atoi(argv[2]))->open();
    }
    if (atoi(argv[1]) == -19)
    {
        Gripper::getInstance(atoi(argv[2]))->close();
    }
    if (atoi(argv[1]) == -20)
    {
        Torso::getInstance()->pos(atof(argv[2]));
    }

    if (atoi(argv[1]) == -21)
    {
        RobotHead::getInstance()->lookAt("/map",Poses::inDrawer0[0],Poses::inDrawer0[1],Poses::inDrawer0[2]);
    }

    //! look at a pos in map
    if (atoi(argv[1]) == -22)
    {
        RobotHead::getInstance()->lookAt("/map",atof(argv[2]),atof(argv[3]),atof(argv[4]));
    }

    // move tool without driving
    if (atoi(argv[1]) == -24)
    {
        //bool move_ik
        double x,y,z,ox,oy,oz,ow;
        x = atof(argv[3]);
        y = atof(argv[4]);
        z = atof(argv[5]);
        ox = atof(argv[6]);
        oy = atof(argv[7]);
        oz = atof(argv[8]);
        ow = atof(argv[9]);
        printf("moving tool to ik goal pos in map %f %f %f or %f %f %f %f \n", x,y,z,ox,oy,oz,ow);
        //RobotArm::getInstance(atoi(argv[2]))->universal_move_toolframe_ik(x,y,z,ox,oy,oz,ow,"map");

        tf::Stamped<tf::Pose> p0;
        p0.frame_id_="map";
        p0.stamp_=ros::Time();
        p0.setOrigin(btVector3(x,y,z));
        p0.setRotation(btQuaternion(ox,oy,oz,ow));
        RobotArm::getInstance(atoi(argv[2]))->move_toolframe_ik_pose(p0);
    }

    // test marker stuff
    if (atoi(argv[1]) == -25)
    {
        ros::NodeHandle node_handle;
        ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "/robot_arm_marker", 0 );

        visualization_msgs::Marker marker;
        marker.header.frame_id = "/base_link";
        marker.header.stamp = ros::Time::now();
        marker.ns = "goalpoints";
        marker.id =  10000;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        double x,y,z,ox,oy,oz,ow;
        x = atof(argv[3]);
        y = atof(argv[4]);
        z = atof(argv[5]);
        ox = atof(argv[6]);
        oy = atof(argv[7]);
        oz = atof(argv[8]);
        ow = atof(argv[9]);
        marker.pose.orientation.x = ox;
        marker.pose.orientation.y = oy;
        marker.pose.orientation.z = oz;
        marker.pose.orientation.w = ow;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        ROS_INFO("PUBLISH MARKER %i", marker.id);

        ros::Rate rate(10.0);
        while (ros::ok())
        {
            marker.id = ++marker.id;
            marker.pose.position.x = marker.pose.position.x + .01;
            vis_pub.publish( marker );
            ros::spinOnce();
            rate.sleep();
        }
    }

    //! open both grippers at the same time
    if (atoi(argv[1]) == -26)
    {
        OperateHandleController::openGrippers();
    }

    if (atoi(argv[1]) == -27)
    {
        OperateHandleController::getHandlePoseFromLaser(0);
    }


    //! print bottle pose
    if (atoi(argv[1]) == -28)
    {
        RobotHead::getInstance()->lookAt("/map", 1.243111, -0.728864, 0.9);
        std::vector<tf::Stamped<tf::Pose> *> handlePoses;
        handlePoses.clear();
        tf::Stamped<tf::Pose> bottle = OperateHandleController::getBottlePose();
        ROS_INFO("BOTTLE in %s : %f %f %f", bottle.frame_id_.c_str(), bottle.getOrigin().x(),bottle.getOrigin().y(),bottle.getOrigin().z());
    }

    //! doesnt work
    if (atoi(argv[1]) == -29)
    {
        std::vector<tf::Stamped<tf::Pose> > targetPose;
        std::vector<std::string> frame_ids;
        RobotDriver::getInstance()->driveToMatch(targetPose, frame_ids);
    }

    if (atoi(argv[1]) == -30)
    {
        Gripper::getInstance(1)->updatePressureZero();
        Gripper::getInstance(1)->closeCompliant();
    }

    //! put down the bottle at the island
    if (atoi(argv[1]) == -31)
        DemoScripts::serveBottle();

    //! bring plate to the island next to the pancake heater and present it so that the pancake can be pushed on top
    if (atoi(argv[1]) == -32)
        DemoScripts::servePlateToIsland();

    //! deprecated: get the lego piece, now implemented in another package
    if (atoi(argv[1]) == -33)
    {
        float p[] = {.1, 0.896, 0.003, 1.000};
        //RobotDriver::getInstance()->moveBase(p, false);

        pr2_controllers_msgs::JointTrajectoryGoal goalA = RobotArm::getInstance(0)->lookAtMarker(Poses::prepDishR1,Poses::prepDishR1);
        boost::thread t2(&RobotArm::startTrajectory, RobotArm::getInstance(0), goalA);

        Torso *torso = Torso::getInstance();
        boost::thread t2u(&Torso::up, torso);

        RobotArm::getInstance(0)->tucked = true;

        RobotDriver::getInstance()->moveBase(p, false);

        t2u.join();
        t2.join();

        RobotHead::getInstance()->lookAt("/map",0.449616, 0.9, 1.078124);

        tf::Stamped<tf::Pose> toyPose = OperateHandleController::getBottlePose(); // in map!

        RobotArm *arm = RobotArm::getInstance(0);

        ROS_INFO("LEGO POSE");
        arm->printPose(toyPose);

        tf::Stamped<tf::Pose> toyPoseBase = RobotArm::getInstance(0)->getPoseIn("base_link",toyPose);
        toyPoseBase.setRotation(btQuaternion(0,0,0,1));

        tf::Stamped<tf::Pose> ap_30 = toyPoseBase;
        ap_30.setOrigin(toyPoseBase.getOrigin() + btVector3(-.3,0,0));
        ap_30 = arm->getPoseIn("map", ap_30);

        tf::Stamped<tf::Pose> ap_20 = toyPoseBase;
        ap_20.setOrigin(toyPoseBase.getOrigin() + btVector3(-.2,0,0));
        ap_20 = arm->getPoseIn("map", ap_20);

        tf::Stamped<tf::Pose> ap_10 = toyPoseBase;
        ap_10.setOrigin(toyPoseBase.getOrigin() + btVector3(-.1,0,0));
        ap_10 = arm->getPoseIn("map", ap_10);

        tf::Stamped<tf::Pose> ap_05 = toyPoseBase;
        ap_05.setOrigin(toyPoseBase.getOrigin() + btVector3(-.05,0,0));
        ap_05 = arm->getPoseIn("map", ap_05 );

        tf::Stamped<tf::Pose> ap_00 = toyPoseBase;
        ap_00.setOrigin(toyPoseBase.getOrigin() + btVector3(0,0,0));
        ap_00 = arm->getPoseIn("map", ap_00);


        tf::Stamped<tf::Pose> ap_0010 = toyPoseBase;
        ap_0010.setOrigin(toyPoseBase.getOrigin() + btVector3(0,0,0.15));
        ap_0010 = arm->getPoseIn("map", ap_0010);

        tf::Stamped<tf::Pose> ap_1010 = toyPoseBase;
        ap_1010.setOrigin(toyPoseBase.getOrigin() + btVector3(-.1,0,0.15));
        ap_1010 = arm->getPoseIn("map", ap_1010);

        tf::Stamped<tf::Pose> ap_3010 = toyPoseBase;
        ap_3010.setRotation(btQuaternion(0,0,.707,.707));
        ap_3010.setOrigin(toyPoseBase.getOrigin() + btVector3(-.3,0,0.15));
        ap_3010 = arm->getPoseIn("map", ap_3010);

        arm->printPose(ap_30);
        arm->printPose(ap_20);
        arm->printPose(ap_10);
        arm->printPose(ap_05);
        arm->printPose(ap_00);
        ROS_INFO("-");
        arm->printPose(ap_0010);
        arm->printPose(ap_1010);
        arm->printPose(ap_3010);

        Gripper::getInstance(0)->open();

        arm->universal_move_toolframe_ik_pose(ap_30);
        arm->universal_move_toolframe_ik_pose(ap_20);
        arm->universal_move_toolframe_ik_pose(ap_10);
        arm->universal_move_toolframe_ik_pose(ap_05);
        arm->universal_move_toolframe_ik_pose(ap_00);

        Gripper::getInstance(0)->closeCompliant();
        Gripper::getInstance(0)->close();

        arm->universal_move_toolframe_ik_pose(ap_0010);
        arm->universal_move_toolframe_ik_pose(ap_1010);
        arm->universal_move_toolframe_ik_pose(ap_3010);
    }

    //! put plate or silverware into carrying pose
    if (atoi(argv[1]) == -34)
    {
        tf::Stamped<tf::Pose> rightTip = RobotArm::getInstance(0)->getToolPose();
        tf::Stamped<tf::Pose> leftTip = RobotArm::getInstance(1)->getToolPose();

        leftTip.setOrigin(btVector3(leftTip.getOrigin().x(), leftTip.getOrigin().y(), rightTip.getOrigin().z()));

        RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(leftTip);

        rightTip = RobotArm::getInstance(0)->getToolPose();
        leftTip = RobotArm::getInstance(1)->getToolPose();

        float averageX = (rightTip.getOrigin().x() + leftTip.getOrigin().x()) * 0.5;
        float averageY = (rightTip.getOrigin().y() + leftTip.getOrigin().y()) * 0.5;
        float averageZ = (rightTip.getOrigin().z() + leftTip.getOrigin().z()) * 0.5;

        ROS_INFO("AVERAGE Y pre: %f", averageY);

        OperateHandleController::spinnerL(.45 - averageX,0 - averageY,.935 - averageZ);

    }


    //! serve pancake / bring silverware over to the pillar table
    if (atoi(argv[1]) == -35)
        DemoScripts::serveToTable();

    //! open drawer no. 4 in one pull, for video comparison with haptic approach
    if (atoi(argv[1]) == -36)
    {

        /*Translation: [0.621, 1.421, 0.757]
        - Rotation: in Quaternion [-0.714, -0.010, 0.051, 0.698]
                 in RPY [-1.591, 0.059, 0.085]
        ruehr@satie:~/sandbox/tumros-internal/highlevel/ias_drawer_executive$ rosrun tf tf_echo map r_gripper_tool_frame
        At time 1286637310.022
        - Translation: [0.168, 1.391, 0.734]
        - Rotation: in Quaternion [-0.722, 0.028, 0.105, 0.684]
                 in RPY [-1.615, 0.190, 0.105]

                  Translation: [-0.031, 1.818, 0.051]
        - Rotation: in Quaternion [-0.003, -0.006, 0.031, 0.999]
                 in RPY [-0.006, -0.013, 0.062]


                 */

        {


            std::vector<int> arm;
            std::vector<tf::Stamped<tf::Pose> > goal;
            btVector3 result;

            tf::Stamped<tf::Pose> p0;
            p0.frame_id_="map";
            p0.stamp_=ros::Time();
            p0.setOrigin(btVector3(0.64, 1.421, 0.757));
            p0.setRotation(btQuaternion(-0.714, -0.010, 0.051, 0.698));
            goal.push_back(p0);
            arm.push_back(0);

            tf::Stamped<tf::Pose> p3;
            p3.frame_id_="map";
            p3.stamp_=ros::Time();
            p3.setOrigin(btVector3(0.55, 1.391, 0.734));
            p3.setRotation(btQuaternion(-0.714, -0.010, 0.051, 0.698));
            goal.push_back(p3);
            arm.push_back(0);

            tf::Stamped<tf::Pose> p1;
            p1.frame_id_="map";
            p1.stamp_=ros::Time();
            p1.setOrigin(btVector3(0.168, 1.391, 0.734));
            p1.setRotation(btQuaternion(-0.714, -0.010, 0.051, 0.698));
            goal.push_back(p1);
            arm.push_back(0);

            Gripper::getInstance(0)->open();

            RobotArm::findBaseMovement(result, arm, goal,true, false);

            RobotArm *a = RobotArm::getInstance(0);
            a->time_to_target = 2;
            a->universal_move_toolframe_ik_pose(p3);
            a->universal_move_toolframe_ik_pose(p0);
            Gripper::getInstance(0)->closeCompliant();
            Gripper::getInstance(0)->close();
            //a->universal_move_toolframe_ik_pose(p3);
            a->universal_move_toolframe_ik_pose(p1);
            Gripper::getInstance(0)->open();

            //RobotArm::findBaseMovement(result, arm, goal,false);
        }


    }

    //! open handle assuming the right gripper surrounds it already, it takes the grippers pose and makes a haptic approach + opens
    if (atoi(argv[1]) == -37)
    {
        {
            std::vector<int> arm;
            std::vector<tf::Stamped<tf::Pose> > goal;
            btVector3 result;

            tf::Stamped<tf::Pose> p0;
            p0.frame_id_="map";
            p0.stamp_=ros::Time();
            p0.setOrigin(btVector3(0.64, 1.421, 0.757));
            p0.setRotation(btQuaternion(-0.714, -0.010, 0.051, 0.698));
            goal.push_back(p0);
            arm.push_back(0);

            tf::Stamped<tf::Pose> p3;
            p3.frame_id_="map";
            p3.stamp_=ros::Time();
            p3.setOrigin(btVector3(0.55, 1.391, 0.734));
            p3.setRotation(btQuaternion(-0.714, -0.010, 0.051, 0.698));
            goal.push_back(p3);
            arm.push_back(0);

            tf::Stamped<tf::Pose> p1;
            p1.frame_id_="map";
            p1.stamp_=ros::Time();
            p1.setOrigin(btVector3(0.168, 1.391, 0.734));
            p1.setRotation(btQuaternion(-0.714, -0.010, 0.051, 0.698));
            goal.push_back(p1);
            arm.push_back(0);

            Gripper::getInstance(0)->open();

            RobotArm::findBaseMovement(result, arm, goal,true, false);

            RobotArm *a = RobotArm::getInstance(0);
            a->time_to_target = 2;
            a->universal_move_toolframe_ik_pose(p3);
            a->universal_move_toolframe_ik_pose(p0);
        }
        //tf::Stamped<tf::Pose> p0;
        //p0.frame_id_="map";
        //p0.stamp_=ros::Time();
        //p0.setOrigin(btVector3(0.64, 1.421, 0.757));
        //p0.setRotation(btQuaternion(-0.714, -0.010, 0.051, 0.698));
        //p0 = RobotArm::getInstance(0)->getPoseIn("base_link",p0);


        tf::Stamped<tf::Pose> p0 = RobotArm::getInstance(0)->getToolPose("base_link");

        int handle = OperateHandleController::operateHandle(0,p0);
        //     OperateHandleController::close(handle);
        //Translation: [0.828, 1.973, 1.313]
//- Rotation: in Quaternion [-0.701, -0.031, 0.039, 0.711]
        //          in RPY [-1.556, 0.012, 0.099]


    }



    if (atoi(argv[1]) == -38)
    {
        ros::Rate rate(5);
        while (ros::ok())
        {
            ROS_INFO("GRIPPER OPENING R%f L%f", Gripper::getInstance(0)->getAmountOpen(), Gripper::getInstance(1)->getAmountOpen());
            rate.sleep();
        }
    }


    //! get silverware, bring to pillar like plate with -35
    if (atoi(argv[1]) == -39)
        DemoScripts::takeSilverware();

    //! open drawer for plate picking with fixed map pose -> a little quicker
    if (atoi(argv[1]) == -40)
        DemoScripts::openDrawer();

    //! get plate back from island
    if (atoi(argv[1]) == -41)
        DemoScripts::takePlateFromIsland();

    if (atoi(argv[1]) == -42)
        OperateHandleController::plateTuckPose();


    if (atoi(argv[1]) == -43)
        OperateHandleController::plateAttackPose();

    // open fridge with laser handle
    if (atoi(argv[1]) == -44)
        DemoScripts::openFridge();


    // runs the whole demo once
    if (atoi(argv[1]) == -100)
    {

        int idx = 0;
        if (argc > 1)
            idx = atoi(argv[2]);

        ROS_INFO("INDEX TO DEMO :%i", idx);

        int handle = 0;

        switch (idx)
        {

        case 0:

            handle = DemoScripts::openFridge();

        case 1:

            DemoScripts::takeBottle();

        case 2:

            DemoScripts::closeFridge(handle);

        case 3:

            DemoScripts::serveBottle();

        case 4:

            DemoScripts::openDrawer();

        case 5:

            DemoScripts::takePlate();

        case 6:

            DemoScripts::servePlateToIsland();

        case 7:

            DemoScripts::takeSilverware();

        case 8:

            DemoScripts::serveToTable();

        case 9:

            DemoScripts::takePlateFromIsland();

        case 10:
            //2nd time :)
            DemoScripts::serveToTable();
        }
    }

    /*

       Translation: [0.208, 2.109, 0.051]
       - Rotation: in Quaternion [0.000, 0.001, 0.034, 0.999]
                  in RPY [0.000, 0.001, 0.069]


       */
    // tool frame target poses
    /*- Translation: [-1.728, 2.191, 0.976]
    - Rotation: in Quaternion [-0.308, 0.646, -0.287, 0.637]
            in RPY [-1.602, 0.702, -1.569]
    ruehr@satie:~/sandbox/tumros-internal/highlevel/ias_drawer_executive$ rosrun tf tf_echo map l_gripper_tool_frame
    At time 1286303719.777
    - Translation: [-1.709, 1.923, 0.977]
    - Rotation: in Quaternion [-0.616, -0.318, 0.670, 0.267]
            in RPY [-1.519, 0.713, 1.703]*/

    /*

    first step base

     Translation: [-0.122, 1.015, 0.051]
    - Rotation: in Quaternion [0.001, -0.000, -0.013, 1.000]
    second step base
    Translation: [-0.113, 1.022, 0.051]
    - Rotation: in Quaternion [0.003, -0.001, 0.975, 0.223]
    thir dstep

    -0.901, 1.778, 0.050]
    - Rotation: in Quaternion [-0.000, -0.002, 0.999, -0.051]

    last step via tf target pose

    */

}


/*
- Translation: [0.330, -0.381, 0.051]
- Rotation: in Quaternion [0.001, -0.001, 0.079, 0.997]
            in RPY [0.002, -0.001, 0.159]

0.542329 -0.981344 1.23612 0.0330708  0.00176235 0.595033  0.803019
0.556517 -0.920874 1.23603 0.0298695 -0.00432607 0.545786  0.837381
0.572956 -0.861407 1.23635 0.0268959 -0.00695657 0.494586  0.868685
0.581633 -0.828156 1.23589 0.0252092 -0.00877769 0.475406  0.879362
0.604209 -0.781168 1.23568 0.0179254  0.00291321 0.436192  0.89967
0.622422 -0.748086 1.23539 0.0152842  0.00913191 0.340254  0.940165
0.639726 -0.721124 1.23581 0.0164386  0.0162648  0.278332  0.960206
0.661803 -0.694327 1.23576 0.0219128  0.0176437  0.246738  0.968674
0.698809 -0.657863 1.23729 0.0284113  0.0207615  0.238006  0.970626
0.740968 -0.625336 1.23853 0.0375375  0.0235445  0.227272  0.972823
0.784319 -0.598149 1.23992 0.0450486  0.0276238  0.212765  0.975673
0.82972  -0.572236 1.24111 0.0451087  0.0322855  0.169386  0.983987
0.875225 -0.553171 1.24195 0.0358857  0.032936   0.122281  0.9913
0.956881 -0.532016 1.24365 0.0254018  0.0329301  0.0418438 0.998258
0.972319 -0.530781 1.2467  0.0022027  0.0133422  0.0679044 0.9976
1.02179  -0.52356  1.24665 0.0022027  0.0133422  0.0679044 0.9976
*/
