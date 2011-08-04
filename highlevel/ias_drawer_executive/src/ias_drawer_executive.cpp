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

#include <ias_drawer_executive/Approach.h>
#include <ias_drawer_executive/RobotDriver.h>
#include <ias_drawer_executive/Gripper.h>
#include <ias_drawer_executive/Perception3d.h>
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

#include <articulation_pr2/ArticulatedObjectAction.h>
#include <actionlib/client/simple_action_client.h>

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

void printPose(const char title[], tf::Pose pose)
{
    ROS_INFO("%s %f %f %f %f %f %f %f", title, pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z()
             , pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z(), pose.getRotation().w());
}

void copyState(double *src, double *tar, int num = 7)
{
    for (int i = 0; i < 7 ; ++i)
        tar[i] = src[i];
}

void addState(double *a, double *b, double *tar, int num = 7)
{
    for (int i = 0; i < 7 ; ++i)
        tar[i] = a[i] + b[i];
}

void addStateDiscounted(double *a, double *b, double discount, double *tar, int num = 7)
{
    for (int i = 0; i < 7 ; ++i)
        tar[i] = a[i] + (b[i] * discount);
}



void cart_err_monit(int side)
{
    RobotArm::RobotArm *arm = RobotArm::getInstance(side);
    ros::Rate rate(5);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
        btVector3 err = arm->cartError();
        ROS_INFO("CARTESIAN ERROR :D %f x %f y %f z %f", err.length(), err.x(), err.y(), err.z());
    }
}


void moveBothArms(tf::Stamped<tf::Pose> leftArm, tf::Stamped<tf::Pose> rightArm)
{
    std::vector<int> arm;
    std::vector<tf::Stamped<tf::Pose> > goal;
    btVector3 result;

    arm.push_back(1);
    goal.push_back(leftArm);
    arm.push_back(0);
    goal.push_back(rightArm);

    RobotArm::findBaseMovement(result, arm, goal,true, false);

    boost::thread t1(&RobotArm::universal_move_toolframe_ik_pose,RobotArm::getInstance(0),rightArm);
    boost::thread t2(&RobotArm::universal_move_toolframe_ik_pose,RobotArm::getInstance(1),leftArm);
    t2.join();
    t1.join();

}

ros::NodeHandle *node_handle_ = 0;
bool have_vis_pub = false;
ros::Publisher vis_pub_;

void pubPose(tf::Stamped<tf::Pose> &t)
{

    if (!have_vis_pub)
    {
        vis_pub_ = node_handle_->advertise<geometry_msgs::PoseStamped>( "/cart_poses", 100, true );
        have_vis_pub = true;
    }

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = t.frame_id_;
    pose.header.stamp = ros::Time::now();
    pose.pose.orientation.x = t.getRotation().x();
    pose.pose.orientation.y = t.getRotation().y();
    pose.pose.orientation.z = t.getRotation().z();
    pose.pose.orientation.w = t.getRotation().w();
    pose.pose.position.x = t.getOrigin().x();
    pose.pose.position.y = t.getOrigin().y();
    pose.pose.position.z = t.getOrigin().z();

    //ros::Rate rate(10.0);
    vis_pub_.publish( pose );

    //ROS_INFO("Published Pose");
    for (int k = 0; k < 10; ++k)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
}

void pubPose(tf::Stamped<tf::Pose> &t, ros::Publisher vis_pub)
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = t.frame_id_;
    pose.header.stamp = ros::Time::now();
    pose.pose.orientation.x = t.getRotation().x();
    pose.pose.orientation.y = t.getRotation().y();
    pose.pose.orientation.z = t.getRotation().z();
    pose.pose.orientation.w = t.getRotation().w();
    pose.pose.position.x = t.getOrigin().x();
    pose.pose.position.y = t.getOrigin().y();
    pose.pose.position.z = t.getOrigin().z();

    //ros::Rate rate(10.0);
    vis_pub.publish( pose );

    ROS_INFO("Published Pose");

    ros::spinOnce();
}


tf::Stamped<tf::Pose> turnToSide(int side,tf::Stamped<tf::Pose> toolPose, tf::Stamped<tf::Pose> bowlPose)
{
    toolPose = RobotArm::getInstance(0)->getPoseIn("/base_link", toolPose);
    bowlPose = RobotArm::getInstance(0)->getPoseIn("/base_link", bowlPose);

    RobotArm *arm = RobotArm::getInstance(side);

    tf::Stamped<tf::Pose> bestPose = toolPose;
    tf::Stamped<tf::Pose> worstPose = toolPose;
    float bestX = -100;
    float bestAngle = 0;
    float worstX = 100;

    for (float angle = -M_PI; (angle < 2 * M_PI) && (ros::ok()); angle += M_PI / 10.0)
    {
        btTransform curr;
        curr.setOrigin(toolPose.getOrigin());
        curr.setRotation(toolPose.getRotation());

        bowlPose.setRotation(btQuaternion(0,0,0,1));

        tf::Stamped<tf::Pose> act = arm->rotateAroundPose(toolPose, bowlPose, 0,0, angle);

        tf::Stamped<tf::Pose> actInBase = arm->getPoseIn("/base_link", act);
        float actx = actInBase.getOrigin().y() * (side ? 1 : -1);
        if (actx > bestX)
        {
            bestX = actx;
            bestPose = act;
            bestAngle = angle;
        }
        else if (actx < worstX)
        {
            worstX = actx;
            worstPose = act;
        }
    }

    return bestPose;
}



tf::Stamped<tf::Pose> turnBowlStraight(int currside, tf::Stamped<tf::Pose> &bowlPose)
{

    RobotHead::getInstance()->lookAt("/base_link",1,0,0.5);

    RobotArm *arm = RobotArm::getInstance(currside);

    bowlPose = OperateHandleController::getBowlPose();
    bowlPose = arm->getPoseIn("/base_link", bowlPose);
    pubPose(bowlPose);
    printPose("Bowl Pose vision", bowlPose);

    tf::Stamped<tf::Pose> bowlInBase = arm->getPoseIn("/base_link", bowlPose);
    //bowlInBase.setOrigin(btVector3(0,0,0));

    tf::Stamped<tf::Pose> bowlCorrected = bowlPose;
    bowlCorrected.setRotation(bowlInBase.getRotation().inverse() * bowlPose.getRotation());

    pubPose(bowlCorrected);
    pubPose(bowlCorrected);
    pubPose(bowlCorrected);
    printPose("Bowl Pose corrected", bowlCorrected);

    tf::Stamped<tf::Pose> newToolPose = arm->getToolPose("/base_link");
    printPose("Tool Pose", newToolPose);
    btQuaternion cor = bowlInBase.getRotation().inverse();

    printPose("Bowl Pose", bowlInBase);
    tf::Stamped<tf::Pose> actt = arm->rotateAroundPose(newToolPose, bowlInBase, cor);
    printPose("act Pose", actt);

    //newToolPose.setRotation(bowlInBase.getRotation().inverse() * newToolPose.getRotation());

    printPose("nt Pose (rot)", newToolPose);

    //newToolPose = arm->getPoseIn("/map", newToolPose);
    //return newToolPose;
    return actt;
}

void releaseWhenPulled(int side)
{

    RobotArm *arm = RobotArm::getInstance(side);
    Gripper *grip = Gripper::getInstance(side);
    ros::Rate rate(5);
    float threshold = 0;
    float numtaken = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
        btVector3 err = arm->cartError();
        ROS_INFO("CARTESIAN ERROR :D %f x %f y %f z %f", err.length(), err.x(), err.y(), err.z());

        threshold += err.length();
        numtaken += 1;

        ROS_INFO("CARTESIAN ERROR :D %f (%f) x %f y %f z %f", err.length(), threshold / numtaken, err.x(), err.y(), err.z());

        if ((numtaken > 5) && (err.length() > (threshold / numtaken) * 3.0))
        {
            boost::thread t1(&Gripper::open, grip, 0.09);
            tf::Stamped<tf::Pose> toolPose = arm->getToolPose("/map");
            toolPose.setOrigin(toolPose.getOrigin() + btVector3(0,0,0.05));
            arm->universal_move_toolframe_ik_pose(toolPose);
            return;
        }

    }
}


int main(int argc, char** argv)
{
    // Init the ROS node
    //char nodename[] = "ias_drawer_executive"'
    //sprintf(nodename,"%s%i",nodename,

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
    node_handle_ = &node_handle;


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
                    //boost::shared_ptr<boost::thread> t;
                    //if (atoi(argv[i* 2 + 1])==8)
                    //t = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&Torso::up, torso)));
                    //else
                    //t = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&Torso::down, torso)));
                    boost::thread t((atoi(argv[i* 2 + 1])==8) ? &Torso::up : &Torso::down, torso);
                    ROS_INFO("TARGET POSE IN MAP %f %f %f %f",Poses::poses[atoi(argv[i* 2 + 1])][0],Poses::poses[atoi(argv[i* 2 + 1])][1],Poses::poses[atoi(argv[i* 2 + 1])][2],Poses::poses[atoi(argv[i* 2 + 1])][3]);
                    RobotDriver::getInstance()->moveBase(Poses::poses[atoi(argv[i* 2 + 1])]);
                    t.join();
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

    if (atoi(argv[1]) == -45)
    {

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

    if (atoi(argv[1]) == -300)
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
        double elbow_hint = atof(argv[10]);
        RobotArm::getInstance(atoi(argv[2]))->raise_elbow = true;
        RobotArm::getInstance(atoi(argv[2]))->preset_angle = elbow_hint;
        printf("moving tool to ik goal pos in map %f %f %f or %f %f %f %f \n", x,y,z,ox,oy,oz,ow);
        RobotArm::getInstance(atoi(argv[2]))->universal_move_toolframe_ik(x,y,z,ox,oy,oz,ow,"map");
        RobotArm::getInstance(atoi(argv[2]))->raise_elbow = false;
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
        RobotArm::getInstance(atoi(argv[2]))->universal_move_toolframe_ik(x,y,z,ox,oy,oz,ow,"base_link");
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

    //! put gripper to some pose in frame
    if (atoi(argv[1]) == -6)
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
        RobotArm::getInstance(atoi(argv[2]))->universal_move_toolframe_ik(x,y,z,ox,oy,oz,ow,argv[10]);
    }


    //if (atoi(argv[1]) == -6)
    //{
        //while (ros::ok())
        //{
            //tf::Stamped<tf::Pose> aM = AverageTF::getMarkerTransform("/4x4_1",20);
        //}
    //}

    //if (atoi(argv[1]) == -7)
    //{
        //RobotArm::getInstance(1)->startTrajectory(RobotArm::getInstance(1)->lookAtMarker(Poses::lf0,Poses::lf1));
        //RobotArm::getInstance(1)->startTrajectory(RobotArm::getInstance(1)->lookAtMarker(Poses::lf2,Poses::lf3));
    //}


    //if (atoi(argv[1]) == -8)
    //{
    //    printf("ias_drawer_executive -8 arm(0=r,1=l) r_x r_y r_z frame_id");
    //    printf("rotate around frame downstream from wrist");
    //   RobotArm::getInstance(atoi(argv[2]))->rotate_toolframe_ik(atof(argv[3]),atof(argv[4]),atof(argv[5]), argv[6]);
    //}

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
            tf::Pose diffPose = oldposes[k].inverse() * oldposes[k-1];
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
        if (argc == 4)
            Gripper::getInstance(atoi(argv[2]))->close(atof(argv[3]));
        else
            Gripper::getInstance(atoi(argv[2]))->close();
    }

    if (atoi(argv[1]) == -20)
    {
        Torso::getInstance()->pos(atof(argv[2]));
    }

    //! look at a pos in frame
    if (atoi(argv[1]) == -22)
    {
        //RobotHead::getInstance()->lookAt(argv[2],atof(argv[3]),atof(argv[4]),atof(argv[5]));
        RobotHead::getInstance()->lookAtThreaded(argv[2],atof(argv[3]),atof(argv[4]),atof(argv[5]));
        ros::Rate rt(10);
        int numpy = 50;
        while (ros::ok() && (--numpy > 0))
        {
            ros::spinOnce();
            rt.sleep();
        }
        RobotHead::getInstance()->stopThread();
        while (ros::ok())
        {
            ros::spinOnce();
            rt.sleep();
        }
    }

    if (atoi(argv[1]) == -23)
    {
        tf::Stamped<tf::Pose> in;
        in.setOrigin(btVector3(atof(argv[2]),atof(argv[3]),atof(argv[4])));
        in.frame_id_ = "base_link";
        tf::Stamped<tf::Pose> out = RobotArm::getInstance(0)->getPoseIn("map",in);
        printf("pos in map %f %f %f \n", out.getOrigin().x(),out.getOrigin().y(),out.getOrigin().z());
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
        Perception3d::getHandlePoseFromLaser(0);
    }


    //! print bottle pose
    if (atoi(argv[1]) == -28)
    {
        RobotHead::getInstance()->lookAt("/map", 1.243111, -0.728864, 0.9);
        std::vector<tf::Stamped<tf::Pose> *> handlePoses;
        handlePoses.clear();
        tf::Stamped<tf::Pose> bottle = Perception3d::getBottlePose();
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
        Gripper::getInstance(atoi(argv[2]))->updatePressureZero();
        Gripper::getInstance(atoi(argv[2]))->closeCompliant(atof(argv[3]));
    }

    //! put down the bottle at the island
    if (atoi(argv[1]) == -31)
        DemoScripts::serveBottle();

    //! bring plate to the island next to the pancake heater and present it so that the pancake can be pushed on top
    if (atoi(argv[1]) == -32)
        DemoScripts::servePlateToIsland();

    //! deprecated: get the lego piece, now implemented in another package
    /*if (atoi(argv[1]) == -33)
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

        tf::Stamped<tf::Pose> toyPose = Perception3d::getBottlePose(); // in map!

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
    }*/

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
        /*{
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
        }*/
        //tf::Stamped<tf::Pose> p0;
        //p0.frame_id_="map";
        //p0.stamp_=ros::Time();
        //p0.setOrigin(btVector3(0.64, 1.421, 0.757));
        //p0.setRotation(btQuaternion(-0.714, -0.010, 0.051, 0.698));
        //p0 = RobotArm::getInstance(0)->getPoseIn("base_link",p0);


        tf::Stamped<tf::Pose> p0 = RobotArm::getInstance(0)->getToolPose("base_link");

        OperateHandleController::operateHandle(0,p0);
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
        OperateHandleController::plateCarryPose();

    //DemoScripts::openFridge();


    // runs the whole demo once
    if (atoi(argv[1]) == -100)
    {

        int idx = 0;
        if (argc > 1)
            idx = atoi(argv[2]);

        ROS_INFO("INDEX TO DEMO :%i", idx);

        //int handle = 0;

        switch (idx)
        {

        case 0:

            //handle = DemoScripts::openFridge();

        case 1:

            //DemoScripts::takeBottle();

        case 2:

            //DemoScripts::closeFridge(handle);
            DemoScripts::takeBottleFromFridge();

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
        case 11:
            //2nd time :)
            DemoScripts::putObjectIntoFridge();
        }
    }

    if (atoi(argv[1]) == -101)
    {
        //OperateHandleController::getPlate(0);

        RobotDriver::getInstance()->moveBaseP(-0.571, 2.819,0.729, 0.685);

        OperateHandleController::pickPlate(btVector3(-0.600, 3.453, 0.87));


        //RobotArm::getInstance(0)->stabilize_grip();
        //RobotArm::getInstance(1)->stabilize_grip();
        //OperateHandleController::plateCarryPose();
    }


    if (atoi(argv[1]) == -102)
    {
        RobotArm::getInstance(atoi(argv[2]))->stabilize_grip();
    }

    if (atoi(argv[1]) == -103)
    {

        ros::Rate rate(5.0);

        while (ros::ok())
        {
            btVector3 diff = RobotArm::getInstance(0)->cartError();
            ROS_INFO("DIFF %f %f %f DISTANCE %f", diff.x(), diff.y(), diff.y(), diff.length());
            ros::spinOnce();
            rate.sleep();
        }
    }

    // primitive and dumb approach of lifting elbow keeping endeffector in place
    if (atoi(argv[1]) == -104)
    {

        ros::Rate rate(5.0);

        double jiggle = 0;

        RobotArm *arm = RobotArm::getInstance(0);

        double state[7];
        double goalstate[7];

        double primitive[100][7];
        int numinprim = 0;

        double mp[100][7];
        for (int k = 0; k < 100; k ++)
        {
            for (int j = 0; j < 7; j++)
            {
                mp[k][j] = 0;
                primitive[k][j] = 0;
            }
        }
        mp[0][0] = 1;
        mp[1][1] = 1;
        mp[2][2] = 1;
        mp[3][3] = 1;
        mp[4][4] = 1;
        mp[5][5] = 1;
        mp[6][6] = 1;
        //int pcnt = 8;

        arm->getJointState(state);
        arm->getJointState(goalstate);

        tf::Stamped<tf::Pose> goalP = arm->runFK(state);

        while (ros::ok())
        {

            jiggle += -.01;

            double add[] = {0,jiggle,0,0,0,0,0};

            double curr[7];

            addState(state,add,curr);

            copyState(curr,state);

            double initialincrement = 0.075;
            //initialincrement = 0.01;

            double increment = initialincrement;

            tf::Stamped<tf::Pose> newp = arm->runFK(curr);

            double oldDist= (goalP.getOrigin() - newp.getOrigin()).length();
            double oldDistAngle = (goalP.getRotation().inverse() * newp.getRotation()).getAngle();

            double flzeroes[] = {0,1,0,0,0,0,0,0,0,0,0};
            int numflippedzero = 1;
            double flipped[7];
            copyState(flzeroes,flipped,9);
            int numflipped = numflippedzero;

            int metric = 0; // o = distance, 1 = angle

            bool found = false;

            while ((!found) && (ros::ok()))
            {
                if (numflipped == 7)
                {
                    increment /= 2.0f;
                    numflipped =numflippedzero;
                    copyState(flzeroes,flipped,9);
                    ROS_INFO("DECREMENTING INCREMENT to %f", increment);
                }

                int jnt = rand() % 7;
                while (flipped[jnt] == 1)
                    jnt = rand() % 7;

                //if (numflipped == numflippedzero)
                //jnt = 7 + metric;

                flipped[jnt] = 1;
                numflipped += 1;

                //double act_add[] = {0,0,0,0,0,0,0};
                //act_add[jnt] = increment;
                double currm[7];
                double currp[7];

                //addState(state,act_add,curr);
                copyState(state,currm);
                copyState(state,currp);

                float distance = 0;
                for (int k = 0; k < 7; k++)
                    distance += mp[jnt][k] * mp[jnt][k];
                distance = sqrtf(distance);

                float actual_increment = increment / distance;

                for (int k = 0; k < 7; k++)
                {
                    currm[k] -= mp[jnt][k] * actual_increment;
                    currp[k] += mp[jnt][k] * actual_increment;
                }

                //tf::Stamped<tf::Pose> oldp = arm->runFK(state);
                tf::Stamped<tf::Pose> newp = arm->runFK(currp);
                double distp = (goalP.getOrigin() - newp.getOrigin()).length();
                tf::Stamped<tf::Pose> newm = arm->runFK(currm);
                double distm = (goalP.getOrigin() - newm.getOrigin()).length();

                double distpangle = (goalP.getRotation().inverse() * newp.getRotation()).getAngle();
                double distmangle = (goalP.getRotation().inverse() * newm.getRotation()).getAngle();

                //if (oldDist < 0.01)
                //metric = false;
                //else
                //  metric = true;
                bool madeMove = false;
                double effect = 0;
                if (metric == 0)
                {
                    if (distp < oldDist)
                    {
                        //state[jnt] += increment;
                        //change[jnt] += increment;
                        effect = actual_increment;
                        oldDist = distp;
                        oldDistAngle = distpangle;
                        madeMove = true;
                    }
                    else if (distm < oldDist)
                    {
                        //state[jnt] -= increment;
                        //change[jnt] -= increment;
                        effect = -actual_increment;
                        oldDist = distm;
                        oldDistAngle = distmangle;
                        madeMove = true;
                    }
                }
                else
                {
                    if (distpangle < oldDistAngle)
                    {
                        //state[jnt] += increment;
                        //change[jnt] += increment;
                        effect = actual_increment;
                        oldDist = distp;
                        oldDistAngle = distpangle;
                        madeMove = true;
                    }
                    else if (distmangle < oldDistAngle)
                    {
                        //state[jnt] -= increment;
                        //change[jnt] -= increment;
                        effect = -actual_increment;
                        oldDist = distm;
                        oldDistAngle = distmangle;
                        madeMove = true;
                    }
                }


                if (madeMove)
                {
                    addStateDiscounted(state, mp[jnt], effect, state);
                    ROS_ERROR("%i Sucessful increment %f Resetting INCREMENT %f , METRIC : %i", jnt ,effect, initialincrement, metric);
                    increment = increment * 2;
                    if (increment > initialincrement)
                        increment = initialincrement;
                    numflipped = numflippedzero;
                    copyState(flzeroes,flipped,9);
                    //metric = !metric;
                    addStateDiscounted(primitive[metric],mp[jnt],effect,primitive[metric]);
                    for (int j = 0; j < 7; j ++)
                    {
                        mp[7][j] = primitive[0][j];
                        mp[8][j] = primitive[1][j];
                    }

                    numinprim ++;
                    ROS_ERROR("Motion primitive D: %f %f %f %f %f %f %f", primitive[0][0], primitive[0][1],primitive[0][2],primitive[0][3],primitive[0][4],primitive[0][5],primitive[0][6]);
                    ROS_ERROR("Motion primitive A: %f %f %f %f %f %f %f", primitive[1][0], primitive[1][1],primitive[1][2],primitive[1][3],primitive[1][4],primitive[1][5],primitive[1][6]);
                }

//                ROS_INFO("%i STATE: %f %f %f %f %f %f %f, D :old  %f plus %f minus %f ANG %f %f %f",jnt, state[0],state[1],state[2],state[3],state[4],state[5],state[6], oldDist,distp,distm,
                //                       oldDistAngle, distpangle, distmangle);
                ROS_INFO("%s %i d %f a %f STATE: %f %f %f %f %f %f %f", (metric==0) ? "dist" : "angl", jnt, oldDist, oldDistAngle, state[0],state[1],state[2],state[3],state[4],state[5],state[6]);

                if ((oldDist < 0.01) && (metric == 0))
                    metric = 1;
                if ((oldDistAngle < .05) && (metric == 1))
                    metric = 0;
                //metric = !metric;

                if ((oldDist < 0.01) && (oldDistAngle < 0.05))
                {
                    float poseA[7];
                    float poseB[7];

                    ROS_INFO(" ");
                    ROS_INFO(" ");
                    ROS_INFO(" ");
                    ROS_INFO(" ");
                    ROS_ERROR("MOVING ARM NOW");
                    ROS_INFO(" ");
                    ROS_INFO(" ");
                    ROS_INFO(" ");
                    ROS_INFO(" ");
                    ROS_INFO("OLD STATE %f %f %f %f %f %f %f", goalstate[0],goalstate[1],goalstate[2],goalstate[3],goalstate[4],goalstate[5],goalstate[6]);
                    ROS_INFO("NEW STATE %f %f %f %f %f %f %f", state[0],state[1],state[2],state[3],state[4],state[5],state[6]);

                    double stateDiff[] = {0,0,0,0,0,0,0};
                    addStateDiscounted(goalstate,state,-1,stateDiff);
                    ROS_INFO("DIFF STATE %f %f %f %f %f %f %f", stateDiff[0],stateDiff[1],stateDiff[2],stateDiff[3],stateDiff[4],stateDiff[5],stateDiff[6]);


                    for (int k = 0; k < 7; ++k)
                    {
                        poseA[k] = goalstate[k];
                        poseB[k] = state[k];
                        goalstate[k] = state[k];
                    }
                    arm->startTrajectory(arm->lookAtMarker(poseA, poseB));
                    found = true;
                }

            }
        }
    }


    if (atoi(argv[1]) == -105)
    {

        RobotArm *arm = RobotArm::getInstance(0);
        float amount =0;

        float start[] = {-0.896923, 0.513388, -1.284203, -1.535167, 4.452332, -0.837711, -9.765933};
        float d[] = {0.037500, 0.250000, 0.337500, 0.000000, -0.300000, -0.300000, 0.000000};
        float next[7] = {-0.896923, 0.513388, -1.284203, -1.535167, 4.452332, -0.837711, -9.765933};
        float act[7];


        while (ros::ok())
        {
            amount += 0.1;

            for (int k = 0; k < 7; ++k)
            {
                act[k] = next[k];
                next[k] = start[k] + (- d[k] * amount);
            }
            ROS_INFO("amount %f" , amount);
            arm->startTrajectory(arm->lookAtMarker(start, next));

        }

    }

    if (atoi(argv[1]) == -106)
    {

        int side = atoi(argv[2]);
        RobotArm *arm = RobotArm::getInstance(side);
        double state[7];
        arm->getJointState(state);
        double solution[7];

        tf::Stamped<tf::Pose> actPose;
        arm->getWristPose(actPose,"base_link");

        geometry_msgs::PoseStamped stamped_pose;
        stamped_pose.header.frame_id = "base_link";
        stamped_pose.header.stamp = ros::Time::now();
        stamped_pose.pose.position.x=actPose.getOrigin().x();
        stamped_pose.pose.position.y=actPose.getOrigin().y();
        stamped_pose.pose.position.z=actPose.getOrigin().z();
        stamped_pose.pose.orientation.x=actPose.getRotation().x();
        stamped_pose.pose.orientation.y=actPose.getRotation().y();
        stamped_pose.pose.orientation.z=actPose.getRotation().z();
        stamped_pose.pose.orientation.w=actPose.getRotation().w();

        double increment = -0.25;

        double last = 0;
        int numup = 0;
        ros::Rate rate(2.0);

        double lastState[7];
        float dif[7];
        int numit = 0;

        for (double add = 0; ros::ok(); add+= 0.01)
        {
            rate.sleep();
            state[2] += increment;
            //double new_state[7];
            //bool f =
            arm->run_ik(stamped_pose,state,solution,side == 0 ? "r_wrist_roll_link" : "l_wrist_roll_link");
            ROS_INFO("inc %f SEED STATE %f %f %f %f %f %f %f",increment, state[0],state[1],state[2],state[3],state[4],state[5],state[6]);
            ROS_INFO("%f SOLUTION %f %f %f %f %f %f %f", state[1], solution[0], solution[1],solution[2], solution[3], solution[4], solution[5], solution[6]);
            float pose[7];
            numit++;

            //if (numit < 4)

            float sum = 0;
            for (int k = 0; k < 7; ++k)
            {
                pose[k] = solution[k];
                sum += pose[k] * pose[k];
            }

            if (sum > 0.01)
                arm->startTrajectory(arm->goalTraj(pose),false);

            ///arm->startTrajectory(arm->goalTraj(pose));
            //else {
            //m->startTrajectory(arm->goalTraj(pose, dif));
            //ROS_INFO("VELOCITY");
            //}

            if ((last == solution[1]) && (numup > 3))
            {
                increment=-increment;
                numup = -4;
            }

            ROS_ERROR("DIFFERENCE %f %f %f %f %f %f %f",lastState[0] - solution[0], lastState[1] - solution[1],lastState[2] - solution[2], lastState[3] - solution[3], lastState[4] - solution[4], lastState[5] - solution[5], lastState[6] - solution[6]);

            for (int k = 0; k < 7 ; ++k)
            {
                dif[k] = (-(lastState[k] - solution[k])) / 1.0;
                lastState[k] = solution[k];
                numit ++;
            }

            last = solution[1];

            numup++;
            //pr2_controllers_msgs::JointTrajectoryGoal goalTraj(float *poseA);
        }


        //bool RobotArm::run_ik(geometry_msgs::PoseStamped pose, double start_angles[7],double solution[7], std::string link_name)

    }



    if (atoi(argv[1]) == -107)
    {

        int side = 1;
        RobotArm *arm = RobotArm::getInstance(side);
        double state[7];
        arm->getJointState(state);
        //double solution[7];

        tf::Stamped<tf::Pose> actPose;
        arm->getWristPose(actPose,"base_link");

        geometry_msgs::PoseStamped stamped_pose;
        stamped_pose.header.frame_id = "base_link";
        stamped_pose.header.stamp = ros::Time::now();
        stamped_pose.pose.position.x=actPose.getOrigin().x();
        stamped_pose.pose.position.y=actPose.getOrigin().y();
        stamped_pose.pose.position.z=actPose.getOrigin().z();
        stamped_pose.pose.orientation.x=actPose.getRotation().x();
        stamped_pose.pose.orientation.y=actPose.getRotation().y();
        stamped_pose.pose.orientation.z=actPose.getRotation().z();
        stamped_pose.pose.orientation.w=actPose.getRotation().w();


        //double last = 0;
        //int numup = 0;
        ros::Rate rate(50.0);

        //double lastState[7];
        //float dif[7];
        //int numit = 0;

        //for (double add = 0; ros::ok(); add+= 0.01)
        while (ros::ok())
        {
            rate.sleep();
            ros::spinOnce();

            double jErr[7];
            double jDes[7];

            arm->getJointStateErr(jErr);
            arm->getJointStateDes(jDes);

            float increment = .1;

            //if (fabs(jErr[1]) > 0.01) {
            {

                double stateP[7];
                double statePK[7];
                double stateM[7];
                //double solStateP[7];
                //double solStatePK[7];
                //double solStateM[7];

                arm->getJointState(stateP);
                arm->getJointState(statePK);
                arm->getJointState(stateM);

                double stA[7];
                double stB[7];
                double stC[7];
                arm->getJointState(stA);
                arm->getJointState(stB);
                arm->getJointState(stC);

                stB[2] += increment;

                double stAs[7];
                double stBs[7];
                double stCs[7];

                arm->run_ik(stamped_pose,stA,stAs,side == 0 ? "r_wrist_roll_link" : "l_wrist_roll_link");
                arm->run_ik(stamped_pose,stB,stBs,side == 0 ? "r_wrist_roll_link" : "l_wrist_roll_link");

                double newinc = (jErr[1] / (stBs[1] - stAs[1])) * increment;

                stC[2] += newinc;

                arm->run_ik(stamped_pose,stC,stCs,side == 0 ? "r_wrist_roll_link" : "l_wrist_roll_link");

                ROS_INFO("curr (sta[1]) %f  stCs %f", stA[1], stCs[1]);

                float pose[7];
                float sum = 0;
                for (int k = 0; k < 7; ++k)
                {
                    pose[k] = stCs[k];
                    sum += stCs[k] * stCs[k];
                }

                if (sum > 0.01)
                    arm->startTrajectory(arm->goalTraj(pose));

                if (stBs[2]==0)
                {
                    increment = -increment;
                    ROS_INFO("HIT LIMIT?");
                }



            }


        }

    }


    //! open handle at map coordinate, expects base to be put more or less frontal to drawer surface and almost in reach
    if (atoi(argv[1]) == -108)
    {
        ROS_INFO("OPEN HANDLE AT MAP COORDINATE  arm orientation(0=vert 1=horiz) x_map y_map z_map");

        //tf::Stamped<tf::Pose> p0 = RobotArm::getInstance(0)->getToolPose("base_link");
        tf::Stamped<tf::Pose> p0;
        p0.setOrigin(btVector3(atof(argv[4]),atof(argv[5]),atof(argv[6])));
        p0.frame_id_ = "map";
        p0 = RobotArm::getInstance(atoi(argv[2]))->getPoseIn("base_link",p0);

        if (atoi(argv[3]) == 0)
            p0.setRotation(btQuaternion(1,0,0,0));
        else
            p0.setRotation(btQuaternion(0.707,0,0,0.707));

        OperateHandleController::operateHandle(atoi(argv[2]),p0);
    }


    //! pick a plate from map coords
    if (atoi(argv[1]) == -109)
    {
        //OperateHandleController::plateTuckPose();
        OperateHandleController::plateAttackPose();

        //bin/ias_drawer_executive -109 .482 1.073 0.761

        //OperateHandleController::pickPlate(btVector3(atof(argv[2]),atof(argv[3]),atof(argv[4])), .28);
        OperateHandleController::pickPlate(btVector3(atof(argv[2]),atof(argv[3]),atof(argv[4])), .27);

        OperateHandleController::spinnerL(0,0,-.2);

        OperateHandleController::openGrippers();

    }




    if (atoi(argv[1]) == -110)
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
                    if (k == 0)
                    {

                        RobotHead::getInstance()->lookAt("/map", 1.243111, -0.728864, 0.9);
                        tf::Stamped<tf::Pose> bottle = Perception3d::getBottlePose();

                        float ptA[] = {0.41491862845470812, 1.3468554401788568, 1.501748997727044, -2.0247783614692936, -16.507431415382143, -1.3292235155277217, 15.027356561279952};
                        float ptB[] = {0.040263624618489424, 0.96465557759293075, 0.27150676981727662, -1.6130504582945409, -14.582800985450046, -1.1869058378819473, 14.819427432123987};
                        RobotArm *arml = RobotArm::getInstance(1);
                        arml->startTrajectory(arml->goalTraj(ptA,1.5));
                        arml->startTrajectory(arml->goalTraj(ptB,1.5));
                        Gripper::getInstance(1)->open();
                        RobotArm::getInstance(1)->universal_move_toolframe_ik(bottle.getOrigin().x() - .1, bottle.getOrigin().y(), bottle.getOrigin().z() + .03, 0.005, -0.053, -0.029005, 0.998160, "map");
                        RobotArm::getInstance(1)->universal_move_toolframe_ik(bottle.getOrigin().x(), bottle.getOrigin().y(), bottle.getOrigin().z() + .03, 0.005, -0.053, -0.029005, 0.998160, "map");

                        Gripper::getInstance(1)->closeCompliant();
                        Gripper::getInstance(1)->close(0.5);
                        RobotArm::getInstance(1)->universal_move_toolframe_ik(bottle.getOrigin().x() - .1, bottle.getOrigin().y(), bottle.getOrigin().z() + .03, 0.005, -0.053, -0.029005, 0.998160, "map");
                        arml->startTrajectory(arml->goalTraj(ptB,1.5));
                        RobotArm::getInstance(1)->universal_move_toolframe_ik(0.208, 0.120, 0.785 , -0.020, -0.101, -0.662, 0.742, "base_link");
                    }
                }
            }
        }

        Gripper::getInstance(0)->open();

        float target[4];
        target[0] = -0.3;
        target[1] = 0;
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



    if (atoi(argv[1]) == -111)
    {

        RobotArm::RobotArm *arm = RobotArm::getInstance(0);
        Torso *torso = Torso::getInstance();
        Gripper *gripper = Gripper::getInstance(0);

        boost::thread t2(&Torso::up, torso);

        OperateHandleController::plateTuckPose();

        RobotDriver::getInstance()->moveBaseP(0.305, -0.515, -0.348, 0.938, false);
        t2.join();

        tf::Stamped<tf::Pose> handleHint;
        handleHint.setOrigin(btVector3( 0.919, -0.553, 1.35 ));
        handleHint.frame_id_ = "/map";
        tf::Stamped<tf::Pose> handlePos = Perception3d::getHandlePoseFromLaser(handleHint);

        //yes TWO Times
        handlePos = Perception3d::getHandlePoseFromLaser(handleHint);


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



    if (atoi(argv[1]) == -112)
    {

        OperateHandleController::plateTuckPose();

        RobotDriver::getInstance()->moveBaseP(0.305, -0.515, -0.348, 0.938, false);
        {
            std::vector<int> arm;
            std::vector<tf::Stamped<tf::Pose> > goal;
            btVector3 result;

            tf::Stamped<tf::Pose> p0;
            p0.frame_id_="map";
            p0.stamp_=ros::Time();
            p0.setOrigin(btVector3(0.8, -0.455, 1.251));
            p0.setRotation(btQuaternion(0.005, -0.053, -0.029, 0.998));
            //goal.push_back(p0);
            //arm.push_back(1);

            tf::Stamped<tf::Pose> p1;
            p1.frame_id_="map";
            p1.stamp_=ros::Time();
            p1.setOrigin(btVector3(0.8, -0.655, 1.251));
            p1.setRotation(btQuaternion(0.005, -0.053, -0.029, 0.998));
            //goal.push_back(p1);
            //arm.push_back(1);

            btVector3 bottle(1.227, -0.741, 0.986);

            tf::Stamped<tf::Pose> p2;
            p2.frame_id_="map";
            p2.stamp_=ros::Time();
            //p2.setOrigin(btVector3(1.165 - fridgeLink, -0.655, 1.151));
            p2.setOrigin(bottle + btVector3(-.1,0,.03));
            p2.setRotation(btQuaternion(0.005, -0.053, -0.029, 0.998));
            //goal.push_back(p2);
            //arm.push_back(1);

            tf::Stamped<tf::Pose> p3;
            p3.frame_id_="map";
            p3.stamp_=ros::Time();
            //p3.setOrigin(btVector3(1.265000 - fridgeLink, -0.655000, 0.951000));
            p3.setOrigin(bottle + btVector3(.02,0,.03));
            p3.setRotation(btQuaternion(0.005001, -0.053009, -0.029005, 0.998160));
            goal.push_back(p3);
            arm.push_back(1);

            RobotArm::findBaseMovement(result, arm, goal,false, false);
            //RobotArm::findBaseMovement(result, arm, goal,false);
        }

    }


    if (atoi(argv[1]) == -113)
    {
        ros::Rate rt(1);
        float ptA[] = {0.41491862845470812, 1.3468554401788568, 1.501748997727044, -2.0247783614692936, -16.507431415382143, -1.3292235155277217, 15.027356561279952};
        float ptB[] = {0.040263624618489424, 0.96465557759293075, 0.27150676981727662, -1.6130504582945409, -14.582800985450046, -1.1869058378819473, 14.819427432123987};
        RobotArm *arml = RobotArm::getInstance(1);
        arml->startTrajectory(arml->goalTraj(ptA));
        rt.sleep();
        rt.sleep();
        arml->startTrajectory(arml->goalTraj(ptB));
        rt.sleep();
        rt.sleep();
        Gripper::getInstance(1)->open();
        Gripper::getInstance(1)->close(0.5);
        arml->startTrajectory(arml->goalTraj(ptB));
        rt.sleep();
        rt.sleep();
        arml->startTrajectory(arml->goalTraj(ptA));
        rt.sleep();
        rt.sleep();
    }


    if (atoi(argv[1]) == -120)
    {
        RobotArm *arm = RobotArm::getInstance(atoi(argv[2]));
        float pose[7];
        for (int k = 0; k < 7; ++k)
        {
            pose[k] = atof(argv[3+k]);
        }
        float duration = 5;
        if (argc > 10)
            duration = atof(argv[10]);
        arm->startTrajectory(arm->goalTraj(pose,duration));
    }

    if (atoi(argv[1]) == -121)
    {
        RobotArm *arm = RobotArm::getInstance(atoi(argv[2]));
        float pose[7];
        for (int k = 0; k < 7; ++k)
        {
            pose[k] = atof(argv[3+k]);
        }
        ros::Rate rate(5);

        while (ros::ok())
        {
            arm->startTrajectory(arm->goalTraj(pose),false);
            pose[6] += atof(argv[10]);
            ros::spinOnce();
            rate.sleep();
        }
    }

    if (atoi(argv[1]) == -130)
    {
        Pressure *r = Pressure::getInstance(0);
        float a[22], b[22];
        ros::Rate rate(25);
        float sum = 0;
        float num = 0;

        float table[100];
        float tablesum=0;
        for (int i= 0; i < 100; ++i)
        {
            table[i] = 0;
        }

        for (int x = 0; x < 10; ++x)
        {
            r->getCurrent(a,b,false);
            rate.sleep();
        }
        while (ros::ok())
        {
            float last = a[11];
            r->getCurrent(a,b,false);
            rate.sleep();
            sum += fabs(a[11]-last);
            num += 1;

            float logchange = log(fabs(a[11] - last) + 1 );
            if (logchange > 9.9)
                logchange = 9.9;
            logchange *= 10;
            table[int(logchange)] += 1;
            tablesum+=1;
            float posi = 0;
            int i;
            for (i = 0; i <= int(logchange); ++i)
                posi += table[i];

            ROS_INFO("at %.5fi %.3i logchange %.5f ", posi / tablesum, i, logchange);
            //ROS_INFO("log change %f 11 : %f , change %f, avg change %f", logchange, a[11], last-a[11], sum / num);
        }
    }


    if (atoi(argv[1]) == -131)
    {
        cart_err_monit(atoi(argv[2]));
    }

    if (atoi(argv[1]) == -200)
    {
        //tf::Stamped<tf::Pose> leftEdge;
        //leftEdge.setOrigin(btVector3(-.27389, 1.529, .888));
        //leftEdge.setOrigin(btVector3(-.3855, 2.243, 0.938));
        //leftEdge.setOrigin(btVector3(atof(argv[2]),atof(argv[3]),atof(argv[4])));
        //leftEdge.frame_id_ = "map";
        //tf::Stamped<tf::Pose> rightEdge;
        //rightEdge.frame_id_ = "map";
        //rightEdge.setOrigin(btVector3(0.0107, 1.797, .878));
        //rightEdge.setOrigin(btVector3(-.35585, 1.908, .94));
        //rightEdge.setOrigin(btVector3(atof(argv[5]),atof(argv[6]),atof(argv[7])));
        //btVector3 rel = leftEdge.getOrigin() - rightEdge.getOrigin();
        //float at2 = atan2(rel.y(), rel.x());
        //btQuaternion ori(btVector3(0,0,1), at2);

        //leftEdge.setRotation(ori);
        //rightEdge.setRotation(ori);

        boost::thread tp1(OperateHandleController::plateTuckPose);

        RobotHead::getInstance()->lookAt("/map",-.5,2.1,0.9,true);

        tp1.join();

        tf::Stamped<tf::Pose> midEdge;
        midEdge.frame_id_ = "map";
        //midEdge.setRotation(ori);
        //midEdge.setOrigin((rightEdge.getOrigin() + leftEdge.getOrigin()) * .5f);

        ros::Publisher ppub = node_handle_->advertise<geometry_msgs::PoseStamped>( "/cart_roi", 100, true );
        tf::Stamped<tf::Pose> roi_center;
        roi_center.frame_id_ = "/map";
        roi_center.setOrigin(btVector3(-.5,2.1,0.9));
        roi_center.setRotation(btQuaternion(0,0,0,1)); // switch on
        pubPose(roi_center,ppub);

        geometry_msgs::PoseStamped midpose = *(ros::topic::waitForMessage<geometry_msgs::PoseStamped>("output_grip_pose"));

        roi_center.setOrigin(btVector3(-1,-1,-1)); // deactivate bread detection by sending negative z roi
        pubPose(roi_center,ppub);

        btVector3 pnt;
        tf::pointMsgToTF(midpose.pose.position,pnt);
        btQuaternion quat;
        tf::quaternionMsgToTF(midpose.pose.orientation, quat);

        midEdge.setOrigin( pnt );
        midEdge.setRotation( quat );


        tf::Stamped<tf::Pose> baseInCart;
        baseInCart.frame_id_ = "map";
        baseInCart.setOrigin(btVector3(-0.002, -0.378, -0.864));
        baseInCart.setRotation(btQuaternion(-0.000, 0.001, 0.713, 0.701));

        btTransform basePose = midEdge * baseInCart;
        tf::Stamped<tf::Pose> newBasePose(basePose,ros::Time::now(),"map");
        //newBasePose.frame_id_ = "/map";
        //newBasePose.setOrigin(basePose);
        //newBasePose.setRotation(btQuaternion(-0.000, 0.001, 0.713, 0.701));

        RobotDriver::getInstance()->driveInMap(newBasePose);

        ros::Rate rt(5);

        tf::Stamped<tf::Pose> rightGrasp;
        rightGrasp.frame_id_ = "map";
        //rightGrasp.setOrigin(btVector3(-0.363, 2.004, 0.953));
        //rightGrasp.setRotation(btQuaternion(0.531, -0.495, 0.517, 0.455));

        tf::Stamped<tf::Pose> leftGrasp;
        leftGrasp.frame_id_ = "map";
        //leftGrasp.setOrigin(btVector3(-0.380, 2.159, 0.957));
        //leftGrasp.setRotation(btQuaternion(-0.457, -0.506, -0.500, 0.535));

        // gripper poses relative to handle center coordinate frame
        tf::Pose lRel;
        //lRel = midEdge.inverseTimes(leftGrasp);
        lRel.setOrigin(btVector3(0.08, 0, 0.02));
        lRel.setRotation(btQuaternion(0.681431, 0.004566, 0.731829, 0.007564));
        printPose("lRel",lRel);

        tf::Pose rRel;
        rRel.setOrigin(btVector3(-0.08, 0 ,0.02));
        rRel.setRotation(btQuaternion(-0.006577, -0.725437, 0.013462, 0.688126));
        rRel.setRotation(btQuaternion(0.681431, 0.004566, 0.731829, 0.007564));
        //rRel = midEdge.inverseTimes(rightGrasp);
        printPose("rRel",rRel);

        tf::Pose btrightGrasp =  midEdge * rRel ; // tf::Pose nxt = start * rel;
        tf::Pose btleftGrasp =  midEdge * lRel;

        rightGrasp.setOrigin(btrightGrasp.getOrigin());
        leftGrasp.setOrigin(btleftGrasp.getOrigin());

        rightGrasp.setRotation(btrightGrasp.getRotation());
        leftGrasp.setRotation(btleftGrasp.getRotation());

        tf::Stamped<tf::Pose> rightBase = RobotArm::getPoseIn("base_link", rightGrasp);
        tf::Stamped<tf::Pose> leftBase = RobotArm::getPoseIn("base_link", leftGrasp);

        // bigger y is left
        if (rightBase.getOrigin().y() > leftBase.getOrigin().y())
        {
            tf::Stamped<tf::Pose> tmp = rightGrasp;
            rightGrasp = leftGrasp;
            leftGrasp = tmp;
        }


        tf::Stamped<tf::Pose> preLeft;
        preLeft.setOrigin(btVector3(0.3, .1, .75));
        preLeft.frame_id_ = "base_link";

        tf::Stamped<tf::Pose> preRight;
        preRight.setOrigin(btVector3(0.3,  -.1, .75));
        preRight.frame_id_ = "base_link";

        preRight = RobotArm::getPoseIn("map",preRight);
        preLeft = RobotArm::getPoseIn("map",preLeft);

        preRight.setRotation(rightGrasp.getRotation());
        preLeft.setRotation(leftGrasp.getRotation());

        preRight = RobotArm::getPoseIn("base_link",preRight);
        preLeft = RobotArm::getPoseIn("base_link",preLeft);

        //{
        //    boost::thread t1(&RobotArm::universal_move_toolframe_ik_pose,RobotArm::getInstance(0),preRight);
        //    boost::thread t2(&RobotArm::universal_move_toolframe_ik_pose,RobotArm::getInstance(1),preLeft);
        //    t2.join();
        //    t1.join();
        // }
        moveBothArms(preLeft, preRight);

        //boost::thread t0(&RobotArm::universal_move_toolframe_ik,RobotArm::getInstance(0),0.3, -.1, .75, -0.563, -0.432, -0.465, 0.530,"base_link");
        //RobotArm::getInstance(1)->universal_move_toolframe_ik(0.3,  .1, .75, -0.563, -0.432, -0.465, 0.530,"base_link");
        //t0.join();

        tf::Stamped<tf::Pose> rightGraspSub = rightGrasp;
        rightGraspSub.setOrigin(rightGraspSub.getOrigin() - btVector3(0,0,.1));
        tf::Stamped<tf::Pose> leftGraspSub = leftGrasp;
        leftGraspSub.setOrigin(leftGraspSub.getOrigin() - btVector3(0,0,.1));

        if (1)
        {
            //RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(rightGraspSub);
            //{
            //    boost::thread t1(&RobotArm::universal_move_toolframe_ik_pose,RobotArm::getInstance(0),rightGraspSub);
            //    boost::thread t2(&RobotArm::universal_move_toolframe_ik_pose,RobotArm::getInstance(1),leftGraspSub);
            //    t2.join();
            //    t1.join();
            //}

            moveBothArms(leftGraspSub, rightGraspSub);

            OperateHandleController::openGrippers();

            //{
            //   boost::thread t1(&RobotArm::universal_move_toolframe_ik_pose,RobotArm::getInstance(0),rightGrasp);
            //    boost::thread t2(&RobotArm::universal_move_toolframe_ik_pose,RobotArm::getInstance(1),leftGrasp);
            //    t2.join();
            //    t1.join();
            // }
            moveBothArms(leftGrasp, rightGrasp);

            boost::thread a(&Gripper::close,Gripper::getInstance(0),0);
            boost::thread b(&Gripper::close,Gripper::getInstance(1),0);

            a.join();
            b.join();


            //{
            //    boost::thread t1(&RobotArm::universal_move_toolframe_ik_pose,RobotArm::getInstance(0),rightGraspPl);
            //    boost::thread t2(&RobotArm::universal_move_toolframe_ik_pose,RobotArm::getInstance(1),leftGraspPl);
            //    t2.join();
            //    t1.join();
            // }

            tf::Stamped<tf::Pose> rightGraspPl = rightGrasp;
            rightGraspPl = RobotArm::getPoseIn("base_link", rightGrasp);
            rightGraspPl.setOrigin(rightGraspPl.getOrigin() + btVector3(0.1,0,0));
            tf::Stamped<tf::Pose> leftGraspPl = leftGrasp;
            leftGraspPl = RobotArm::getPoseIn("base_link", leftGrasp);
            leftGraspPl.setOrigin(leftGraspPl.getOrigin() + btVector3(0.1,0,0));

            moveBothArms(leftGraspPl, rightGraspPl);

            //{
            //    boost::thread t1(&RobotArm::universal_move_toolframe_ik_pose,RobotArm::getInstance(0),rightGrasp);
            //    boost::thread t2(&RobotArm::universal_move_toolframe_ik_pose,RobotArm::getInstance(1),leftGrasp);
            //    t2.join();
            //    t1.join();
            // }

            moveBothArms(leftGrasp, rightGrasp);

            OperateHandleController::openGrippers();

            //{
            //    boost::thread t1(&RobotArm::universal_move_toolframe_ik_pose,RobotArm::getInstance(0),rightGraspSub);
            //    boost::thread t2(&RobotArm::universal_move_toolframe_ik_pose,RobotArm::getInstance(1),leftGraspSub);
            //    t2.join();
            //    t1.join();
            // }

            moveBothArms(leftGraspSub, rightGraspSub);


            //{
            //    boost::thread t1(&RobotArm::universal_move_toolframe_ik_pose,RobotArm::getInstance(0),preRight);
            //    boost::thread t2(&RobotArm::universal_move_toolframe_ik_pose,RobotArm::getInstance(1),preLeft);
            //    t2.join();
            //    t1.join();
            // }
            moveBothArms(preLeft, preRight);
        }

        ROS_INFO("Entering nop loop, spinning");
        if (0)
            while (ros::ok())
            {
                //pubPose(leftEdge);
                //printPose("leftEdge" , leftEdge);
                //rt.sleep();
                //ros::spinOnce();
                //pubPose(rightEdge);
                //printPose("rightEdge" , rightEdge);
                //rt.sleep();
                //ros::spinOnce();

                pubPose(rightGrasp);
                printPose("rightGrasp" , rightGrasp);
                rt.sleep();
                ros::spinOnce();
                pubPose(leftGrasp);
                printPose("leftGrasp" , leftGrasp);
                rt.sleep();
                ros::spinOnce();
            }

    }


    if (atoi(argv[1]) == -201)
    {
        while (ros::ok())
        {
            pr2_controllers_msgs::JointTrajectoryGoal goalAT = RobotArm::getInstance(0)->lookAtMarker(Poses::prepDishRT,Poses::prepDishRT);
            pr2_controllers_msgs::JointTrajectoryGoal goalBT = RobotArm::getInstance(1)->lookAtMarker(Poses::prepDishLT,Poses::prepDishLT);
            boost::thread t2T(&RobotArm::startTrajectory, RobotArm::getInstance(0), goalAT,true);
            boost::thread t3T(&RobotArm::startTrajectory, RobotArm::getInstance(1), goalBT,true);
            t2T.join();
            t3T.join();

            pr2_controllers_msgs::JointTrajectoryGoal goalA = RobotArm::getInstance(0)->lookAtMarker(Poses::prepDishR1,Poses::prepDishR1);
            pr2_controllers_msgs::JointTrajectoryGoal goalB = RobotArm::getInstance(1)->lookAtMarker(Poses::prepDishL1,Poses::prepDishL1);
            boost::thread t2(&RobotArm::startTrajectory, RobotArm::getInstance(0), goalA,true);
            boost::thread t3(&RobotArm::startTrajectory, RobotArm::getInstance(1), goalB,true);
            t2.join();
            t3.join();
        }

    }

    if (atoi(argv[1]) == -202)
    {
        while (ros::ok())
        {
            RobotArm::getInstance(1)->universal_move_toolframe_ik(0.195, 0.215, 0.621, -0.438, 0.529, -0.163, 0.708,"base_link");
            RobotArm::getInstance(1)->universal_move_toolframe_ik(0.297, 0.357, 0.987, -0.457, 0.446, -0.339, 0.690,"base_link");
        }
    }

    if (atoi(argv[1]) == -203)
    {

        RobotHead::getInstance()->lookAt("/map",-1.625371,1.143165,0.833442,true);

        //activate bread detection by sending roi
        ros::Publisher ppub = node_handle_->advertise<geometry_msgs::PoseStamped>( "/bread_roi", 100, true );
        tf::Stamped<tf::Pose> roi_center;
        roi_center.frame_id_ = "/map";
        roi_center.setOrigin(btVector3(-1.7,1.28,0.833442));
        roi_center.setRotation(btQuaternion(0,0,0,1)); // switch on
        pubPose(roi_center,ppub);

        tf::Stamped<tf::Pose> midEdge;
        midEdge.frame_id_ = "map";
        midEdge.setRotation(btQuaternion(-0.473, 0.504, 0.504, 0.518));
        //midEdge.setOrigin((rightEdge.getOrigin() + leftEdge.getOrigin()) * .5f);

        geometry_msgs::PoseStamped midpose = *(ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/bread_pose"));

        roi_center.setOrigin(btVector3(-1,-1,-1)); // deactivate bread detection by sending negative z roi
        pubPose(roi_center,ppub);

        btVector3 pnt;
        tf::pointMsgToTF(midpose.pose.position,pnt);

        RobotArm::getInstance(1)->universal_move_toolframe_ik(0.297, 0.357, 0.987, -0.457, 0.446, -0.339, 0.690,"base_link");

        midEdge.setOrigin(pnt + btVector3(0,0,0.02) + btVector3(0,0,0.1));
        RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(midEdge);

        Gripper::getInstance(1)->open();

        midEdge.setOrigin(pnt + btVector3(0,0,0.02) + btVector3(0,0,0));
        RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(midEdge);

        Gripper::getInstance(1)->updatePressureZero();
        Gripper::getInstance(1)->closeCompliant();
        Gripper::getInstance(1)->close(0.04);

        midEdge.setOrigin(pnt + btVector3(0,0,0.02) + btVector3(0,0,0.05));
        RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(midEdge);

        tf::Stamped<tf::Pose> putdown;
        putdown.frame_id_ = "/map";
        putdown.setOrigin(btVector3(-1.595, 0.907, midEdge.getOrigin().z() + 0.03));
        putdown.setRotation(midEdge.getRotation()); // switch on

        //midEdge.setOrigin(pnt + btVector3(0,0,0.02) + btVector3(0,0,0));
        RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(putdown);

        putdown.setOrigin(btVector3(-1.595, 0.907, midEdge.getOrigin().z() - 0.02));
        RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(putdown);

        Gripper::getInstance(1)->open();

        putdown.setOrigin(btVector3(-1.595, 0.907, midEdge.getOrigin().z() + 0.03));
        RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(putdown);


        //midEdge.setOrigin(pnt + btVector3(0,0,0.02) + btVector3(0,0,0.1));

        RobotArm::getInstance(1)->universal_move_toolframe_ik(0.297, 0.357, 0.987, -0.457, 0.446, -0.339, 0.690,"base_link");

        //- Translation: [-1.595, 0.907, 0.876]- Rotation: in Quaternion [-0.627, -0.019, 0.778, -0.034]


    }


    if (atoi(argv[1]) == -204)
    {

        RobotHead::getInstance()->lookAt("/map",-1.625371,1.143165,0.833442,true);
        //activate bread detection by sending roi
        ros::Publisher ppub = node_handle_->advertise<geometry_msgs::PoseStamped>( "/bread_roi", 100, true );
        tf::Stamped<tf::Pose> roi_center;
        roi_center.frame_id_ = "/map";
        roi_center.setOrigin(btVector3(-1.7,1.28,0.833442));
        roi_center.setRotation(btQuaternion(0,0,0,1)); // switch on
        pubPose(roi_center,ppub);
        ros::Rate rt(10);
        for (int j = 0; j < 50 ; j++)
        {
            rt.sleep();
            ros::spinOnce();
        }

    }

    if (atoi(argv[1]) == -205)
    {

        RobotHead::getInstance()->lookAt("/map",-1.625371,1.143165,0.833442,true);

        //activate bread detection by sending roi
        ros::Publisher ppub = node_handle_->advertise<geometry_msgs::PoseStamped>( "/bread_roi", 100, true );
        tf::Stamped<tf::Pose> roi_center;
        roi_center.frame_id_ = "/map";
        roi_center.setOrigin(btVector3(-1,-1,-1)); // deactivate bread detection by sending negative z roi
        roi_center.setRotation(btQuaternion(0,0,0,1)); // switch on
        pubPose(roi_center,ppub);
        ros::Rate rt(10);
        for (int j = 0; j < 50 ; j++)
        {
            rt.sleep();
            ros::spinOnce();
        }
    }


    if (atoi(argv[1]) == -133)
    {
        OperateHandleController::plateAttackPose();

        RobotArm *arm = RobotArm::getInstance(atoi(argv[2]));

        double x,y,z,ox,oy,oz,ow;
        x = atof(argv[3]);
        y = atof(argv[4]);
        z = atof(argv[5]);
        ox = atof(argv[6]);
        oy = atof(argv[7]);
        oz = atof(argv[8]);
        ow = atof(argv[9]);
        printf("moving tool to ik goal pos in map %f %f %f or %f %f %f %f \n", x,y,z,ox,oy,oz,ow);
        arm->universal_move_toolframe_ik(x,y,z,ox,oy,oz,ow,"map");
        double state[7];
        arm->getJointState(state);

        float pose[7];
        for (int k = 0; k < 7; ++k)
        {
            pose[k] = state[k];
            ROS_INFO("k %i , state %f", k, pose[k]);
        }
        pose[0] = 0;
        float duration = 20;
        //if (argc > 10)
        //duration = atof(argv[10]);

        arm->startTrajectory(arm->goalTraj(pose,duration));
    }

    // slice it, dont dice it
    if (atoi(argv[1]) == -210)
    {

        Torso::getInstance()->up();

        float numslices = atoi(argv[2]); // 0 = one slice for computer scientists! -1 = none
        float slicethickness = 0.02;


        boost::thread t0(&OperateHandleController::plateAttackPose);

        RobotHead::getInstance()->lookAt("/slicer",0,0,-0.3);

        tf::Stamped<tf::Pose> newBasePose;
        newBasePose.frame_id_ = "/map";
        newBasePose.setOrigin(btVector3(-2.679, 1.397, 0.052));
        newBasePose.setRotation(btQuaternion(-0.001, 0.000, 0.046, 0.999));

        RobotDriver::getInstance()->driveInMap(newBasePose);

        t0.join();

        RobotHead::getInstance()->lookAt("/slicer",0,0,-0.3);


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
        butup.setOrigin(btVector3(0.118, -0.037, 0.14));
        butup.setRotation(btQuaternion(-0.011, 0.705, 0.000, 0.709));
        butup.frame_id_ = "slicer";
        tf::Stamped<tf::Pose> butdown;
        //butdown.setOrigin(btVector3(0.118, -0.037, 0.08));
        //butdown.setOrigin(btVector3(0.118, -0.037, 0.07));
        butdown.setOrigin(btVector3(0.118, -0.037, 0.065));
        butdown.setRotation(btQuaternion(-0.011, 0.705, 0.000, 0.709));
        butdown.frame_id_ = "slicer";

        RobotArm *rarm = RobotArm::getInstance(0);
        Gripper *rgrip = Gripper::getInstance(0);
        RobotArm *larm = RobotArm::getInstance(1);
        Gripper *lgrip = Gripper::getInstance(1);

        ros::Rate onesec(1);

        if (1)
        {
            tf::Stamped<tf::Pose> nextPoseR = pur;
            nextPoseR.setOrigin(pur.getOrigin() + btVector3(-numslices * slicethickness,0,0.15));
            rarm->universal_move_toolframe_ik_pose(nextPoseR);
            rgrip->open();

            nextPoseR.setOrigin(pur.getOrigin() + btVector3(-numslices * slicethickness,0,0));
            rarm->universal_move_toolframe_ik_pose(nextPoseR);
            rgrip->close(0.04);

            //nextPoseR.setOrigin(pur.getOrigin() + btVector3(-numslices * slicethickness -0.05,0,0.02));
            //rarm->universal_move_toolframe_ik_pose(nextPoseR);

            boost::thread buttona(&RobotArm::move_toolframe_ik_pose, larm, butup);

            nextPoseR.setOrigin(pur.getOrigin() + btVector3(-numslices * slicethickness -0.05,0,0.05));
            rarm->universal_move_toolframe_ik_pose(nextPoseR);

            buttona.join();

            lgrip->close();

            boost::thread button(&RobotArm::move_toolframe_ik_pose, larm, butdown);

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

                rarm->universal_move_toolframe_ik_pose(nextPoseR);


                if (nums > 0)
                {
                    nextPoseR = pre;
                    nextPoseR.setOrigin(pre.getOrigin()  + btVector3(-nums * slicethickness, 0,0));
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

            OperateHandleController::plateAttackPose();

        }


        if (0)
        {
            larm->move_toolframe_ik_pose(butup);
            lgrip->close();
            larm->move_toolframe_ik_pose(butdown);
            onesec.sleep();
            onesec.sleep();
            onesec.sleep();
            larm->move_toolframe_ik_pose(butup);
        }



        ros::Rate rt(10);

        ROS_INFO("Entering nop loop, spinning");
        if (0)
            while (ros::ok())
            {
                //pubPose(leftEdge);
                //printPose("leftEdge" , leftEdge);
                //rt.sleep();
                //ros::spinOnce();
                //pubPose(rightEdge);
                //printPose("rightEdge" , rightEdge);
                //rt.sleep();
                //ros::spinOnce();

                pubPose(midEdge);
                printPose("midEdge" , midEdge);
                rt.sleep();
                ros::spinOnce();
                //pubPose(leftGrasp);
                //printPose("leftGrasp" , leftGrasp);
                rt.sleep();
                ros::spinOnce();
            }

    }

    if (atoi(argv[1]) == -211)
    {

        Pressure::getInstance(0);
        Pressure::getInstance(1);

        OperateHandleController::plateTuckPose();

        tf::Stamped<tf::Pose> newBasePose;
        newBasePose.frame_id_ = "/map";
        newBasePose.setOrigin(btVector3(-2.679, 1.397, 0.052));
        newBasePose.setRotation(btQuaternion(-0.001, 0.000, 0.046, 0.999));

        RobotDriver::getInstance()->driveInMap(newBasePose);

        OperateHandleController::plateAttackPose();

        tf::Stamped<tf::Pose> start;
        start.frame_id_ = "/map";
        start.setOrigin(btVector3(-2.144, 1.69, 0.889));
        start.setRotation(btQuaternion(-0.270, 0.666, -0.247, 0.650));

        tf::Stamped<tf::Pose> end;
        end.frame_id_ = "/map";
        end.setOrigin(btVector3(-2.153, 1.573, 0.898));
        end.setRotation(btQuaternion(-0.270, 0.666, -0.247, 0.650));

        OperateHandleController::singleSidedPick(1,start,end);

        tf::Stamped<tf::Pose> larm = RobotArm::getInstance(1)->getToolPose("/map");
        btVector3 pos = larm.getOrigin();
        pos.setY(2.15);
        larm.setOrigin(pos);

        RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(larm);

        //Translation: [-2.278, 1.979, 0.868] - Rotation: in Quaternion [-0.671, 0.187, 0.279, 0.661]
        larm.setOrigin(btVector3(-2.278, 1.979, 0.87));
        larm.setRotation(btQuaternion(-0.671, 0.187, 0.279, 0.661));

        // [-2.226, 2.135, 0.891]
        //- Rotation: in Quaternion [-0.376, 0.550, -0.068, 0.742]

        larm.setOrigin(btVector3(-2.226, 2.135, 0.87));
        larm.setRotation(btQuaternion(-0.376, 0.550, -0.068, 0.742));
        RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(larm);


        //Translation: [-2.300, 2.051, 0.892]
        //- Rotation: in Quaternion [-0.533, 0.468, 0.116, 0.695]

        larm.setOrigin(btVector3(-2.300, 2.051, 0.87));
        larm.setRotation(btQuaternion(-0.533, 0.468, 0.116, 0.695));
        RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(larm);

        //RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(larm);

        Gripper::getInstance(1)->open();

        //exit(0);

        OperateHandleController::plateAttackPose();

        //drive to grasp plate pose
        //RobotDriver::getInstance()->moveBaseP(-2.844, 1.403,0.055, 0.998);
        RobotDriver::getInstance()->moveBaseP(-2.804, 1.95,  0.055, 0.998);

        OperateHandleController::pickPlate(btVector3(-2.18, 1.95, 0.865),.26);

        tf::Stamped<tf::Pose> basePos;
        RobotDriver::getInstance()->getRobotPose(basePos);
        basePos.setOrigin(basePos.getOrigin() + btVector3(-.15,0,0));

        RobotDriver::getInstance()->driveInMap(basePos);

        RobotDriver::getInstance()->moveBaseP(-3.390, -1.027, -0.697, 0.717);

        OperateHandleController::spinnerL(0.25,0,-.25);

        OperateHandleController::openGrippers();

        OperateHandleController::plateAttackPose();


    }
    if (atoi(argv[1]) == -212)
    {
        Pressure::getInstance(0);
        Pressure::getInstance(1);
        OperateHandleController::plateAttackPose();

        RobotDriver::getInstance()->moveBaseP(-2.685, 2.022,0,1);
        RobotHead::getInstance()->lookAt("/map",-1.8,1.6,.5,true);

        tf::Stamped<tf::Pose> bowlPose_ = OperateHandleController::getBowlPose();
        btVector3 bowlPose = bowlPose_.getOrigin();// = OperateHandleController::getBowlPose();

        //btVector3 bowlPose = OperateHandleController::getBowlPose();
        bowlPose.setZ(.909);

        tf::Stamped<tf::Pose> start;
        start.frame_id_ = "/map";
        //start.setOrigin(btVector3(-2.215, 1.955, 0.909));
        start.setOrigin(bowlPose - btVector3(.15,0,0));
        start.setRotation(btQuaternion(-0.631, 0.304, 0.379, 0.605));
        printPose("start", start);

        tf::Stamped<tf::Pose> end;
        end.frame_id_ = "/map";
        //end.setOrigin(btVector3(-2.07, 1.955, 0.909));
        end.setOrigin(bowlPose - btVector3(0.05,0,0));
        end.setRotation(btQuaternion(-0.631, 0.304, 0.379, 0.605));
        printPose("end", end);

        OperateHandleController::singleSidedPick(0,start,end);
        //- Translation: [-2.130, 1.967, 0.883]     -  Rotation: in Quaternion [-0.505, 0.472, 0.540, 0.480]

        //[-2.215, 1.955, 0.909] Rotation: in Quaternion [-2.215, 1.955, 0.909]

        tf::Stamped<tf::Pose> pitch;
        pitch = RobotArm::getInstance(0)->getToolPose("/map");
        pitch.setRotation(btQuaternion(-0.615489,0.343613,0.456079,0.543226));

        RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(pitch);

        tf::Stamped<tf::Pose> carry;
        carry.frame_id_ = "/base_link";
        carry.setOrigin(btVector3(0.347, -0.045, 0.986));
        carry.setRotation(btQuaternion(-0.610, -0.094, 0.737, 0.276));
        RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(carry);

        RobotDriver::getInstance()->moveBaseP(-2.981, 2.031, 0.002, 1.000);

        RobotDriver::getInstance()->moveBaseP(-3.180, -1.117,  -0.725, 0.689);

        RobotArm::getInstance(0)->universal_move_toolframe_ik(-3.085, -1.792, 0.768, -0.371, 0.651, 0.132, 0.649,"/map");

        Gripper::getInstance(0)->open();

        OperateHandleController::plateAttackPose();

    }




    // runs the whole demo once
    if (atoi(argv[1]) == -500)
    {

        int idx = 0;
        if (argc > 1)
            idx = atoi(argv[2]);

        ROS_INFO("INDEX TO DEMO :%i", idx);

        //int handle = 0;

        switch (idx)
        {

        case 0:

            DemoScripts::sliceTheBread(3);

        case 1:

            DemoScripts::takeBreadPlate(0);

        case 2:

            DemoScripts::takeBowl(0);

        }
    }

    if (atoi(argv[1]) == -304)
    {
        //    while (ros::ok())
        {

            tf::Stamped<tf::Pose> handleHint;

            handleHint.setOrigin(btVector3(atof(argv[3]),atof(argv[4]),atof(argv[5])));
            handleHint.frame_id_ = argv[2];

            handleHint = RobotArm::getPoseIn("/base_link", handleHint);

            tf::Stamped<tf::Pose> handlePos = Perception3d::getHandlePoseFromLaser(handleHint);

            ros::Rate rt(10);
            int numsec = 5;
            while (ros::ok() && (--numsec > 0))
            {
                rt.sleep();
                ros::spinOnce();
                pubPose(handlePos);
            }


            tf::Stamped<tf::Pose> preGraspPose = handlePos;
            //preGraspPose = RobotArm::getPoseIn("/base_link", preGraspPose);
            preGraspPose.setOrigin(preGraspPose.getOrigin() - btVector3(0.15,0,0)); // todo substract_in_frame
            preGraspPose = RobotArm::getPoseIn("/map", preGraspPose);

            OperateHandleController::graspHandle(0, handlePos);

            tf::Stamped<tf::Pose> actualGraspPose = RobotArm::getInstance(0)->getToolPose("/map");

            //RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(preGraspPose);

            //RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(actualGraspPose);

            articulate();

            Gripper::getInstance(0)->open();

            RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(preGraspPose);

            boost::thread t1(&Gripper::close, Gripper::getInstance(0), 0);

            OperateHandleController::plateAttackPose();
            OperateHandleController::plateTuckPose();
        }
    }


    if (atoi(argv[1]) == -305)
    {

        bool right = true;

        while (ros::ok())
        {
            RobotHead::getInstance()->lookAt("/base_link",1,0,0.5);

            right = !right;

            if (!((argc > 2) && (atoi(argv[2]) == -1)))
                OperateHandleController::plateAttackPose();

            tf::Stamped<tf::Pose> bowlPose = OperateHandleController::getBowlPose();
            pubPose(bowlPose);
            pubPose(bowlPose);
            pubPose(bowlPose);
            printPose("Bowl Pose", bowlPose);

            if ((argc > 2) && (atoi(argv[2]) == -1))
                exit(0);

            tf::Stamped<tf::Pose> prepur; //pick up the bread
            prepur.setOrigin(btVector3(-0.015, -0.078, 0.067));
            prepur.setRotation(btQuaternion(0.726, 0.004, -0.686, -0.043));
            prepur.frame_id_ = "bowly";
            btTransform prepur_ = bowlPose * prepur;//tf::Pose btrightGrasp =  midEdge * rRel ;

            tf::Stamped<tf::Pose> pur; //pick up the bread
            pur.setOrigin(btVector3(-0.015, -0.078, 0.017));
            pur.setRotation(btQuaternion(0.726, 0.004, -0.686, -0.043));
            pur.frame_id_ = "bowly";
            btTransform pur_ = bowlPose * pur;//tf::Pose btrightGrasp =  midEdge * rRel ;

            tf::Stamped<tf::Pose> pickup;
            pickup.frame_id_ = "map";
            pickup.setOrigin(pur_.getOrigin());
            pickup.setRotation(pur_.getRotation());

            tf::Stamped<tf::Pose> prepickup;
            prepickup.frame_id_ = "map";
            prepickup.setOrigin(prepur_.getOrigin());
            prepickup.setRotation(prepur_.getRotation());

            RobotArm *arm = RobotArm::getInstance(right ? 0 : 1);
            arm->time_to_target = 0;
            Gripper *grip = Gripper::getInstance(right ? 0 : 1);

            boost::thread t1(&Gripper::open, grip, 0.09);

            Pressure::getInstance(right ? 0 : 1)->reset();
            arm->universal_move_toolframe_ik_pose(prepickup);

            t1.join();

            arm->universal_move_toolframe_ik_pose(pickup);
            {
                RobotArm *arm = RobotArm::getInstance(right ? 0 : 1);
                tf::Stamped<tf::Pose> toolPose = arm->getToolPose("map");

                btTransform pur;
                pur.setOrigin(btVector3(-0.015, -0.078, 0.017));
                pur.setRotation(btQuaternion(0.726, 0.004, -0.686, -0.043));
                //btTransform pur_ = ;//tf::Pose btrightGrasp =  midEdge * rRel ;
                btTransform tool;
                tool.setOrigin(toolPose.getOrigin());
                tool.setRotation(toolPose.getRotation());

                btTransform id;
                id.setIdentity();
                pur = pur.inverseTimes(id);

                btTransform bowl = tool * pur;

                //tf::Stamped<tf::Pose> bowlPose;
                bowlPose.frame_id_ = "/map";
                bowlPose.setOrigin(bowl.getOrigin());
                bowlPose.setRotation(bowl.getRotation());

                pubPose(bowlPose);
                printPose("Bowl Pose calc", bowlPose);
            }

            grip->closeCompliant();
            grip->close();

            arm->stabilize_grip();

            tf::Stamped<tf::Pose> tpush = arm->getToolPose("/map");
            tpush.setOrigin(tpush.getOrigin() + btVector3(0,0,-.02));
            arm->universal_move_toolframe_ik_pose(tpush);

            arm->stabilize_grip();

            tf::Stamped<tf::Pose> tp = arm->getToolPose("/map");
            tf::Stamped<tf::Pose> tp_high = arm->getToolPose("/map");
            tp_high = arm->getPoseIn("/base_link",tp_high);
            //tp_high.setOrigin(tp_high.getOrigin() + btVector3(-.1,right ? -0.1 : 0.1,0.1));
            tp_high.setOrigin(tp_high.getOrigin() + btVector3(-.1,0,0.1));

            arm->universal_move_toolframe_ik_pose(tp_high);

            if (argc > 2)
                exit(0);

            arm->universal_move_toolframe_ik_pose(tp);
            grip->open();
            arm->universal_move_toolframe_ik_pose(prepickup);

            OperateHandleController::plateAttackPose();


        }
    }


    if (atoi(argv[1]) == -306)
    {

        int currside = atoi(argv[2]);

        bool closingCalibrated[2] = {false, false};
        float minClosing[2] = {0.0025, 0.0025};

        while (ros::ok())
        {

            RobotHead::getInstance()->lookAt("/base_link",1,0,0.5);

            tf::Stamped<tf::Pose> bowlPose; // = OperateHandleController::getBowlPose();
            pubPose(bowlPose);
            printPose("Bowl Pose vision", bowlPose);

            RobotArm *arm = RobotArm::getInstance(currside);
            Gripper *gripper = Gripper::getInstance(currside);
            //float side = (currside ? 1 : -1);
            tf::Stamped<tf::Pose> toolPose = arm->getToolPose("map");

            btTransform pur; //pick up the bread
            pur.setOrigin(btVector3(-0.015, -0.078, 0.017));
            pur.setRotation(btQuaternion(0.726, 0.004, -0.686, -0.043));
            //btTransform pur_ = ;//tf::Pose btrightGrasp =  midEdge * rRel ;
            btTransform tool;
            tool.setOrigin(toolPose.getOrigin());
            tool.setRotation(toolPose.getRotation());

            btTransform id;
            id.setIdentity();
            pur = pur.inverseTimes(id);

            btTransform bowl = tool * pur;

            //tf::Stamped<tf::Pose> bowlPose;
            bowlPose.frame_id_ = "/map";
            bowlPose.setOrigin(bowl.getOrigin());
            bowlPose.setRotation(bowl.getRotation());

            //pubPose(bowlPose);
            //printPose("Bowl Pose calc", bowlPose);

            //arm->evil_switch = true;
            arm->time_to_target = 0.3;

            ros::Rate rt(2);

            //turn to best ext
            tf::Stamped<tf::Pose> bestPose = turnToSide(currside,toolPose,bowlPose);

            arm->universal_move_toolframe_ik_pose(bestPose);

            // bowl vision turn
            tf::Stamped<tf::Pose> straightPose = turnBowlStraight(currside, bowlPose);
            straightPose = turnToSide(currside,straightPose,bowlPose);

            arm->universal_move_toolframe_ik_pose(straightPose);

            tf::Stamped<tf::Pose> worstPose = turnToSide(currside ? 0 : 1,straightPose,bowlPose);

            RobotArm *otherarm = RobotArm::getInstance(currside  ? 0 : 1);

            tf::Stamped<tf::Pose> highapp = worstPose;
            highapp.setOrigin(worstPose.getOrigin() + btVector3(0,0,0.05));
            Gripper *othergripper = Gripper::getInstance(currside ? 0 : 1);

            otherarm->universal_move_toolframe_ik_pose(highapp);

            float amountopen = 0;
            float additional_depth = -.014;
            while (amountopen < minClosing[currside ? 0 : 1] + 0.003)
            {
                othergripper->open();
                tf::Stamped<tf::Pose> grasppose = worstPose;
                grasppose.setOrigin(worstPose.getOrigin() + btVector3(0,0,additional_depth));
                otherarm->universal_move_toolframe_ik_pose(grasppose);
                othergripper->closeCompliant();
                othergripper->close();
                ROS_INFO("Amount open %f", othergripper->getAmountOpen());
                amountopen = othergripper->getAmountOpen();
                additional_depth -= 0.025;
            }
            otherarm->stabilize_grip();

            tf::Stamped<tf::Pose> highleave = bestPose;
            highleave.setOrigin(bestPose.getOrigin() + btVector3(0,0,0.05));
            ros::Duration(0.5).sleep();
            gripper->open();
            arm->universal_move_toolframe_ik_pose(highleave);

            boost::thread t1(&Gripper::close, gripper, 0);

            currside ? OperateHandleController::plateAttackPoseLeft() : OperateHandleController::plateAttackPoseRight();

            t1.join();
            closingCalibrated[currside] = true;
            minClosing[currside] = gripper->getAmountOpen();

            currside = currside ? 0 : 1;

        }

    }


    if (atoi(argv[1]) == -307)
    {

        int currside = atoi(argv[2]);

        while (ros::ok())
        {

            RobotHead::getInstance()->lookAt("/base_link",1,0,0.5);

            RobotArm *arm = RobotArm::getInstance(currside);

            tf::Stamped<tf::Pose> bowlPose = OperateHandleController::getBowlPose();
            bowlPose = arm->getPoseIn("/base_link", bowlPose);
            pubPose(bowlPose);
            printPose("Bowl Pose vision", bowlPose);

            tf::Stamped<tf::Pose> bowlInBase = arm->getPoseIn("/base_link", bowlPose);
            //bowlInBase.setOrigin(btVector3(0,0,0));

            tf::Stamped<tf::Pose> bowlCorrected = bowlPose;
            bowlCorrected.setRotation(bowlInBase.getRotation().inverse() * bowlPose.getRotation());

            pubPose(bowlCorrected);
            pubPose(bowlCorrected);
            pubPose(bowlCorrected);
            printPose("Bowl Pose corrected", bowlCorrected);

            tf::Stamped<tf::Pose> newToolPose = arm->getToolPose("/base_link");
            printPose("Tool Pose", newToolPose);
            btQuaternion cor = bowlInBase.getRotation().inverse();

            printPose("Bowl Pose", bowlInBase);
            tf::Stamped<tf::Pose> act = arm->rotateAroundPose(newToolPose, bowlInBase, cor);
            printPose("act Pose", act);

            newToolPose.setRotation(bowlInBase.getRotation().inverse() * newToolPose.getRotation());

            printPose("nt Pose (rot)", newToolPose);

            arm->universal_move_toolframe_ik_pose(act);

            exit(0);

        }
    }

    if (atoi(argv[1]) == -310)
    {
        releaseWhenPulled(atoi(argv[2]));
    }


    if (atoi(argv[1]) == -600)
    {

        int idx = 0;
        if (argc > 1)
            idx = atoi(argv[2]);

        ROS_INFO("INDEX TO DEMO :%i", idx);

        //int handle = 0;

        switch (idx)
        {
        case 0:

        case 2:

            DemoScripts::takeBottleFromFridge();

        case 3:

            DemoScripts::serveBottle();

        case 4:

            DemoScripts::openDrawer();

        case 5:

            DemoScripts::takePlate();

        case 6:
            {
            RobotHead::getInstance()->lookAtThreaded("/map", -1.85, 2.1, 1);
            DemoScripts::servePlateToIsland();
            }

        case 7:

            DemoScripts::takeSilverware();

        case 8:
            {
            RobotHead::getInstance()->lookAtThreaded("/map", -1.85, 2.1, 1);
            DemoScripts::servePlateToIsland();
            }

        }
    }




}



