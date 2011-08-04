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

#include <ias_drawer_executive/OperateHandleController.h>
#include <ias_drawer_executive/RobotArm.h>
#include <ias_drawer_executive/Gripper.h>
#include <ias_drawer_executive/Perception3d.h>
#include <ias_drawer_executive/Pressure.h>
#include <ias_drawer_executive/Poses.h>
#include <ias_drawer_executive/AverageTF.h>
#include <ias_drawer_executive/Torso.h>
#include <ias_drawer_executive/Head.h>
#include <ias_drawer_executive/Approach.h>

#include <cop_client_cpp/cop_client.h>
#include <geometry_msgs/PoseStamped.h>

#include <boost/thread.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"



tf::Stamped<tf::Pose> OperateHandleController::getBowlPose()
{
    ros::NodeHandle nh;
    CopClient cop(nh);
    ros::Rate r(100);

    ROS_INFO("GET PLATE POSE");

    unsigned long id_searchspace = cop.LONameQuery("/openni_rgb_optical_frame");
    long vision_primitive =  cop.CallCop("Bowl2", id_searchspace);

    size_t num_results_expected = 1;

    std::vector<vision_msgs::cop_answer> results;
    bool found = false;
    while (nh.ok() && !found)
    {
        if (cop.HasResult(vision_primitive) >= num_results_expected)
            found = true;
        ros::spinOnce();
        r.sleep();
    }

    tf::Stamped<tf::Pose> bowlPose;

    results = cop.GetResult(vision_primitive);
    //    if (results[0].error.length() > 0)
    //       return btVector3(0,0,0);
    float vec[] = {0,0,0};
    if (results[0].error.length() == 0)
    {
        unsigned long frame = cop.LOFrameQuery(results[0].found_poses[0].position,1);

        //cop.LOPointQuery(frame,vec);
        bowlPose = cop.LOPoseQuery(frame);
    }

    btTransform adj;
    adj.setOrigin(btVector3(-0.022,-0.022,0));

    btTransform bl_;
    bl_.setRotation(bowlPose.getRotation());
    bl_.setOrigin(bowlPose.getOrigin());

    bl_ = bl_ * adj;
    bowlPose.setOrigin(bl_.getOrigin());



    //ROS_INFO("PLATE POSE IN MAP %f %f %f", vec[0],vec[1],vec[2]);

    //tf::Stamped<tf::Pose> pMb;
    //pMb.setOrigin(btVector3(vec[0],vec[1],vec[2]));
    //pMb.frame_id_ = "map";


    //pMb = RobotArm::getInstance(0)->getPoseIn("base_link",pMb);
    //ROS_INFO("PLATE POSE IN BASE_LINK %f %f %f", pMb.getOrigin().x(), pMb.getOrigin().y(), pMb.getOrigin().z());

    //btVector3 aPose = pMb.getOrigin();
    //ROS_INFO("aPose %f %f %f ", pMb.getOrigin().x(), pMb.getOrigin().y(),pMb.getOrigin().z());

    //return btVector3(pMb.getOrigin().x(), pMb.getOrigin().y(), pMb.getOrigin().z());

    return bowlPose;
}


btVector3  OperateHandleController::getPlatePose()
{
    ros::NodeHandle nh;
    CopClient cop(nh);
    ros::Rate r(100);

    ROS_INFO("GET PLATE POSE");

    unsigned long id_searchspace = cop.LONameQuery("/narrow_stereo_optical_frame");
    long vision_primitive =  cop.CallCop("RoundPlate", id_searchspace);

    size_t num_results_expected = 1;

    std::vector<vision_msgs::cop_answer> results;
    bool found = false;
    while (nh.ok() && !found)
    {
        if (cop.HasResult(vision_primitive) >= num_results_expected)
            found = true;
        ros::spinOnce();
        r.sleep();
    }

    results = cop.GetResult(vision_primitive);
    //    if (results[0].error.length() > 0)
    //       return btVector3(0,0,0);
    float vec[] = {0,0,0};
    if (results[0].error.length() == 0)
    {
        unsigned long frame = cop.LOFrameQuery(results[0].found_poses[0].position,1);

        cop.LOPointQuery(frame,vec);
    }

    ROS_INFO("PLATE POSE IN MAP %f %f %f", vec[0],vec[1],vec[2]);

    tf::Stamped<tf::Pose> pMb;
    pMb.setOrigin(btVector3(vec[0],vec[1],vec[2]));
    pMb.frame_id_ = "map";
    //pMb = RobotArm::getInstance(0)->getPoseIn("base_link",pMb);
    //ROS_INFO("PLATE POSE IN BASE_LINK %f %f %f", pMb.getOrigin().x(), pMb.getOrigin().y(), pMb.getOrigin().z());
    btVector3 fakePose(0.438186, 1.079624 ,0.759398);
    btVector3 aPose = pMb.getOrigin();
    ROS_INFO("aPose %f %f %f ", pMb.getOrigin().x(), pMb.getOrigin().y(),pMb.getOrigin().z());
    ROS_INFO("fakePose %f %f %f ", fakePose.x(), fakePose.y(),fakePose.z());
    ROS_INFO("DIST %f", (fakePose - aPose).length() );
    if ((fakePose - aPose).length() > 0.5)
    {
        ROS_ERROR("------------------------------------------------------");
        ROS_ERROR("SOMETHING WENT WRONG WITH GETTING A POSE FOR THE PLATE");
        ROS_ERROR("------------------------------------------------------");
        pMb.setOrigin(fakePose);
    }

    return btVector3(pMb.getOrigin().x(), pMb.getOrigin().y(), pMb.getOrigin().z());
}

btVector3  OperateHandleController::getTabletPose()
{
    ros::NodeHandle nh;
    CopClient cop(nh);
    ros::Rate r(100);

    unsigned long id_searchspace = cop.LONameQuery("/wide_stereo_optical_frame");
    long vision_primitive =  cop.CallCop("tablet", id_searchspace);

    size_t num_results_expected = 1;

    std::vector<vision_msgs::cop_answer> results;
    while (nh.ok())
    {
        if (cop.HasResult(vision_primitive) >= num_results_expected)
            break;
        ros::spinOnce();
        r.sleep();
    }

    results = cop.GetResult(vision_primitive);
    if (results[0].error.length() > 0)
        return btVector3(0,0,0);

    unsigned long frame = cop.LOFrameQuery(results[0].found_poses[0].position,1);
    float vec[] = {0,0,0};
    cop.LOPointQuery(frame,vec);

    ROS_INFO("TABLET POSE IN MAP %f %f %f", vec[0],vec[1],vec[2]);

    tf::Stamped<tf::Pose> pMb;
    pMb.setOrigin(btVector3(vec[0],vec[1],vec[2]));
    pMb.frame_id_ = "map";
    pMb = RobotArm::getInstance(0)->getPoseIn("base_link",pMb);
    ROS_INFO("TABLET POSE IN BASE_LINK %f %f %f", pMb.getOrigin().x(), pMb.getOrigin().y(), pMb.getOrigin().z());

    return btVector3(pMb.getOrigin().x(), pMb.getOrigin().y(), pMb.getOrigin().z());
}


int OperateHandleController::maxHandle = -1;

//static std::vector<std::vector<float> > openingTrajectories;
std::vector<std::vector< tf::Stamped<tf::Pose> * > > OperateHandleController::openingTraj;


#include <stdio.h>
#include <stdlib.h>


tf::Stamped<tf::Pose> OperateHandleController::getHandlePoseFromMarker(int arm_, int pos)
{

    int side_ = arm_;

    Gripper *gripper = Gripper::getInstance(side_);

    gripper->open(0.3);

    RobotArm *arm = RobotArm::getInstance(side_);

    if (arm->isTucked()) arm->startTrajectory(arm->lookAtMarker(Poses::untuckPoseA, Poses::untuckPoseB));

    Pressure::getInstance(side_)->reset();

    if (arm_ == 1)
    {
        RobotArm::getInstance(0)->lookAtMarker(Poses::tuckPoseForLeft, Poses::tuckPoseForLeft);
    }

    // Start the trajectory
    if (pos == 0) arm->startTrajectory(arm->lookAtMarker(Poses::lowPoseA,Poses::lowPoseB));
    if (pos == 1) arm->startTrajectory(arm->lookAtMarker(Poses::midPoseA,Poses::midPoseB));
    if (pos == 2) arm->startTrajectory(arm->lookAtMarker(Poses::highPoseA,Poses::highPoseB));
    if (pos == 3) arm->startTrajectory(arm->lookAtMarker(Poses::milehighPoseA,Poses::milehighPoseB));
    if (pos == 4) arm->startTrajectory(arm->lookAtMarker(Poses::leftHighA,Poses::leftHighB));


    // Wait for trajectory completion
    while (!arm->getState().isDone() && ros::ok())
    {
        usleep(500);
    }

    tf::Stamped<tf::Pose> aM = AverageTF::getMarkerTransform("/4x4_1",20);

    if (pos == 0) arm->startTrajectory(arm->lookAtMarker(Poses::lowPoseB,Poses::lowPoseA));
    if (pos == 1) arm->startTrajectory(arm->lookAtMarker(Poses::midPoseB,Poses::midPoseA));
    if (pos == 2) arm->startTrajectory(arm->lookAtMarker(Poses::highPoseB,Poses::highPoseA));
    if (pos == 3) arm->startTrajectory(arm->lookAtMarker(Poses::milehighPoseB,Poses::milehighPoseA));
    if (pos == 4) arm->startTrajectory(arm->lookAtMarker(Poses::leftHighB,Poses::leftHighA));

    // Wait for trajectory completion
    while (!arm->getState().isDone() && ros::ok())
    {
        usleep(500);
    }

    ROS_INFO("AVERAGED MARKER POSa %f %f %f ROT %f %f %f %f", aM.getOrigin().x(),aM.getOrigin().y(),aM.getOrigin().z(),aM.getRotation().x(),aM.getRotation().y(),aM.getRotation().z(),aM.getRotation().w());

    ROS_INFO("AVERAGED MARKER POSb %f %f %f ROT %f %f %f %f", aM.getOrigin().x(),aM.getOrigin().y(),aM.getOrigin().z(),aM.getRotation().x(),aM.getRotation().y(),aM.getRotation().z(),aM.getRotation().w());

    tf::Stamped<tf::Pose> ret;
    ret.stamp_ = ros::Time::now();
    ret.frame_id_ = aM.frame_id_;
    ret.setOrigin(aM.getOrigin());
    ret.setRotation(aM.getRotation());

    return ret;
}

btTransform scaleTransform(const btTransform &in, double scale)
{
    btTransform ret;
    ret.setRotation(btQuaternion::getIdentity().slerp(in.getRotation(), scale));
    ret.setOrigin(in.getOrigin() * scale);
    return ret;
}

void printTransform(const char title[], btTransform a)
{
    ROS_INFO("%s : %f %f %f  %f %f %f %f", title, a.getOrigin().x(), a.getOrigin().y(), a.getOrigin().z(),
           a.getRotation().x(), a.getRotation().y(), a.getRotation().z(), a.getRotation().w());
}


int OperateHandleController::graspHandle(int arm_, tf::Stamped<tf::Pose> aM)
{
    boost::thread(&RobotHead::lookAt,RobotHead::getInstance(),aM.frame_id_,  aM.getOrigin().x(), aM.getOrigin().y(), aM.getOrigin().z(),false);
    int side_ = arm_;
    RobotArm *arm = RobotArm::getInstance(side_);

    Gripper *gripper = Gripper::getInstance(side_);



    tf::Stamped<tf::Pose> p =  arm->getPoseIn("base_link",aM);
    p.setOrigin(p.getOrigin() + btVector3(-0.05,0,0));
    p =  arm->getPoseIn("map",p);

    gripper->close();

    arm->excludeBaseProjectionFromWorkspace = true;

    arm->universal_move_toolframe_ik_pose(p);

    gripper->open();
    arm->universal_move_toolframe_ik_pose(aM);

    gripper->close();

    return 0;
/*

    gripper->close();

    char fixed_frame[] = "map";

    tf::Stamped<tf::Pose> p;
    p.setOrigin(aM.getOrigin());
    p.setRotation(aM.getRotation());
    p.frame_id_ = "base_link";

    tf::Stamped<tf::Pose> pMa = p;
    pMa.frame_id_ = "base_link";
    pMa.setOrigin(pMa.getOrigin() + btVector3(-.03,0,0));
    pMa = arm->getPoseIn(fixed_frame,pMa);

    tf::Stamped<tf::Pose> pMb = p;
    //pMb.setOrigin(pMb.getOrigin() + btVector3(1,0,0));
    pMb.setOrigin(pMb.getOrigin() + btVector3(.08,0,0));
    pMb.frame_id_ = "base_link";
    pMb = arm->getPoseIn(fixed_frame,pMb);
    btVector3 diff = pMb.getOrigin() - pMa.getOrigin(); // we define a line (pMa, diff) in map frame here that is used for the approach

    p.frame_id_ = "map";
    p.setRotation(pMa.getRotation());

    gripper->updatePressureZero();

    // approach xxx
    ROS_INFO("APPROACH");

    std::vector<int> armv;
    std::vector<tf::Stamped<tf::Pose> > goal;
    btVector3 result;
    armv.push_back(side_);
    goal.push_back(pMa);
    armv.push_back(side_);
    goal.push_back(pMb);

    RobotArm::findBaseMovement(result, armv, goal, true, false);

    Approach *apr = new Approach();
    apr->init(side_,pMa, pMb, Approach::front);

    apr->move_to(-.3);

    gripper->close();

    float distA = (apr->increment(0,0.5));
    if (distA == 0)
    {
        ROS_ERROR("DIDNT TOUCH IN THE FIRST 5 CM OF APPROACH");
        distA = (apr->increment(0.5,1));
    }

    RobotArm::getInstance(arm_)->time_to_target = 0;

    //back up 1 centimeter
    apr->move_to(((distA - .02) / .1));
    gripper->open();
    //go 2.75 cm forward from touch position
    apr->move_to(((distA + .0275) / .1));

    tf::Stamped<tf::Pose> actPose;

    gripper->closeCompliant();

    gripper->close();

    RobotArm::getInstance(arm_)->time_to_target = 0;

    arm->excludeBaseProjectionFromWorkspace = false;*/
}

int OperateHandleController::operateHandle(int arm_, tf::Stamped<tf::Pose> aM, int numretry)
{
    //boost::thread(&RobotHead::lookAt,RobotHead::getInstance(),aM.frame_id_,  aM.getOrigin().x(), aM.getOrigin().y(), aM.getOrigin().z(),false);

    RobotHead::getInstance()->lookAtThreaded("/r_gripper_tool_frame", 0 , 0, 0);

    int side_ = arm_;

    int handle = ++maxHandle;

    std::vector<tf::Stamped<tf::Pose> * > openingTrajAct;
    openingTraj.push_back (openingTrajAct);

    Gripper *gripper = Gripper::getInstance(side_);

    gripper->open();

    RobotArm *arm = RobotArm::getInstance(side_);

    arm->excludeBaseProjectionFromWorkspace = true;

    char fixed_frame[] = "map";

    tf::Stamped<tf::Pose> p;
    p.setOrigin(aM.getOrigin());
    p.setRotation(aM.getRotation());
    p.frame_id_ = "base_link";

    tf::Stamped<tf::Pose> pMa = p;
    pMa.frame_id_ = "base_link";
    pMa.setOrigin(pMa.getOrigin() + btVector3(-.05,0,0));
    pMa = arm->getPoseIn(fixed_frame,pMa);

    tf::Stamped<tf::Pose> pMb = p;
    //pMb.setOrigin(pMb.getOrigin() + btVector3(1,0,0));
    pMb.setOrigin(pMb.getOrigin() + btVector3(.05,0,0));
    pMb.frame_id_ = "base_link";
    pMb = arm->getPoseIn(fixed_frame,pMb);
    btVector3 diff = pMb.getOrigin() - pMa.getOrigin(); // we define a line (pMa, diff) in map frame here that is used for the approach

    p.frame_id_ = "map";
    p.setRotation(pMa.getRotation());

    gripper->updatePressureZero();

    // approach xxx
    ROS_INFO("APPROACH");

    std::vector<int> armv;
    std::vector<tf::Stamped<tf::Pose> > goal;
    btVector3 result;
    armv.push_back(side_);
    goal.push_back(pMa);
    armv.push_back(side_);
    goal.push_back(pMb);

    RobotArm::findBaseMovement(result, armv, goal, true, false);

    Approach *apr = new Approach();
    apr->init(side_,pMa, pMb, Approach::front);

    apr->move_to(-.3);

    gripper->close();

    float distA = (apr->increment(0,0.5));
    if (distA == 0)
    {
        ROS_ERROR("DIDNT TOUCH IN THE FIRST 5 CM OF APPROACH");
        distA = (apr->increment(0.5,1));
    }
//back up 5 centimeter
    apr->move_to(((distA - .05) / .1));
    gripper->open();
    //go 2.75 cm forward from touch position
    apr->move_to(((distA + .0275) / .1));

    tf::Stamped<tf::Pose> actPose;

    gripper->closeCompliant();

    gripper->close();

    arm->stabilize_grip();

    tf::Stamped<tf::Pose> aMp;
    aMp.frame_id_ = "base_link";
    arm->getToolPose(aMp,"base_link");

    tf::Stamped<tf::Pose> desiredPose = aMp;
    desiredPose.setOrigin(btVector3(aMp.getOrigin().x(),aMp.getOrigin().y(),aMp.getOrigin().z()));
    desiredPose = arm->getPoseIn(fixed_frame,desiredPose);

    tf::Stamped<tf::Pose> startPose = aMp;
    startPose.setOrigin(btVector3(aMp.getOrigin().x() + .05,aMp.getOrigin().y(),aMp.getOrigin().z()));
    startPose = arm->getPoseIn(fixed_frame,startPose);

    tf::Stamped<tf::Pose> nextPose = desiredPose;

    double dx, dy, dz;
    double maxK = 18;//8;
    double lastlength = 0;
    //int firstbad = 2;

    double gripOpen = gripper->getAmountOpen();

    bool slippedEarly = false;

    //boost::thread(&RobotHead::lookAt,RobotHead::getInstance(),"/map", actPose.getOrigin().x(),actPose.getOrigin().y(),actPose.getOrigin().z(),false);

    tf::Stamped<tf::Pose> *pushPose = new tf::Stamped<tf::Pose> (startPose);
    openingTraj[handle].push_back(pushPose);
    for (double k = 2; k <= maxK; k++)
    {

        using namespace boost::posix_time;
        time_duration td = milliseconds(5); //00:00:00.001
        boost::this_thread::sleep(td);

        tf::Stamped<tf::Pose> actPose;
        arm->getToolPose(actPose,fixed_frame);
        tf::Stamped<tf::Pose> *pushPose = new tf::Stamped<tf::Pose> (actPose);
        openingTraj[handle].push_back(pushPose);
        ROS_INFO_STREAM("rosrun ias_drawer_executive ias_drawer_executive -3 0 " << actPose.getOrigin().x() << " " << actPose.getOrigin().y() << " " << actPose.getOrigin().z()
                        << " " << actPose.getRotation().x() << " " << actPose.getRotation().y() << " " << actPose.getRotation().z() << " " << actPose.getRotation().w());
        ROS_INFO_STREAM("ACTUAL  POSE k " << k << "   " << actPose.getOrigin().x() << " " << actPose.getOrigin().y() << " " << actPose.getOrigin().z()) ;
        ROS_INFO_STREAM("DESIRED POSE " << desiredPose.getOrigin().x() << " " << desiredPose.getOrigin().y() << " " << desiredPose.getOrigin().z()) ;
        ROS_INFO_STREAM("START   POSE " << startPose.getOrigin().x() << " " << startPose.getOrigin().y() << " " << startPose.getOrigin().z()) ;

        //aim head at hand

        dx = actPose.getOrigin().x() - startPose.getOrigin().x();
        dy = actPose.getOrigin().y() - startPose.getOrigin().y();
        dz = actPose.getOrigin().z() - startPose.getOrigin().z();
        double length = sqrtf(dx * dx + dy * dy + dz * dz);
        ROS_INFO("lengt %f hdiff %f", length ,length - lastlength);
        /*if (fabs(length - lastlength) < 0.02)
        {
            ROS_INFO("no movement any more, get out of loop %f %i", k, firstbad);
            if (k - firstbad > 4)
            {
                ROS_ERROR("no movement any more, get out of loop %f %i", k, firstbad);
                break;
            }
        }
        else
        {
            firstbad = k;
        }*/

        //detect slippage via gripper opening
        ROS_INFO("GRIPPER OPENING %f diff %f ", gripper->getAmountOpen(), gripOpen - gripper->getAmountOpen());
        //if (gripOpen - gripper->getAmountOpen() > 0.005)
        if (gripper->getAmountOpen() < 0.005)
        {
            ROS_ERROR("SLIPPED OFF at opening %f", length);
            if (length < 0.2)
            {
                slippedEarly = true;
                ROS_ERROR("SLIPPED EARLY, RETRY");
            }
            break;
        }
        lastlength = length;

        //calculate next target pose
        ROS_ERROR("D x %f y %f z %f length %f", dx , dy ,dz, length);
        dx *= 0.05 / length;
        dy *= 0.05 / length;
        dz *= 0.05 / length;
        double x = startPose.getOrigin().x() + dx * k;
        double y = startPose.getOrigin().y() + dy * k;
        double z = startPose.getOrigin().z() + dz * k;
        ROS_INFO("D %f %f %f NEXT POSE %f %f %f" ,dx,dy,dz, x,y,z);

        nextPose.setOrigin(btVector3(x,y,z));
        nextPose.setRotation(actPose.getRotation());

        // move to next pose
        if (openingTraj[handle].size() > 2) {
            size_t curr = openingTraj[handle].size() - 1;
            btTransform t_0, t_1, t_2;
            t_0.setOrigin(openingTraj[handle][curr -1]->getOrigin());
            t_0.setRotation(openingTraj[handle][curr -1]->getRotation());
            t_1.setOrigin(openingTraj[handle][curr -0]->getOrigin());
            t_1.setRotation(openingTraj[handle][curr -0]->getRotation());
            t_2 = t_1 * (t_0.inverseTimes(t_1));
            nextPose.setOrigin(t_2.getOrigin());
            nextPose.setRotation(t_2.getRotation());

            tf::Stamped<tf::Pose> start = *openingTraj[handle][1]; // is this the first nice one ?
            tf::Stamped<tf::Pose> last = *openingTraj[handle][curr];
            //btTransform start_t = start.get nextPose.
            btTransform rel = start.inverseTimes(last);

            double length = rel.getOrigin().length();
            ROS_INFO("CURRENT distance travelled : @@@@@@@@@@ %f", length);
            if (length > 0.0001) {
               rel = scaleTransform(rel, (length + 0.05) / length);
               btTransform nxt = start * rel;
               printTransform("relative transform normalized to a 5cm more: ", rel);
               nextPose.setOrigin(nxt.getOrigin());
               nextPose.setRotation(nxt.getRotation());
               printTransform("next pose a 5cm more: ", nextPose);
            }
        }

        btVector3 reply = arm->universal_move_toolframe_ik_pose(nextPose);

        //end of trajectory detection via cartesian error
        tf::Stamped<tf::Pose> actPose1;
        arm->getToolPose(actPose1,fixed_frame);

        btVector3 carterr = arm->cartError();
        ROS_ERROR("cartesian error JNT %f XXX  DIFF %f %f %f DISTANCE %f", arm->time_to_target, carterr.x(),carterr.y(), carterr.y(), carterr.length());
        if (carterr.length() > 0.035)
        {
            ROS_ERROR("SLIPPED OFF: CART ERROR JOINT ");
            break;
        }

        btVector3 carterr1 = (actPose1.getOrigin() - nextPose.getOrigin());
        ROS_ERROR("cartesian error REL %f XXX  DIFF %f %f %f DISTANCE %f", arm->time_to_target, carterr1.x(),carterr1.y(), carterr1.y(), carterr1.length());
        if (carterr1.length() > 0.035)
        {
            ROS_ERROR("SLIPPED OFF: CART ERROR POSE DIFF");
            break;
        }

        arm->stabilize_grip();
    }

    arm->excludeBaseProjectionFromWorkspace = false;

    if (slippedEarly)
    {
        if (numretry < 3)
           return OperateHandleController::operateHandle(arm_, aM, ++numretry);
        else return -1;
    }
    else
    {

        arm->getToolPose(actPose,fixed_frame);
        arm->universal_move_toolframe_ik_pose(actPose);

        gripper->open();

        for (int k = openingTraj[handle].size() - 1; k >= 0 ; k--)
        {
            tf::Stamped<tf::Pose> *actPose = openingTraj[handle][k];
            ROS_INFO_STREAM("rosrun ias_drawer_executive ias_drawer_executive -3 0 " << actPose->getOrigin().x() << " " << actPose->getOrigin().y() << " " << actPose->getOrigin().z()
                            << " " << actPose->getRotation().x() << " " << actPose->getRotation().y() << " " << actPose->getRotation().z() << " " << actPose->getRotation().w());
        }

        return handle;
    }
}

void OperateHandleController::close(int side_c, int handle_)
{
    int handle = handle_;

    RobotArm *arm = RobotArm::getInstance(side_c);
    //Gripper::getInstance(side_c)->close(0);

    ROS_INFO("OperateHandleController: traj size %zu", openingTraj[handle].size());

    for (size_t k = 0; k < openingTraj[handle].size() ; ++k) {
        tf::Stamped<tf::Pose> *actPose = openingTraj[handle][k];
        ROS_INFO("K = %zu", k);
        ROS_INFO_STREAM("rosrun ias_drawer_executive ias_drawer_executive -3 0 " << actPose->getOrigin().x() << " " << actPose->getOrigin().y() << " " << actPose->getOrigin().z()
                        << " " << actPose->getRotation().x() << " " << actPose->getRotation().y() << " " << actPose->getRotation().z() << " " << actPose->getRotation().w());
    }

    for (int k = openingTraj[handle].size() - 1; k >= 1 ; k-=3)
    {
        tf::Stamped<tf::Pose> *actPose = openingTraj[handle][k];
        ROS_INFO_STREAM("rosrun ias_drawer_executive ias_drawer_executive -3 0 " << actPose->getOrigin().x() << " " << actPose->getOrigin().y() << " " << actPose->getOrigin().z()
                        << " " << actPose->getRotation().x() << " " << actPose->getRotation().y() << " " << actPose->getRotation().z() << " " << actPose->getRotation().w());

        boost::thread(&RobotHead::lookAt,RobotHead::getInstance(),"/map", actPose->getOrigin().x(),actPose->getOrigin().y(),actPose->getOrigin().z(), false);
        arm->universal_move_toolframe_ik_pose(*actPose);
    }
    arm->universal_move_toolframe_ik_pose(*openingTraj[handle][0]);


    for (size_t k = 1; k <= openingTraj[handle].size() / 3 ; k+=2)
    {
        tf::Stamped<tf::Pose> *actPose = openingTraj[handle][k];
        arm->universal_move_toolframe_ik_pose(*actPose);
    }
}

void spinner(Approach *apr)
{
    float dist = 0;
    //float pos = 0;
    //RobotArm::getInstance(apr->side_)->time_to_target = 0.5;
    RobotArm::getInstance(apr->side_)->time_to_target = 1.0;
    //dist = apr->increment(0,0.15);
    //dist = apr->increment(0,0.6);
    dist = apr->increment(0,1.0);
    //RobotArm::getInstance(apr->side_)->time_to_target = 2.5;
    //ROS_INFO("DIST = %f ", dist);
    //if (dist == 0) {
        //ROS_ERROR("side %i DIST == 0", apr->side_);
        //dist = apr->increment(.6,1.0);
    //}
}

void OperateHandleController::spinnerL(float x, float y, float z)
{
    //lft->increment(l);
    Lift ll, lr;
    boost::thread a0(&Lift::init,&ll,0);
    boost::thread a1(&Lift::init,&lr,1);
    //lr.init(1);
    a0.join(); a1.join();

    boost::thread a(&Lift::increment, &ll, x, y, z);
    boost::thread b(&Lift::increment, &lr, x, y, z);

    a.join();
    b.join();
}

void OperateHandleController::openGrippers()
{
    Gripper *l = Gripper::getInstance(0);
    Gripper *r = Gripper::getInstance(1);

    boost::thread a(&Gripper::open,l,1);
    boost::thread b(&Gripper::open,r,1);

    a.join();
    b.join();
}

void OperateHandleController::pickPlate(btVector3 plate, float width)
{

    if (plate.z() == 0)
    {
        ROS_ERROR("NO PLATE FOUND!");
        return;
    }

    OperateHandleController::openGrippers();

    RobotArm *rightArm = RobotArm::getInstance(0);
    RobotArm *leftArm = RobotArm::getInstance(1);


    tf::Stamped<tf::Pose> plateCenter;
    plateCenter.frame_id_ = "map";
    plateCenter.stamp_ = ros::Time(0);
    plateCenter.setOrigin(plate + btVector3(0,0,0.035)); // magic numbers galore -> this is adjusted for the cop plate detector with the standard .27m plates
    plateCenter.setOrigin(btVector3(plateCenter.getOrigin().x(),plateCenter.getOrigin().y(),plateCenter.getOrigin().z()));
    tf::Stamped<tf::Pose> plateCenterInBase = leftArm->getPoseIn("base_link", plateCenter);
    plateCenterInBase.setOrigin(plateCenterInBase.getOrigin() + btVector3(0,0.03,0));

    // how far from the edge of the plate should the approach start ?
    float startDistance = 0.05;
    // target point is somewhat inwards from the edge of the plate
    float insideDistance = 0.1;
    //float approachDistance = object ? 0.33 : 0.2; //distnace to center to start approach from

    tf::Stamped<tf::Pose> leftApproach;
    leftApproach.frame_id_ = "base_link";
    leftApproach.setOrigin(plateCenterInBase.getOrigin() + btVector3(0,width/2.0f + startDistance,0));
    leftApproach.stamp_ = ros::Time(0);
    leftApproach.setRotation(btQuaternion(-0.302, 0.626, -0.303, 0.652));
    tf::Stamped<tf::Pose> leftApproachMap = RobotArm::getInstance(0)->getPoseIn("map", leftApproach);
    leftApproach.setOrigin(plateCenterInBase.getOrigin() + btVector3(0.05,width/2.0f + startDistance,0));
    tf::Stamped<tf::Pose> leftApproachMapPad = RobotArm::getInstance(0)->getPoseIn("map", leftApproach);
    leftApproach.setOrigin(plateCenterInBase.getOrigin() + btVector3(0,width/2.0f - insideDistance,0));
    tf::Stamped<tf::Pose> leftApproachMapTarget = RobotArm::getInstance(0)->getPoseIn("map", leftApproach);

    ROS_INFO("START AT %f END AT %f", width/2.0f + startDistance, width/2.0f - insideDistance);

    tf::Stamped<tf::Pose> rightApproach;
    rightApproach.frame_id_ = "base_link";
    rightApproach.stamp_ = ros::Time(0);
    rightApproach.setOrigin(plateCenterInBase.getOrigin() + btVector3(0,-width/2.0f - startDistance,0));
    rightApproach.setRotation(btQuaternion(0.651, 0.295, -0.621, -0.322));
    tf::Stamped<tf::Pose> rightApproachMap = RobotArm::getInstance(0)->getPoseIn("map", rightApproach);
    rightApproach.setOrigin(plateCenterInBase.getOrigin() + btVector3(0.05,-width/2.0f - startDistance,0));
    tf::Stamped<tf::Pose> rightApproachMapPad = RobotArm::getInstance(0)->getPoseIn("map", rightApproach);
    rightApproach.setOrigin(plateCenterInBase.getOrigin() + btVector3(0,-width/2.0f + insideDistance,0));
    tf::Stamped<tf::Pose> rightApproachMapTarget = RobotArm::getInstance(0)->getPoseIn("map", rightApproach);

    ROS_INFO("START AT %f END AT %f", -width/2.0f - startDistance, - width/2.0f + insideDistance);


    tf::Stamped<tf::Pose> rightApproachMapHigh = rightApproachMap;
    rightApproachMapHigh.setOrigin(rightApproachMapHigh.getOrigin() + btVector3(0,0,.20));

    tf::Stamped<tf::Pose> leftApproachMapHigh = leftApproachMap;
    leftApproachMapHigh.setOrigin(leftApproachMapHigh.getOrigin() + btVector3(0,0,.20));


    RobotArm::getInstance(0)->time_to_target = 5;
    RobotArm::getInstance(1)->time_to_target = 5;

    std::vector<int> armv;
    std::vector<tf::Stamped<tf::Pose> > goal;
    btVector3 result;
    armv.push_back(0);
    goal.push_back(rightApproachMapPad);
    armv.push_back(0);
    goal.push_back(rightApproachMap);
    armv.push_back(1);
    goal.push_back(leftApproachMapPad);
    armv.push_back(1);
    goal.push_back(leftApproachMap);
    RobotArm::findBaseMovement(result, armv, goal, true, false);


    ROS_ERROR("LEFT APPROACH MAP");
    leftArm->printPose(leftApproachMap);
    ROS_ERROR("RIGHT APPROACH MAP");
    rightArm->printPose(rightApproachMap);

    ROS_INFO("PLATE CENTER");
    rightArm->printPose(plateCenter);

    if (false)
    {

        leftArm->preset_angle = 1.4;
        rightArm->preset_angle = 1.4;

        leftArm->raise_elbow = true;
        rightArm->raise_elbow = true;
    }

    leftApproachMap.stamp_ = ros::Time::now();
    rightApproachMap.stamp_ = ros::Time::now();

    boost::thread tA(&RobotArm::move_toolframe_ik_pose, leftArm, leftApproachMap);
    boost::thread tB(&RobotArm::move_toolframe_ik_pose, rightArm, rightApproachMap);
    tA.join();
    tB.join();

    Approach apl;
    Approach apr;
    apl.init(0,rightApproachMap,rightApproachMapTarget,Approach::inside);
    apr.init(1,leftApproachMap,leftApproachMapTarget,Approach::inside);
    boost::thread t1(spinner,&apl);
    boost::thread t2(spinner,&apr);

    t1.join();
    t2.join();

    boost::thread t3(&Approach::finish, apl);
    boost::thread t4(&Approach::finish, apr);
    t3.join();
    t4.join();


    if ((Gripper::getInstance(0)->getAmountOpen() < 0.005) || (Gripper::getInstance(1)->getAmountOpen() < 0.005))
    {
        return pickPlate(plate);
    }

    spinnerL(0,0,.2);

    leftArm->preset_angle = -1;
    rightArm->preset_angle = -1;
    OperateHandleController::plateCarryPose();
    leftArm->raise_elbow = false;
    rightArm->raise_elbow = false;
}


void OperateHandleController::singleSidedPick(int side,tf::Stamped<tf::Pose> start, tf::Stamped<tf::Pose> end)
{
    Gripper *grip = Gripper::getInstance(side);
    RobotArm *arm = RobotArm::getInstance(side);

    boost::thread t1(&Gripper::open, grip, 0.09);

    arm->universal_move_toolframe_ik_pose(start);

    t1.join();

    Approach apl;

    apl.init(side,start,end,Approach::inside);
    spinner(&apl);
    apl.finish();

}


void OperateHandleController::getPlate(int object, float zHint)
{

    OperateHandleController::openGrippers();


    pr2_controllers_msgs::JointTrajectoryGoal goalA = RobotArm::getInstance(0)->lookAtMarker(Poses::prepDishR1,Poses::prepDishR1);
    pr2_controllers_msgs::JointTrajectoryGoal goalB = RobotArm::getInstance(1)->lookAtMarker(Poses::prepDishL1,Poses::prepDishL1);
    boost::thread t7(&RobotArm::startTrajectory, RobotArm::getInstance(0), goalA,true);
    boost::thread t8(&RobotArm::startTrajectory, RobotArm::getInstance(1), goalB,true);

    t7.join();
    t8.join();

    btVector3 plate = object ? OperateHandleController::getTabletPose() : OperateHandleController::getPlatePose();
    //btVector3 plate = btVector3(0.502,1.093,.785);

    // account for hard coded / laser detected plate height
    if (zHint > 0.01)
    {
        plate.setZ(zHint);
    }



    pickPlate(plate,0.3);
}


void OperateHandleController::plateCarryPose()
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

    //ROS_INFO("AVERAGE Y pre: %f", averageY);

    RobotArm::getInstance(0)->raise_elbow= true;
    RobotArm::getInstance(1)->raise_elbow= true;

    RobotArm::getInstance(0)->preset_angle = .8;
    RobotArm::getInstance(1)->preset_angle = .8;

    OperateHandleController::spinnerL(.45 - averageX,0 - averageY,.935 - averageZ);

    RobotArm::getInstance(0)->raise_elbow= false;
    RobotArm::getInstance(1)->raise_elbow= false;


}

void OperateHandleController::plateTuckPose()
{
    pr2_controllers_msgs::JointTrajectoryGoal goalAT = RobotArm::getInstance(0)->lookAtMarker(Poses::prepDishRT,Poses::prepDishRT);
    pr2_controllers_msgs::JointTrajectoryGoal goalBT = RobotArm::getInstance(1)->lookAtMarker(Poses::prepDishLT,Poses::prepDishLT);
    boost::thread t2T(&RobotArm::startTrajectory, RobotArm::getInstance(0), goalAT,true);
    boost::thread t3T(&RobotArm::startTrajectory, RobotArm::getInstance(1), goalBT,true);
    t2T.join();
    t3T.join();
}

void OperateHandleController::plateTuckPoseLeft()
{
    pr2_controllers_msgs::JointTrajectoryGoal goalBT = RobotArm::getInstance(1)->lookAtMarker(Poses::prepDishLT,Poses::prepDishLT);
    boost::thread t3T(&RobotArm::startTrajectory, RobotArm::getInstance(1), goalBT,true);
    t3T.join();
}


void OperateHandleController::plateTuckPoseRight()
{
    pr2_controllers_msgs::JointTrajectoryGoal goalAT = RobotArm::getInstance(0)->lookAtMarker(Poses::prepDishRT,Poses::prepDishRT);
    boost::thread t2T(&RobotArm::startTrajectory, RobotArm::getInstance(0), goalAT,true);
    t2T.join();
}


void OperateHandleController::plateAttackPose()
{
    pr2_controllers_msgs::JointTrajectoryGoal goalA = RobotArm::getInstance(0)->lookAtMarker(Poses::prepDishR1,Poses::prepDishR1);
    pr2_controllers_msgs::JointTrajectoryGoal goalB = RobotArm::getInstance(1)->lookAtMarker(Poses::prepDishL1,Poses::prepDishL1);
    boost::thread t2(&RobotArm::startTrajectory, RobotArm::getInstance(0), goalA,true);
    boost::thread t3(&RobotArm::startTrajectory, RobotArm::getInstance(1), goalB,true);
    t2.join();
    t3.join();
}

void OperateHandleController::plateAttackPoseLeft()
{
    pr2_controllers_msgs::JointTrajectoryGoal goalB = RobotArm::getInstance(1)->lookAtMarker(Poses::prepDishL1,Poses::prepDishL1);
    boost::thread t3(&RobotArm::startTrajectory, RobotArm::getInstance(1), goalB,true);
    t3.join();
}

void OperateHandleController::plateAttackPoseRight()
{
    pr2_controllers_msgs::JointTrajectoryGoal goalA = RobotArm::getInstance(0)->lookAtMarker(Poses::prepDishR1,Poses::prepDishR1);
    boost::thread t2(&RobotArm::startTrajectory, RobotArm::getInstance(0), goalA,true);
    t2.join();
}


