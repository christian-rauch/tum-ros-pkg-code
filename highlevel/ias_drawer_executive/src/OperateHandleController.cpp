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
#include <ias_drawer_executive/Pressure.h>
#include <ias_drawer_executive/Poses.h>
#include <ias_drawer_executive/AverageTF.h>
#include <ias_drawer_executive/Torso.h>
#include <ias_drawer_executive/Head.h>
#include <ias_drawer_executive/Approach.h>

#include <cop_client_cpp/cop_client.h>
#include <geometry_msgs/PoseStamped.h>
//int main(int argc, char* argv[])

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

boost::mutex OperateHandleController::handle_mutex;

std::vector<tf::Stamped<tf::Pose> *> OperateHandleController::handlePoses = *(new std::vector<tf::Stamped<tf::Pose> *> ());

void OperateHandleController::handleCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    //make this a boost shared ptr!
    tf::Stamped<tf::Pose> *newPose = new tf::Stamped<tf::Pose>();
    newPose->frame_id_ = msg->header.frame_id;
    newPose->stamp_ = msg->header.stamp;
    newPose->setOrigin(btVector3(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z));
    newPose->setRotation(btQuaternion(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w));
    ROS_INFO("CALLBACK GOT HANDLE AT %f %f %f", msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
    handle_mutex.lock();
    handlePoses.push_back(newPose);
    handle_mutex.unlock();
}

//ostopic echo /stereo_table_cluster_detector/pointcloud_minmax_3d_node/cluster

tf::Stamped<tf::Pose> OperateHandleController::getHandlePoseFromLaser(int pos)
{


    system("rosservice call laser_tilt_controller/set_periodic_cmd '{ command: { header: { stamp: 0 }, profile: \"linear\" , period: 10 , amplitude: 0.8 , offset: 0.3 }}'");

    handlePoses.clear();
    ros::NodeHandle n_;
    ros::Subscriber sub_= n_.subscribe("/handle_detector/pointcloud_line_pose_node/handle_pose", 100, &OperateHandleController::handleCallback);

    size_t numHandles = 0;
    ros::Rate rate(20);
    while (ros::ok() && (numHandles < 10))
    {
        handle_mutex.lock();
        if (OperateHandleController::handlePoses.size() > numHandles)
        {
            ROS_INFO("LAST HANDLE: %f %f %f ", handlePoses[numHandles]->getOrigin().x(),handlePoses[numHandles]->getOrigin().y(),handlePoses[numHandles]->getOrigin().z());
            numHandles = handlePoses.size();
        }
        handle_mutex.unlock();
        rate.sleep();
        ros::spinOnce();
    }

    return *handlePoses[0];

    /*tf::StampedTransform decid;
    decid.frame_id_ = "none";
    decid.stamp_ = ros::Time();
    decid.setOrigin(btVector3(1,0,0));
    decid.setRotation(btQuaternion(0,0,0,1));
    btTransform classify = transform * decid;

    tf::StampedTransform decid2;
    decid2.frame_id_ = "none";
    decid2.stamp_ = ros::Time();
    decid2.setOrigin(btVector3(0,1,0));
    decid2.setRotation(btQuaternion(0,0,0,1));

    btTransform classify2 = transform * decid2;

    ROS_INFO("DECID :c  %f c1 %f m %f", classify.getOrigin().z(), classify2.getOrigin().z(), transform.getOrigin().z());

    if (classify.getOrigin().z() < classify2.getOrigin().z())
    {
        //numh++;
        ROS_INFO("HORIZONTAL %i");//, numh);
    }
    else
    {
        //numv++;
        ROS_INFO("VERTICAL %i");//, numv);
    }*/
}


boost::mutex OperateHandleController::handle_cloud_mutex;
PointCloudT OperateHandleController::lastCloud;

#include <pcl/ros/conversions.h>

void OperateHandleController::handleCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    //make this a boost shared ptr!
    handle_cloud_mutex.lock();
    pcl::fromROSMsg(*msg,lastCloud);
    //lastCloud = msg;
    handle_cloud_mutex.unlock();
}

tf::Stamped<tf::Pose> OperateHandleController::getHandlePoseFromLaser(tf::Stamped<tf::Pose> hint)
{
    system("rosservice call laser_tilt_controller/set_periodic_cmd '{ command: { header: { stamp: 0 }, profile: \"linear\" , period: 10 , amplitude: 0.8 , offset: 0.3 }}'");

    tf::Stamped<tf::Pose> inBase = RobotArm::getInstance(0)->getPoseIn("base_link", hint);

    ros::NodeHandle n_;
    ros::Subscriber subHandleInliers = n_.subscribe("/handle_detector/handle_projected_inliers/output", 10, OperateHandleController::handleCloudCallback);

    bool found = false;
    ros::Rate rate(5);

    int maxNum = 30 * 5;

    btVector3 best(0,0,0);

    while ((!found) && ((--maxNum) > 0))
    {
        rate.sleep();
        ros::spinOnce();
        handle_cloud_mutex.lock();
        float shortest = 10;
        for (size_t k = 0; k < lastCloud.points.size(); k++)
        {
            btVector3 act(lastCloud.points[k].x,lastCloud.points[k].y,lastCloud.points[k].z);
            float dist = btVector3(act - inBase.getOrigin()).length();
            if (dist < shortest)
            {
                shortest = dist;
                best = act;
            }
        }
        handle_cloud_mutex.unlock();
        if (shortest < 0.2)
            found = true;
    }

    tf::Stamped<tf::Pose> ret;
    ret.frame_id_ = "base_link";
    ret.setOrigin(best);

    if (maxNum <= 0)
        ret.setOrigin(btVector3(0,0,-1));

    return ret;
}


void OperateHandleController::bottleCallback(const ias_table_msgs::TableCluster::ConstPtr& msg)
{
    //make this a boost shared ptr!
    tf::Stamped<tf::Pose> *newPose = new tf::Stamped<tf::Pose>();
    newPose->frame_id_ = msg->header.frame_id;
    newPose->stamp_ = msg->header.stamp;
    newPose->setOrigin(btVector3(msg->center.x,msg->center.y,msg->center.z));
    //newPose->setRotation(btQuaternion(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w));
    ROS_INFO("CALLBACK GOT HANDLE AT %f %f %f", msg->center.x,msg->center.y,msg->center.z);
    handle_mutex.lock();
    handlePoses.push_back(newPose);
    handle_mutex.unlock();
}

//ostopic echo /stereo_table_cluster_detector/pointcloud_minmax_3d_node/cluster

#include <stdio.h>
#include <stdlib.h>

tf::Stamped<tf::Pose> OperateHandleController::getBottlePose()
{

    //system("rosrun dynamic_reconfigure dynparam set  /camera_synchronizer_node projector_mode 3");
    //system("rosrun dynamic_reconfigure dynparam set  /camera_synchronizer_node narrow_stereo_trig_mode 3");
    system("rosrun dynamic_reconfigure dynparam set  /camera_synchronizer_node '{projector_mode: 3, narrow_stereo_trig_mode: 3}'");
    //handlePoses.clear();

    //tf::Stamped<tf::Pose> *newPose = new tf::Stamped<tf::Pose>();
    //handlePoses.push_back(newPose);

    ros::NodeHandle n_;
    ros::Subscriber sub_= n_.subscribe("/stereo_table_cluster_detector/pointcloud_minmax_3d_node/cluster", 100, &OperateHandleController::bottleCallback);

    size_t numHandles = 0;
    ros::Rate rate(20);

    btVector3 average(0,0,0);
    float numav = 0;

    tf::Stamped<tf::Pose> ret;

    while (ros::ok() && (numHandles < 5))
    {
        handle_mutex.lock();
        if (OperateHandleController::handlePoses.size() > numHandles)
        {
            ROS_INFO("LAST HANDLE: %f %f %f ", handlePoses[numHandles]->getOrigin().x(),handlePoses[numHandles]->getOrigin().y(),handlePoses[numHandles]->getOrigin().z());
            average += btVector3(handlePoses[numHandles]->getOrigin().x(),handlePoses[numHandles]->getOrigin().y(),handlePoses[numHandles]->getOrigin().z());
            ret.stamp_ = handlePoses[numHandles]->stamp_;
            ret.frame_id_ = handlePoses[numHandles]->frame_id_;
            numav++;
            numHandles = handlePoses.size();
        }

        handle_mutex.unlock();
        rate.sleep();
        ros::spinOnce();
    }

    average *= 1.0f / numav;
    ret.setOrigin(average);

    ret = RobotArm::getPoseIn("map", ret);

    system("rosrun dynamic_reconfigure dynparam set  /camera_synchronizer_node projector_mode 1");

    return ret;
}


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

    //a.join();

    //aM.setOrigin(btVector3(0.629, -0.366, 0.723));
    //aM.setRotation(btQuaternion(-0.741, -0.033, 0.017, 0.670));

    tf::StampedTransform aM = AverageTF::getMarkerTransform("/4x4_1",20);

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

    //play safe and dont actually touch
    // aM.setOrigin(aM.getOrigin() - btVector3(.1,0,0));

    ROS_INFO("AVERAGED MARKER POSb %f %f %f ROT %f %f %f %f", aM.getOrigin().x(),aM.getOrigin().y(),aM.getOrigin().z(),aM.getRotation().x(),aM.getRotation().y(),aM.getRotation().z(),aM.getRotation().w());

    //double gripHorizontal = true;

    //btVector3 euler;
    //QuaternionToEuler(aM.getRotation().normalize(), euler);

    //ROS_INFO("EULER %f %f %f", euler.x(),euler.y(),euler.z());

    //ROS_INFO("DISTANCE TO HORIZ %f DISTANCE TO VERT %f", distToHoriz, distToVert);
    // 0.532024 -0.471480 0.928748 ROT -0.003390 -0.645296 0.010487 0.763853
    //if (distToVert < distToHoriz)
    //  gripHorizontal = false;



    // rotate marker position for gripper compatibility, when we look at the front of the marker, we want to grip it from the front also
    //tf::StampedTransform rot;
    //rot.setRotation(btQuaternion(0,0,-M_PI/2));

    //btQuaternion myq(aM.getRotation().x(),aM.getRotation().y(),aM.getRotation().z(),aM.getRotation().w());
    //btQuaternion qy = myq * btQuaternion(0,M_PI / 2,0);;
    //ROS_INFO("ROT Y %f %f %f %f", qy.x(),qy.y(),qy.z(),qy.w());

    //aM.setRotation(btQuaternion(gripOrientation[0],gripOrientation[1],gripOrientation[2],gripOrientation[3]));
    //aM.setOrigin(aM.getOrigin() - btVector3(.1,0,0));

    //btQuaternion grip(aM.getRotation());
    //grip *= btQuaternion(btVector3(0,-1,0), - M_PI / 2.0f);

    tf::Stamped<tf::Pose> ret;
    ret.stamp_ = ros::Time::now();
    ret.frame_id_ = aM.frame_id_;
    ret.setOrigin(aM.getOrigin());
    ret.setRotation(aM.getRotation());

    return ret;
}


//int OperateHandleController::operateHandle(int arm_, int pos)
int OperateHandleController::operateHandle(int arm_, tf::Stamped<tf::Pose> aM)
{


    boost::thread(&RobotHead::lookAt,RobotHead::getInstance(),"/map",  aM.getOrigin().x(), aM.getOrigin().y(), aM.getOrigin().z());

    int side_ = arm_;

    int handle = ++maxHandle;

    std::vector<tf::Stamped<tf::Pose> * > openingTrajAct;
    openingTraj.push_back (openingTrajAct);

    //Torso *torso = Torso::getInstance();

    //boost::thread a(&Torso::down, torso);

    Gripper *gripper = Gripper::getInstance(side_);

    //gripper->open(0.3);
    gripper->open();

    RobotArm *arm = RobotArm::getInstance(side_);

    double gripOrientation[4];

    gripOrientation[0] = aM.getRotation().x();
    gripOrientation[1] = aM.getRotation().y();
    gripOrientation[2] = aM.getRotation().z();
    gripOrientation[3] = aM.getRotation().w();

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

    float distA = (apr->increment(0.5));
    if (distA == 0)
    {
        ROS_ERROR("DIDNT TOUCH IN THE FIRST 5 CM OF APPROACH");
        distA = (apr->increment(1));
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
    //aMp.setRotation(aM.getRotation());
    //aMp.setOrigin(aM.getOrigin());
    arm->getToolPose(aMp,"base_link");

    tf::Stamped<tf::Pose> desiredPose = aMp;
    desiredPose.setOrigin(btVector3(aMp.getOrigin().x(),aMp.getOrigin().y(),aMp.getOrigin().z()));
    desiredPose = arm->getPoseIn(fixed_frame,desiredPose);

    tf::Stamped<tf::Pose> startPose = aMp;
    startPose.setOrigin(btVector3(aMp.getOrigin().x() + .05,aMp.getOrigin().y(),aMp.getOrigin().z()));
    startPose = arm->getPoseIn(fixed_frame,startPose);

    tf::Stamped<tf::Pose> nextPose = desiredPose;

//  arm->move_ik(aM.getOrigin().x() - .35,aM.getOrigin().y(),aM.getOrigin().z(), -0.711, -0.008, 0.005, 0.703);
    //arm->universal_move_toolframe_ik(desiredPose);
    double dx, dy, dz;
    double maxK = 18;//8;
    double lastlength = 0;
    int firstbad = 2;

    double gripOpen = gripper->getAmountOpen();

    bool slippedEarly = false;

    tf::Stamped<tf::Pose> *pushPose = new tf::Stamped<tf::Pose> (startPose);
    openingTraj[handle].push_back(pushPose);
    for (double k = 2; k <= maxK; k++)
    {
        tf::Stamped<tf::Pose> actPose;
        arm->getToolPose(actPose,fixed_frame);

        boost::thread(&RobotHead::lookAt,RobotHead::getInstance(),"/map", actPose.getOrigin().x(),actPose.getOrigin().y(),actPose.getOrigin().z());

        tf::Stamped<tf::Pose> *pushPose = new tf::Stamped<tf::Pose> (actPose);
        openingTraj[handle].push_back(pushPose);
        ROS_INFO_STREAM("rosrun ias_drawer_executive ias_drawer_executive -3 0 " << actPose.getOrigin().x() << " " << actPose.getOrigin().y() << " " << actPose.getOrigin().z()
                        << " " << actPose.getRotation().x() << " " << actPose.getRotation().y() << " " << actPose.getRotation().z() << " " << actPose.getRotation().w());
        ROS_INFO_STREAM("ACTUAL  POSE k " << k << "   " << actPose.getOrigin().x() << " " << actPose.getOrigin().y() << " " << actPose.getOrigin().z()) ;
        ROS_INFO_STREAM("DESIRED POSE " << desiredPose.getOrigin().x() << " " << desiredPose.getOrigin().y() << " " << desiredPose.getOrigin().z()) ;
        ROS_INFO_STREAM("START   POSE " << startPose.getOrigin().x() << " " << startPose.getOrigin().y() << " " << startPose.getOrigin().z()) ;
        dx = actPose.getOrigin().x() - startPose.getOrigin().x();
        dy = actPose.getOrigin().y() - startPose.getOrigin().y();
        dz = actPose.getOrigin().z() - startPose.getOrigin().z();
        double length = sqrtf(dx * dx + dy * dy + dz * dz);
        ROS_INFO("lengt %f hdiff %f", length ,length - lastlength);
        if (fabs(length - lastlength) < 0.02)
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
        }
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
        ROS_ERROR("D x %f y %f z %f length %f", dx , dy ,dz, length);
        dx *= 0.05 / length;
        dy *= 0.05 / length;
        dz *= 0.05 / length;
        double x = startPose.getOrigin().x() + dx * k;
        double y = startPose.getOrigin().y() + dy * k;
        double z = startPose.getOrigin().z() + dz * k;
        ROS_INFO("D %f %f %f NEXT POSE %f %f %f" ,dx,dy,dz, x,y,z);
        //if (success)
        // success = universal_move_toolframe_ik(x,y,z, gripOrientation[0],gripOrientation[1],gripOrientation[2],gripOrientation[3]);
        //if (success)
        // lastK = k;
        //btVector3 reply = universal_move_toolframe_ik(x,y,z, gripOrientation[0],gripOrientation[1],gripOrientation[2],gripOrientation[3]);
        //mx += reply.x();
        //my += reply.y();
        nextPose.setOrigin(btVector3(x,y,z));
        nextPose.setRotation(actPose.getRotation());
        btVector3 reply = arm->universal_move_toolframe_ik_pose(nextPose);

        arm->stabilize_grip();
    }

    if (slippedEarly)
    {
        return OperateHandleController::operateHandle(arm_, aM);
    }
    else
    {

        //tf::Stamped<tf::Pose> actPose;
        arm->getToolPose(actPose,fixed_frame);
        arm->universal_move_toolframe_ik_pose(actPose);

        gripper->open();

        for (int k = openingTraj[handle].size() - 1; k >= 0 ; k--)
        {
            tf::Stamped<tf::Pose> *actPose = openingTraj[handle][k];
            ROS_INFO_STREAM("rosrun ias_drawer_executive ias_drawer_executive -3 0 " << actPose->getOrigin().x() << " " << actPose->getOrigin().y() << " " << actPose->getOrigin().z()
                            << " " << actPose->getRotation().x() << " " << actPose->getRotation().y() << " " << actPose->getRotation().z() << " " << actPose->getRotation().w());
            //arm->universal_move_toolframe_ik(actPose);
        }

        /*
          for (int k = 0; k <= openingTraj[handle].size() / 2 ; k++) {
            tf::Stamped<tf::Pose> actPose = openingTraj[handle][k];
            arm->universal_move_toolframe_ik(actPose);
          }
          */

        return handle;

    }


    //tf::Stamped<tf::Pose> actPose;
    //arm->getToolPose(actPose,fixed_frame);

    //universal_move_toolframe_ik(startPose);

    //universal_move_toolframe_ik(actPose);
}

void OperateHandleController::close(int handle_)
{
    int handle = handle_;

    RobotArm *arm = RobotArm::getInstance(0);

    for (int k = openingTraj[handle].size() - 1; k >= 1 ; k-=3)
    {
        tf::Stamped<tf::Pose> *actPose = openingTraj[handle][k];
        ROS_INFO_STREAM("rosrun ias_drawer_executive ias_drawer_executive -3 0 " << actPose->getOrigin().x() << " " << actPose->getOrigin().y() << " " << actPose->getOrigin().z()
                        << " " << actPose->getRotation().x() << " " << actPose->getRotation().y() << " " << actPose->getRotation().z() << " " << actPose->getRotation().w());

        boost::thread(&RobotHead::lookAt,RobotHead::getInstance(),"/map", actPose->getOrigin().x(),actPose->getOrigin().y(),actPose->getOrigin().z());
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
    RobotArm::getInstance(apr->side_)->time_to_target = 0.5;
    dist = apr->increment(0.15);
    RobotArm::getInstance(apr->side_)->time_to_target = 2.5;
    if (dist == 0)
        dist = apr->increment(1);

    //for (pos = 0 ; (pos <= 1) && (dist==0); pos+= 0.2)
    //{
    //dist = apr->increment(pos);
    //}
    //apr->increment(1);
    //apr->increment(1);
}

void OperateHandleController::spinnerL(float x, float y, float z)
{
    //lft->increment(l);
    Lift ll, lr;
    ll.init(0);
    lr.init(1);

    //boost::thread a(&Lift::increment, &ll, l, back);
    //boost::thread b(&Lift::increment, &lr, l, back);

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

void OperateHandleController::pickPlate(btVector3 plate)
{

    if (plate.z() == 0)
    {
        ROS_ERROR("NO PLATE FOUND!");
        return;
    }

    //Gripper::getInstance(0)->open();
    //Gripper::getInstance(1)->open();
    OperateHandleController::openGrippers();

    RobotArm *rightArm = RobotArm::getInstance(0);
    RobotArm *leftArm = RobotArm::getInstance(1);


    tf::Stamped<tf::Pose> plateCenter;
    plateCenter.frame_id_ = "map";
    plateCenter.stamp_ = ros::Time(0);
    plateCenter.setOrigin(plate + btVector3(0,0,0.035));
    plateCenter.setOrigin(btVector3(plateCenter.getOrigin().x(),plateCenter.getOrigin().y(),plateCenter.getOrigin().z()));
    tf::Stamped<tf::Pose> plateCenterInBase = leftArm->getPoseIn("base_link", plateCenter);


    //float approachDistance = object ? 0.33 : 0.2; //distnace to center to start approach from

    tf::Stamped<tf::Pose> leftApproach;
    leftApproach.frame_id_ = "base_link";
    leftApproach.setOrigin(plateCenterInBase.getOrigin() + btVector3(0,0.17,0));
    leftApproach.stamp_ = ros::Time(0);
    leftApproach.setRotation(btQuaternion(-0.302, 0.626, -0.303, 0.652));
    tf::Stamped<tf::Pose> leftApproachMap = RobotArm::getInstance(0)->getPoseIn("map", leftApproach);
    leftApproach.setOrigin(plateCenterInBase.getOrigin() + btVector3(0.05,0.15,0));
    tf::Stamped<tf::Pose> leftApproachMapPad = RobotArm::getInstance(0)->getPoseIn("map", leftApproach);

    tf::Stamped<tf::Pose> rightApproach;
    rightApproach.frame_id_ = "base_link";
    rightApproach.stamp_ = ros::Time(0);
    rightApproach.setOrigin(plateCenterInBase.getOrigin() + btVector3(0,-0.17,0));
    rightApproach.setRotation(btQuaternion(0.651, 0.295, -0.621, -0.322));
    tf::Stamped<tf::Pose> rightApproachMap = RobotArm::getInstance(0)->getPoseIn("map", rightApproach);
    rightApproach.setOrigin(plateCenterInBase.getOrigin() + btVector3(0.05,-0.15,0));
    tf::Stamped<tf::Pose> rightApproachMapPad = RobotArm::getInstance(0)->getPoseIn("map", rightApproach);


    RobotArm::getInstance(0)->time_to_target = 5;
    RobotArm::getInstance(1)->time_to_target = 5;

    //RobotArm::getInstance(0)->universal_move_toolframe_ik(rightApproachMap);
    //RobotArm::getInstance(1)->universal_move_toolframe_ik(leftApproachMap);
    //RobotArm::getInstance(0)->universal_move_toolframe_ik(rightApproachMap);

    //XXXX
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


    ROS_INFO("LEFT APPROACH MAP");
    leftArm->printPose(leftApproachMap);
    ROS_INFO("RIGHT APPROACH MAP");
    rightArm->printPose(rightApproachMap);

    //RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(rightApproachMap);
    //RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(leftApproachMap);

    leftApproachMap.stamp_ = ros::Time::now();
    rightApproachMap.stamp_ = ros::Time::now();

    boost::thread tA(&RobotArm::move_toolframe_ik_pose, leftArm, leftApproachMap);
    boost::thread tB(&RobotArm::move_toolframe_ik_pose, rightArm, rightApproachMap);
    tA.join();
    tB.join();

    //rightApproach.stamp_ = ros::Time::now() ;
    //plateCenter.stamp_ = ros::Time::now();
    //leftApproach.stamp_ = ros::Time::now();

    Approach apl;
    Approach apr;
    apl.init(0,rightApproachMap,plateCenter,Approach::inside);
    apr.init(1,leftApproachMap,plateCenter,Approach::inside);
    boost::thread t1(spinner,&apl);
    boost::thread t2(spinner,&apr);

    t1.join();
    t2.join();

    boost::thread t3(&Approach::finish, apl);
    boost::thread t4(&Approach::finish, apr);
    //apl.finish();
    //apr.finish();
    t3.join();
    t4.join();

    if ((Gripper::getInstance(0)->getAmountOpen() < 0.001) || (Gripper::getInstance(1)->getAmountOpen() < 0.001))
    {
        return pickPlate(plate);
    }

    spinnerL(0,0,.2);

    //spinnerL(0,0,.1);


    OperateHandleController::plateCarryPose();
}

void OperateHandleController::getPlate(int object, float zHint)
//int side, tf::Stamped<tf::Pose> plateCenter)
{

    /*Translation: [0.725, 0.269, 0.843]
    - Rotation: in Quaternion [-0.633, 0.296, 0.644, -0.312]
    plate 0.698948 0.046852 0.816873
    */

    OperateHandleController::openGrippers();


    pr2_controllers_msgs::JointTrajectoryGoal goalA = RobotArm::getInstance(0)->lookAtMarker(Poses::prepDishR1,Poses::prepDishR1);
    pr2_controllers_msgs::JointTrajectoryGoal goalB = RobotArm::getInstance(1)->lookAtMarker(Poses::prepDishL1,Poses::prepDishL1);
    boost::thread t7(&RobotArm::startTrajectory, RobotArm::getInstance(0), goalA);
    boost::thread t8(&RobotArm::startTrajectory, RobotArm::getInstance(1), goalB);

    t7.join();
    t8.join();

    btVector3 plate = object ? OperateHandleController::getTabletPose() : OperateHandleController::getPlatePose();

    // HARD CORDED
    if (zHint > 0.01)
    {
        plate.setZ(zHint);
    }

    pickPlate(plate);
    //spinnerL(-.1);

    //openGrippers();
// for (float ap = 1; ap > 0; ap-=0.025) {
//     apl.increment(ap,false);
//     apr.increment(ap,false);
// }
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

    OperateHandleController::spinnerL(.45 - averageX,0 - averageY,.935 - averageZ);

}

void OperateHandleController::plateTuckPose()
{
    pr2_controllers_msgs::JointTrajectoryGoal goalAT = RobotArm::getInstance(0)->lookAtMarker(Poses::prepDishRT,Poses::prepDishRT);
    pr2_controllers_msgs::JointTrajectoryGoal goalBT = RobotArm::getInstance(1)->lookAtMarker(Poses::prepDishLT,Poses::prepDishLT);
    boost::thread t2T(&RobotArm::startTrajectory, RobotArm::getInstance(0), goalAT);
    boost::thread t3T(&RobotArm::startTrajectory, RobotArm::getInstance(1), goalBT);
    t2T.join();
    t3T.join();
}

void OperateHandleController::plateAttackPose()
{
    pr2_controllers_msgs::JointTrajectoryGoal goalA = RobotArm::getInstance(0)->lookAtMarker(Poses::prepDishR1,Poses::prepDishR1);
    pr2_controllers_msgs::JointTrajectoryGoal goalB = RobotArm::getInstance(1)->lookAtMarker(Poses::prepDishL1,Poses::prepDishL1);
    boost::thread t2(&RobotArm::startTrajectory, RobotArm::getInstance(0), goalA);
    boost::thread t3(&RobotArm::startTrajectory, RobotArm::getInstance(1), goalB);
    t2.join();
    t3.join();
}


/*
bin/ias_drawer_executive -2 0 0.65, -0.12, 0.846 0.651, 0.295, -0.621, -0.322 ; bin/ias_drawer_executive -2 1 0.671, 0.223, 0.830 -0.302, 0.626, -0.303, 0.652

rosservice call /r_reactive_grasp/compliant_close
approach start
[ INFO] [1285523953.977658083]: in map
[ INFO] [1285523953.977821783]: bin/ias_drawer_executive -2 arm 0.844287 1.009164 0.897106 0.638829 0.320449 -0.633328 -0.296825

[ INFO] [1285523953.977885195]: in base_link
[ INFO] [1285523953.977937993]: bin/ias_drawer_executive -2 arm 0.650000 -0.120000 0.846000 0.650951 0.294978 -0.620953 -0.321976

PLATE CENTER
[ INFO] [1285523950.184531670]: in map
[ INFO] [1285523950.184845102]: bin/ias_drawer_executive -2 arm 0.827575 1.225973 0.896841 0.638747 0.320219 -0.632991 -0.297968

[ INFO] [1285523950.184929745]: in base_link
[ INFO] [1285523950.185011874]: bin/ias_drawer_executive -2 arm 0.650000 0.100000 0.846000 0.650951 0.294978 -0.620953 -0.321976

[1.249, -0.562, 0.948]
- Rotation: in Quaternion [0.004, 0.002, 0.021, 1.000]

evil fridge
bin/ias_drawer_executive -2 arm 1.265000 -0.655000 0.951000 0.005001 -0.053009 -0.029005 0.998160

bin/ias_drawer_executive -3 1 1.165, -0.655, 1.151 0.005, -0.053, -0.029, 0.998

bin/ias_drawer_executive -3 1 0.965, -0.655, 1.251 0.005, -0.053, -0.029, 0.998

bin/ias_drawer_executive -3 1 0.965, -0.255, 1.251 0.005, -0.053, -0.029, 0.998

*/
