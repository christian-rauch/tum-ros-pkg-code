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

#include <ias_drawer_executive/Perception3d.h>
#include <ias_drawer_executive/RobotArm.h>
#include <pcl/ros/conversions.h>

boost::mutex Perception3d::handle_mutex;
boost::mutex Perception3d::plane_mutex;

std::vector<tf::Stamped<tf::Pose> *> Perception3d::handlePoses = *(new std::vector<tf::Stamped<tf::Pose> *> ());
std::vector<tf::Stamped<tf::Pose> *> Perception3d::planePoses = *(new std::vector<tf::Stamped<tf::Pose> *> ());

boost::mutex Perception3d::handle_cloud_mutex;
PointCloudT Perception3d::lastCloud;
int op_handle_cloud_cnt = 0;

btVector3 Perception3d::handleHint;
btVector3 Perception3d::handleResult;
float Perception3d::handleMinDist;
ros::Time Perception3d::cloud_time;
ros::Time Perception3d::query_time;


void Perception3d::handleCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
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

tf::Stamped<tf::Pose> Perception3d::getHandlePoseFromLaser(int pos)
{


    system("rosservice call laser_tilt_controller/set_periodic_cmd '{ command: { header: { stamp: 0 }, profile: \"linear\" , period: 10 , amplitude: 0.8 , offset: 0.3 }}'");

    handlePoses.clear();
    ros::NodeHandle n_;
    ros::Subscriber sub_= n_.subscribe("/handle_detector/pointcloud_line_pose_node/handle_pose", 100, &Perception3d::handleCallback);

    size_t numHandles = 0;
    ros::Rate rate(20);
    while (ros::ok() && (numHandles < 10))
    {
        handle_mutex.lock();
        if (Perception3d::handlePoses.size() > numHandles)
        {
            ROS_INFO("LAST HANDLE: %f %f %f ", handlePoses[numHandles]->getOrigin().x(),handlePoses[numHandles]->getOrigin().y(),handlePoses[numHandles]->getOrigin().z());
            numHandles = handlePoses.size();
        }
        handle_mutex.unlock();
        rate.sleep();
        ros::spinOnce();
    }

    return *handlePoses[0];

    /*tf::Stamped<tf::Pose> decid;
    decid.frame_id_ = "none";
    decid.stamp_ = ros::Time();
    decid.setOrigin(btVector3(1,0,0));
    decid.setRotation(btQuaternion(0,0,0,1));
    btTransform classify = transform * decid;

    tf::Stamped<tf::Pose> decid2;
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

btQuaternion resOri;

void Perception3d::handleCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{

    ros::Time t(msg->header.stamp);
    cloud_time = t;
    // ignore clouds taken before query time, since robot may have been moving, which results in problems since clouds are assembled in base_link
    if (t < query_time)
       return;
    //make this a boost shared ptr!
    handle_cloud_mutex.lock();
    pcl::fromROSMsg(*msg,lastCloud);

    bool takethisone = false;
    for (size_t k = 0; k < lastCloud.points.size(); k++)
    {
        btVector3 act(lastCloud.points[k].x,lastCloud.points[k].y,lastCloud.points[k].z);
        float dist = btVector3(act - handleHint).length();
        if (dist < handleMinDist)
        {
            handleMinDist = dist;
            handleResult = act;
            takethisone = true;
        }
    }

    // if this handle made it, calculate an orientation for it
    if (takethisone) {
       btVector3 l(lastCloud.points[0].x,lastCloud.points[0].y,lastCloud.points[0].z);
       btVector3 r(lastCloud.points[lastCloud.points.size() / 2].x,lastCloud.points[lastCloud.points.size() / 2].y,lastCloud.points[lastCloud.points.size() / 2].z);
       btVector3 rel = r - l;
       float at2 = atan2(rel.y(), rel.z());
       btQuaternion ori(btVector3(1,0,0), at2);
       resOri = ori;
    }

    //if (takethisone) {
    //   btVector3 l(lastCloud.points[0].x,lastCloud.points[0].y,lastCloud.points[0].z);
    //   btVector3 r(lastCloud.points[lastCloud.points.size() / 2].x,lastCloud.points[lastCloud.points.size() / 2].y,lastCloud.points[lastCloud.points.size() / 2].z);
    //   btVector3 rel = r - l;
    //   float at2 = atan2(rel.y(), rel.x());
    //   btQuaternion ori(btVector3(0,0,1), at2);
    //   resOri = ori;
    //}


    //btVector3 rel = leftEdge.getOrigin() - rightEdge.getOrigin();
    //float at2 = atan2(rel.y(), rel.x());
    //btQuaternion ori(btVector3(0,0,1), at2);


    //lastCloud = msg;
    op_handle_cloud_cnt++;

    ROS_INFO("GOT CLOUD %i", op_handle_cloud_cnt);
    handle_cloud_mutex.unlock();
}

tf::Stamped<tf::Pose> Perception3d::getHandlePoseFromLaser(tf::Stamped<tf::Pose> hint)
{

    system("rosservice call laser_tilt_controller/set_periodic_cmd '{ command: { header: { stamp: 0 }, profile: \"linear\" , period: 10 , amplitude: 0.8 , offset: 0.3 }}'");

    query_time = ros::Time::now();
    cloud_time = ros::Time(0);

    tf::Stamped<tf::Pose> inBase = RobotArm::getInstance(0)->getPoseIn("base_link", hint);

    ROS_INFO("HINT in map");
    RobotArm::getInstance(0)->printPose(hint);
    ROS_INFO("HINT in base");
    RobotArm::getInstance(0)->printPose(inBase);

    ros::NodeHandle n_;
    ros::Subscriber subHandleInliers = n_.subscribe("/handle_detector/handle_projected_inliers/output", 10, Perception3d::handleCloudCallback);
    //    ros::Subscriber subHandleInliers = n_.subscribe("/handle_detector/handle_candidates_protrude/output", 10, Perception3d::handleCloudCallback);

    bool found = false;
    ros::Rate rate(3);

    int maxNum = 5;

    btVector3 best(0,0,0);

    handleHint = inBase.getOrigin();
    handleMinDist = 100;
    float curBestDist = 100;
    int goodcloud = op_handle_cloud_cnt + 100000;

    ROS_INFO("Waiting for handles published after query");

    while (cloud_time < query_time) {
        rate.sleep();
        ros::spinOnce();
    }

    ros::Time current_timeslice = cloud_time;

    ROS_INFO("Waiting done");

    //while (((!found) || (op_handle_cloud_cnt < goodcloud + 2)) && (maxNum > 0))

    //while (!found && (maxNum > 0))
    while (cloud_time == current_timeslice)
    {
        rate.sleep();
        ros::spinOnce();
        if (handleMinDist < curBestDist)
        {
            // if we got better by at least 2 cm look for another 10 clouds
            //if (handleMinDist < curBestDist + 0.02)
                //goodcloud = op_handle_cloud_cnt;
            found = true;
            best = handleResult;
            curBestDist = handleMinDist;
        }
        ROS_INFO("Dist : %f", handleMinDist);
    }

    tf::Stamped<tf::Pose> ret;
    ret.frame_id_ = "base_link";
    ret.setOrigin(best);
    //ret.setRotation(btQuaternion(0,0,0,1));
    ret.setRotation(resOri);

    if (maxNum <= 0)
        ret.setOrigin(btVector3(0,0,-1));

    ROS_INFO("RET");
    RobotArm::getInstance(0)->printPose(ret);

    ret = RobotArm::getInstance(0)->getPoseIn(hint.frame_id_.c_str(), ret);

    return ret;
}


void Perception3d::bottleCallback(const ias_table_msgs::TableCluster::ConstPtr& msg)
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


tf::Stamped<tf::Pose> Perception3d::getBottlePose()
{

    //system("rosrun dynamic_reconfigure dynparam set  /camera_synchronizer_node projector_mode 3");
    //system("rosrun dynamic_reconfigure dynparam set  /camera_synchronizer_node narrow_stereo_trig_mode 3");
    int sysret = system("rosrun dynamic_reconfigure dynparam set  /camera_synchronizer_node '{projector_mode: 3, narrow_stereo_trig_mode: 3}'");
    ROS_INFO("%i rosrun dynamic_reconfigure dynparam set  /camera_synchronizer_node '{projector_mode: 3, narrow_stereo_trig_mode: 3}'", sysret);
    //handlePoses.clear();

    //tf::Stamped<tf::Pose> *newPose = new tf::Stamped<tf::Pose>();
    //handlePoses.push_back(newPose);

    ros::NodeHandle n_;
    ros::Subscriber sub_= n_.subscribe("/stereo_table_cluster_detector/pointcloud_minmax_3d_node/cluster", 100, &Perception3d::bottleCallback);

    size_t numHandles = 0;
    ros::Rate rate(20);

    btVector3 average(0,0,0);
    float numav = 0;

    tf::Stamped<tf::Pose> ret;

    while (ros::ok() && (numHandles < 1))
    {
        handle_mutex.lock();
        if (Perception3d::handlePoses.size() > numHandles)
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

void Perception3d::fridgePlaneCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    //make this a boost shared ptr!
    tf::Stamped<tf::Pose> *newPose = new tf::Stamped<tf::Pose>();
    newPose->frame_id_ = msg->header.frame_id;
    newPose->stamp_ = msg->header.stamp;
    pcl::PointXYZ point_max, point_min, point_center;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    pcl::getMinMax3D (cloud, point_min, point_max);
    //Calculate the centroid of the hull
    point_center.x = (point_max.x + point_min.x)/2;
    point_center.y = (point_max.y + point_min.y)/2;
    point_center.z = (point_max.z + point_min.z)/2;

    newPose->setOrigin(btVector3(point_center.x,point_center.y,point_center.z));
    //newPose->setRotation(btQuaternion(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w));
    ROS_INFO("CALLBACK GOT PLANE CENTER AT %f %f %f", point_center.x,point_center.y,point_center.z);
    plane_mutex.lock();
    planePoses.push_back(newPose);
    plane_mutex.unlock();
}

tf::Stamped<tf::Pose> Perception3d::getFridgePlaneCenterPose()
{
    int sysret = system("rosrun dynamic_reconfigure dynparam set  /camera_synchronizer_node '{projector_mode: 3, narrow_stereo_trig_mode: 3}'");
    ROS_INFO("%i rosrun dynamic_reconfigure dynparam set  /camera_synchronizer_node '{projector_mode: 3, narrow_stereo_trig_mode: 3}'", sysret);

    ros::NodeHandle n_;
    ros::Subscriber sub_= n_.subscribe("/stereo_table_cluster_detector/extract_clusters_on_table/table_inliers", 100, &Perception3d::fridgePlaneCloudCallback);

    size_t numPlanes = 0;
    ros::Rate rate(20);

    btVector3 average(0,0,0);
    float numav = 0;

    tf::Stamped<tf::Pose> ret;

    while (ros::ok() && (numPlanes < 1))
    {
        plane_mutex.lock();
        if (Perception3d::planePoses.size() > numPlanes)
        {
            ROS_INFO("LAST PLANE: %f %f %f ", planePoses[numPlanes]->getOrigin().x(),planePoses[numPlanes]->getOrigin().y(),planePoses[numPlanes]->getOrigin().z());
            average += btVector3(planePoses[numPlanes]->getOrigin().x(),planePoses[numPlanes]->getOrigin().y(),planePoses[numPlanes]->getOrigin().z());
            ret.stamp_ = planePoses[numPlanes]->stamp_;
            ret.frame_id_ = planePoses[numPlanes]->frame_id_;
            numav++;
            numPlanes = planePoses.size();
        }

        plane_mutex.unlock();
        rate.sleep();
        ros::spinOnce();
    }

    average *= 1.0f / numav;
    ret.setOrigin(average);

    ret = RobotArm::getPoseIn("map", ret);

    system("rosrun dynamic_reconfigure dynparam set  /camera_synchronizer_node projector_mode 1");

    return ret;
}
