
/*
 * Copyright (c) 2011, Thomas Ruehr <ruehr@cs.tum.edu>
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
#include <eigen_conversions/eigen_msg.h>
#include <tf/tf.h>
#include <actionlib/server/simple_action_server.h>
#include <find_base_pose/FindBasePoseAction.h>
#include <tf/transform_listener.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetPositionFK.h>
#include <pr2_arm_kinematics/pr2_arm_kinematics.h>
#include <pr2_arm_kinematics/pr2_arm_ik.h>
#include <pr2_arm_kinematics/pr2_arm_kinematics_utils.h>
#include <pr2_arm_kinematics/pr2_arm_ik_solver.h>
#include <pr2_arm_kinematics/pr2_arm_kinematics_plugin.h>
#include <kinematics_base/kinematics_base.h>

#include <urdf/model.h>

#include <algorithm>

#include <laser_geometry/laser_geometry.h>
#include <visualization_msgs/Marker.h>

pr2_arm_kinematics::PR2ArmKinematics *kinemar,*kinemal;
pr2_arm_kinematics::PR2ArmIK *ik_r;
pr2_arm_kinematics::PR2ArmIKSolver *solver;



class FindBasePoseAction
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<find_base_pose::FindBasePoseAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    find_base_pose::FindBasePoseFeedback feedback_;
    find_base_pose::FindBasePoseResult result_;
    tf::TransformListener *listener_;
    ros::ServiceClient ik_client[2];

    //! We need laser scans for avoiding obstacles
    ros::Subscriber subScan_;
    laser_geometry::LaserProjection *projector_;
    boost::mutex scan_mutex;
    bool weHaveScan;
    //float scanPoints[5000][2];
    btVector3 scanPoints[5000];

    size_t numScanPoints;

    // bounding box of reachable area
    btVector3 bbox[2][2];

    ros::Publisher vis_pub;

    int markerCnt;

public:

    FindBasePoseAction(std::string name) :
            as_(nh_, name, boost::bind(&FindBasePoseAction::executeCB, this, _1), false),
            action_name_(name)
    {
        listener_ = new tf::TransformListener();
        for (int side = 0; side <= 1; ++side)
            ik_client[side] = nh_.serviceClient<kinematics_msgs::GetPositionIK>((side == 0) ? "/pr2_right_arm_kinematics/get_ik" : "/pr2_left_arm_kinematics/get_ik" , true);
        as_.start();


        subScan_ = nh_.subscribe("base_scan", 10, &FindBasePoseAction::scanCallback, this);

        vis_pub = nh_.advertise<visualization_msgs::Marker>( "find_base_pose_markers", 10000, true );

        projector_ = 0;
        markerCnt = 0;

        float padding = 0.02;

        bbox[0][0].setX(-0.564000 - padding);
        bbox[0][0].setY(-0.990000 - padding);
        bbox[0][0].setZ(0.336000 - padding);

        bbox[0][1].setX(0.759333 + padding);
        bbox[0][1].setY(0.396000 + padding);
        bbox[0][1].setZ(1.494000 + padding);

        bbox[1][0].setX(-0.564000 - padding);
        bbox[1][0].setY(-0.396000 - padding);
        bbox[1][0].setZ(0.336000 - padding);

        bbox[1][1].setX(0.759333 + padding);
        bbox[1][1].setY(0.990000 + padding);
        bbox[1][1].setZ(1.494000 + padding);
    }

    ~FindBasePoseAction(void)
    {
    }

    void pubBaseMarkerBlob(float dx, float dy, float dz, float colr = 0, float colg = 1, float colb = 0)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time::now();
        marker.ns = "target base footprints";
        marker.id = ++markerCnt;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = dx;
        marker.pose.position.y = dy;
        marker.pose.position.z = dz;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.a = 0.5;
        marker.color.r = colr;
        marker.color.g = colg;
        marker.color.b = colb;
        vis_pub.publish( marker );
        ros::spinOnce();
    }

    void pubBaseMarkerBlob(tf::Pose pose, float colr = 0, float colg = 1, float colb = 0)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time::now();
        marker.ns = "target base footprints";
        marker.id = ++markerCnt;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = pose.getOrigin().x();
        marker.pose.position.y = pose.getOrigin().y();
        marker.pose.position.z = pose.getOrigin().z();
        marker.pose.orientation.x = pose.getRotation().x();
        marker.pose.orientation.y = pose.getRotation().y();
        marker.pose.orientation.z = pose.getRotation().z();
        marker.pose.orientation.w = pose.getRotation().w();
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.a = 0.5;
        marker.color.r = colr;
        marker.color.g = colg;
        marker.color.b = colb;
        vis_pub.publish( marker );
        ros::spinOnce();
    }

    void pubBaseMarkerArrow(tf::Pose pose, float colr = 0, float colg = 1, float colb = 0)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time::now();
        marker.ns = "target base footprints";
        marker.id = ++markerCnt;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = pose.getOrigin().x();
        marker.pose.position.y = pose.getOrigin().y();
        marker.pose.position.z = pose.getOrigin().z();
        marker.pose.orientation.x = pose.getRotation().x();
        marker.pose.orientation.y = pose.getRotation().y();
        marker.pose.orientation.z = pose.getRotation().z();
        marker.pose.orientation.w = pose.getRotation().w();
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.a = 1;
        marker.color.r = colr;
        marker.color.g = colg;
        marker.color.b = colb;
        vis_pub.publish( marker );
        ros::spinOnce();
    }

    void pubBaseMarkerArrow(geometry_msgs::PoseStamped pose, float colr = 0, float colg = 1, float colb = 0)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = pose.header.frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "target base footprints";
        marker.id = ++markerCnt;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = pose.pose;
        marker.scale.x = 0.2;
        marker.scale.y = 0.7;
        marker.scale.z = 0.7;
        marker.color.a = 1;
        marker.color.r = colr;
        marker.color.g = colg;
        marker.color.b = colb;
        vis_pub.publish( marker );
        ros::spinOnce();

        //marker.header.stamp = ros::Time(0);
        //marker.id = ++markerCnt;
        //vis_pub.publish( marker );
        ros::spinOnce();

    }

    void pubBaseMarker(float dx, float dy, float dz = 0, float colr = 0, float colg = 1, float colb = 0, float si_ = 0.325)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time::now();
        marker.ns = "target base footprints";
        marker.id = ++markerCnt;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color.a = 0.5;
        marker.color.r = colr;
        marker.color.g = colg;
        marker.color.b = colb;
        float si = .325;
        //if (!isGood)
        si = .3;

        si = si_;

        //si = 0.01;
        //if (!isGood)
        //  marker.color.b = markerCnt / 1000.0f;
        {
            geometry_msgs::Point p;
            p.x =  -si +  dx;
            p.y =  -si +  dy;
            p.z = dz;
            marker.points.push_back(p);
        }
        {
            geometry_msgs::Point p;
            p.x =  si + dx;
            p.y =  -si +  dy;
            p.z = dz;
            marker.points.push_back(p);
        }
        {
            geometry_msgs::Point p;
            p.x =  si +  dx;
            p.y =  si +  dy;
            p.z = dz;
            marker.points.push_back(p);
        }
        {
            geometry_msgs::Point p;
            p.x =  -si +  dx;
            p.y =  si +  dy;
            p.z = dz;
            marker.points.push_back(p);
        }
        {
            geometry_msgs::Point p;
            p.x =  -si +  dx;
            p.y =  -si +  dy;
            p.z = dz;
            marker.points.push_back(p);
        }

        vis_pub.publish( marker );
    }

    void pubBaseMarker(tf::Pose pose, float colr = 0, float colg = 1, float colb = 0, float si_ = 0.325)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time::now();
        marker.ns = "target base footprints";
        marker.id = ++markerCnt;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = pose.getOrigin().x();
        marker.pose.position.y = pose.getOrigin().y();
        marker.pose.position.z = pose.getOrigin().z();
        marker.pose.orientation.x = pose.getRotation().x();
        marker.pose.orientation.y = pose.getRotation().y();
        marker.pose.orientation.z = pose.getRotation().z();
        marker.pose.orientation.w = pose.getRotation().w();
        /*marker.pose.position.x = 1;
        marker.pose.position.y = 0.5;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 1;
        marker.pose.orientation.w = 0.1;*/
        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color.a = 0.5;
        marker.color.r = colr;
        marker.color.g = colg;
        marker.color.b = colb;
        float si = .325;
        //if (!isGood)
        si = .3;

        si = si_;

        //si = 0.01;
        //if (!isGood)
        //  marker.color.b = markerCnt / 1000.0f;
        {
            geometry_msgs::Point p;
            p.x =  -si;
            p.y =  -si;
            p.z = 0;
            marker.points.push_back(p);
        }
        {
            geometry_msgs::Point p;
            p.x =  si;
            p.y =  -si;
            p.z = 0;
            marker.points.push_back(p);
        }
        {
            geometry_msgs::Point p;
            p.x =  si;
            p.y =  si;
            p.z = 0;
            marker.points.push_back(p);
        }
        {
            geometry_msgs::Point p;
            p.x =  -si;
            p.y =  si;
            p.z = 0;
            marker.points.push_back(p);
        }
        {
            geometry_msgs::Point p;
            p.x =  -si;
            p.y =  -si;
            p.z = 0;
            marker.points.push_back(p);
        }

        vis_pub.publish( marker );
    }

    void pubBaseMarker(std::vector<btVector3> pts, float colr = 0, float colg = 1, float colb = 0, float si_ = 0.325)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time::now();
        marker.ns = "target base footprints";
        marker.id = ++markerCnt;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color.a = 0.5;
        marker.color.r = colr;
        marker.color.g = colg;
        marker.color.b = colb;
        float si = si_;

        std::vector<btVector3>::iterator it;
        for (it = pts.begin(); it != pts.end(); ++it)
        {
            geometry_msgs::Point p;
            p.x =  it->x();
            p.y =  it->y();
            p.z =  it->z();
            marker.points.push_back(p);
        }
        it = pts.begin();
        {
            geometry_msgs::Point p;
            p.x =  it->x();
            p.y =  it->y();
            p.z =  it->z();
            marker.points.push_back(p);
        }

        vis_pub.publish( marker );
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        if (!projector_)
            projector_ = new laser_geometry::LaserProjection();

        scan_mutex.lock();
        sensor_msgs::PointCloud scan_cloud;
        projector_->projectLaser(*scan_in,scan_cloud);
        int step = 1;
        for (size_t k = 0; k < scan_cloud.points.size(); k += step)
        {
            //scanPoints[k/step][0] = scan_cloud.points[k].x;
            //scanPoints[k/step][1] = scan_cloud.points[k].y;
            scanPoints[k] = btVector3(scan_cloud.points[k].x + 0.275, scan_cloud.points[k].y, 0);
            numScanPoints = k / step;
            //ROS_INFO("SCAN POINTS : %i %f %f",numScanPoints, scanPoints[k/step][0], scanPoints[k/step][1]);
        }
        scan_mutex.unlock();
        weHaveScan = true;
    }

    bool collisionFree(float relativePose[])
    {
        ros::Rate rate(10);
        while (!weHaveScan)
        {
            rate.sleep();
            ros::spinOnce();
        }
        scan_mutex.lock();
        //ROS_INFO("SCAN POINTS : %i",numScanPoints);
        bool good = true;
        float padding = 0.05;
        for (size_t k = 0; good && (k < numScanPoints); k += 1)
        {
            //float x = scanPoints[k][0] - relativePose[0] + 0.275;
            //float y = scanPoints[k][1] - relativePose[1];
            float x = scanPoints[k].x() - relativePose[0] + 0.275;
            float y = scanPoints[k].y() - relativePose[1];
            if ((x < .325 + padding) && (x > -.325 - padding) && (y < .325 + padding) && (y > -.325 - padding))
            {
                //ROS_INFO("POINT %f %f",  scanPoints[k][0] , scanPoints[k][1]);
                good = false;
            }
        }
        scan_mutex.unlock();
        return good;
    }

    //bool collisionFree(tf::Stamped<tf::Pose> relativePose)
    bool collisionFree(btTransform relativePose)
    {
        ros::Rate rate(10);
        while (!weHaveScan)
        {
            rate.sleep();
            ros::spinOnce();
        }
        scan_mutex.lock();
        //ROS_INFO("SCAN POINTS : %i",numScanPoints);
        bool good = true;
        float padding = 0.05;
        for (size_t k = 0; good && (k < numScanPoints); k += 1)
        {

            //float x = scanPoints[k].x() - relativePose.getOrigin().x() + 0.275;
            //float y = scanPoints[k].y()  - relativePose.getOrigin().y();
            btVector3 in = scanPoints[k];
            btVector3 transformed = relativePose.invXform(in);
            //btVector3 transformed = in * relativePose;
            if ((transformed.x() < .325 + padding) && (transformed.x() > -.325 - padding) && (transformed.y() < .325 + padding) && (transformed.y() > -.325 - padding))
            {
                //ROS_INFO("POINT %f %f",  scanPoints[k][0] , scanPoints[k][1]);
                good = false;
            }
        }
        scan_mutex.unlock();
        return good;
    }

    tf::Stamped<tf::Pose> tool2wrist(tf::Stamped<tf::Pose> toolPose)
    {
        tf::Stamped<tf::Pose> wrist2tool;
        wrist2tool.stamp_ = ros::Time();
        wrist2tool.setOrigin(btVector3(-.18,0,0));
        wrist2tool.setRotation(btQuaternion(0,0,0,1));
        tf::Stamped<tf::Pose> ret;
        ret = toolPose;
        ret *= wrist2tool;
        return ret;
    }

    /*tf::Stamped<tf::Pose> getPoseIn(const char target_frame[], tf::Stamped<tf::Pose>src)
    {
        tf::Stamped<tf::Pose> transform;
        listener_->waitForTransform(src.frame_id_, target_frame,ros::Time(0), ros::Duration(30.0));
        listener_->transformPose(target_frame, src, transform);
        return transform;
    }*/

    tf::Stamped<tf::Pose> getPoseIn(const char target_frame[], tf::Stamped<tf::Pose>src)
    {

        if (!listener_)
            listener_ = new tf::TransformListener();

        // tf::Stamped<tf::Pose>
        tf::Stamped<tf::Pose> transform;
        //this shouldnt be here TODO
        src.stamp_ = ros::Time(0);

        listener_->waitForTransform(src.frame_id_, target_frame,
                                    ros::Time(0), ros::Duration(30.0));
        bool transformOk = false;
        while (!transformOk)
        {
            try
            {
                transformOk = true;
                listener_->transformPose(target_frame, src, transform);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("getPoseIn %s",ex.what());
                // dirty:
                src.stamp_ = ros::Time(0);
                transformOk = false;
            }
            ros::spinOnce();
        }
        return transform;
    }

    bool insideBB(int side, geometry_msgs::PoseStamped pose)
    {
        btVector3 *min = &bbox[side][0];
        btVector3 *max = &bbox[side][1];
        if ((pose.pose.position.x >= min->x()) &&
                (pose.pose.position.x <= max->x()) &&
                (pose.pose.position.y >= min->y()) &&
                (pose.pose.position.y <= max->y()))
            return true;
        else
            return false;
    }


    tf::Pose rotateAroundPose(tf::Pose toolPose, tf::Pose pivot, btQuaternion qa)
    {
        btTransform curr = toolPose;
        btTransform pivo = pivot;

        curr = pivo.inverseTimes(curr);

        btTransform rot;
        rot.setOrigin(btVector3(0,0,0));
        rot.setRotation(qa);
        curr = rot * curr;
        curr = pivo * curr;

        tf::Pose act;
        //act.frame_id_ = toolPose.frame_id_;
        act.setOrigin(curr.getOrigin());
        act.setRotation(curr.getRotation());

        return act;
    }



    bool run_ik(int side_,geometry_msgs::PoseStamped pose, double start_angles[7],
                double solution[7]) //, std::string link_name)
    {

        kinematics_msgs::GetPositionIK::Request  ik_request;
        kinematics_msgs::GetPositionIK::Response ik_response;

        ik_request.timeout = ros::Duration(5.0);
        if (side_ == 0)
        {
            ik_request.ik_request.ik_seed_state.joint_state.name.push_back("r_shoulder_pan_joint");
            ik_request.ik_request.ik_seed_state.joint_state.name.push_back("r_shoulder_lift_joint");
            ik_request.ik_request.ik_seed_state.joint_state.name.push_back("r_upper_arm_roll_joint");
            ik_request.ik_request.ik_seed_state.joint_state.name.push_back("r_elbow_flex_joint");
            ik_request.ik_request.ik_seed_state.joint_state.name.push_back("r_forearm_roll_joint");
            ik_request.ik_request.ik_seed_state.joint_state.name.push_back("r_wrist_flex_joint");
            ik_request.ik_request.ik_seed_state.joint_state.name.push_back("r_wrist_roll_joint");

            ik_request.ik_request.robot_state.joint_state.name.push_back("r_shoulder_pan_joint");
            ik_request.ik_request.robot_state.joint_state.name.push_back("r_shoulder_lift_joint");
            ik_request.ik_request.robot_state.joint_state.name.push_back("r_upper_arm_roll_joint");
            ik_request.ik_request.robot_state.joint_state.name.push_back("r_elbow_flex_joint");
            ik_request.ik_request.robot_state.joint_state.name.push_back("r_forearm_roll_joint");
            ik_request.ik_request.robot_state.joint_state.name.push_back("r_wrist_flex_joint");
            ik_request.ik_request.robot_state.joint_state.name.push_back("r_wrist_roll_joint");
        }
        else
        {
            ik_request.ik_request.ik_seed_state.joint_state.name.push_back("l_shoulder_pan_joint");
            ik_request.ik_request.ik_seed_state.joint_state.name.push_back("l_shoulder_lift_joint");
            ik_request.ik_request.ik_seed_state.joint_state.name.push_back("l_upper_arm_roll_joint");
            ik_request.ik_request.ik_seed_state.joint_state.name.push_back("l_elbow_flex_joint");
            ik_request.ik_request.ik_seed_state.joint_state.name.push_back("l_forearm_roll_joint");
            ik_request.ik_request.ik_seed_state.joint_state.name.push_back("l_wrist_flex_joint");
            ik_request.ik_request.ik_seed_state.joint_state.name.push_back("l_wrist_roll_joint");

            ik_request.ik_request.robot_state.joint_state.name.push_back("l_shoulder_pan_joint");
            ik_request.ik_request.robot_state.joint_state.name.push_back("l_shoulder_lift_joint");
            ik_request.ik_request.robot_state.joint_state.name.push_back("l_upper_arm_roll_joint");
            ik_request.ik_request.robot_state.joint_state.name.push_back("l_elbow_flex_joint");
            ik_request.ik_request.robot_state.joint_state.name.push_back("l_forearm_roll_joint");
            ik_request.ik_request.robot_state.joint_state.name.push_back("l_wrist_flex_joint");
            ik_request.ik_request.robot_state.joint_state.name.push_back("l_wrist_roll_joint");
        }

        ik_request.ik_request.ik_link_name = side_ ? "l_wrist_roll_link" : "r_wrist_roll_link";

        ik_request.ik_request.pose_stamped = pose;
        ik_request.ik_request.ik_seed_state.joint_state.position.resize(7);
        ik_request.ik_request.robot_state.joint_state.position.resize(7);

        for (int i=0; i<7; i++)
        {
            ik_request.ik_request.ik_seed_state.joint_state.position[i] = start_angles[i];
            ik_request.ik_request.robot_state.joint_state.position[i] = start_angles[i];
        }

        //ROS_INFO("request pose: %s %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f",ik_request.ik_request.ik_link_name.c_str() , pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);

        //ROS_INFO("error code a %i",ik_response.error_code.val);

        bool ik_service_call ;

        if (0)
        {
            ik_service_call = ik_client[side_].call(ik_request,ik_response);
        }
        //for (int i = 0; i < 1; ++i)
        //if (0)
        {
            if (side_==0)
                ik_service_call  = kinemar->getPositionIK(ik_request,ik_response);
            else
                ik_service_call  = kinemal->getPositionIK(ik_request,ik_response);
        }
        //bool ik_service_call = true;
        //Eigen::eigen2_Transform3d pose_eigen;
        //KDL::Frame pose_kdl;
        //tf::PoseMsgToKDL(pose.pose, pose_kdl);
        //tf::poseMsgToEigen(pose.pose, pose_eigen);
        //Eigen::Matrix4f pose_eigen = pr2_arm_kinematics::KDLToEigenMatrix(pose_kdl);

        //std::vector<std::vector<double> > direct_solution;
        //int numtry = 0;
        //while ((numtry++ < 50) && (direct_solution.size() == 0)) {
        //double free_angle = ((rand() % 10000) / 10000.0) * 2 * M_PI  - M_PI;
        //ROS_INFO("free_angle = %f", free_angle);
        //ik_r->computeIKShoulderRoll(pose_eigen, free_angle,direct_solution);
        //}

        //ROS_INFO("len %i", direct_solution.size());
        //if (direct_solution.size()>0)
        //ROS_INFO("len first %i", direct_solution[0].size());
        //(const KDL::JntArray& q_init,
        //      const KDL::Frame& p_in,
        //    std::vector<KDL::JntArray> &q_out);
        //KDL::JntArray q_init;
        //bool found = solver->CartToJntSearch(

        if (!ik_service_call)
        {
            ROS_ERROR("IK service call failed! is ik service running? /pr2_right_arm_kinematics/get_ik");
            return false;
        }

        //        if (ik_response.error_code.val ==  pr2_arm_kinematics::TIMED_OUT)

        //ROS_INFO("error code %i",ik_response.error_code.val);
        if (ik_response.error_code.val == pr2_arm_kinematics::NO_IK_SOLUTION)
        {
            //  ROS_INFO("no solution found");
            return false;
        }


        if (ik_response.error_code.val == ik_response.error_code.SUCCESS)
        {
            for (int i=0; i<7; i++)
            {
                solution[i] = ik_response.solution.joint_state.position[i];
            }
            //ROS_INFO("solution angles: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f",solution[0],solution[1], solution[2],solution[3],solution[4],solution[5],solution[6]);
            //ROS_INFO("IK service call succeeded, solution found");
            return true;
        }
        //ROS_INFO("IK service call error code: %d", ik_response.error_code.val);
        return false;
    }

    // todo: we should penalize angle/ robot rotation also!
    static bool pose_compare(tf::Pose a,tf::Pose b)
    {
        //float angle_factor = 0.1 / (15 / 180 * M_PI); // 30 deg turn is equal to 10cm drive
        float angle_factor = 0.5;
        return (a.getOrigin().length() + angle_factor * a.getRotation().getAngle() < b.getOrigin().length() + angle_factor * b.getRotation().getAngle());
    }

    void generateCandidatePoses(std::vector<tf::Pose> &search_poses,std::vector<tf::Stamped<tf::Pose> > target_poses , tf::Pose preTrans, const std::vector<std_msgs::Int32> &arm)
    {

        std::vector<tf::Pose>  target_poses_transformed;
        std::vector<tf::Stamped<tf::Pose> >::iterator it;
        for (it = target_poses.begin(); it != target_poses.end(); ++it)
        {
            tf::Pose act;
            act.setOrigin(it->getOrigin());
            act.setRotation(it->getRotation());
            //act = act * preTrans;
            act = preTrans.inverseTimes(act);
            target_poses_transformed.push_back(act);
        }

        double minx = target_poses_transformed[0].getOrigin().x()-bbox[arm[0].data][1].x(); //goalInBaseMsg[0].pose.position.x - reach;
        double maxx = target_poses_transformed[0].getOrigin().x()-bbox[arm[0].data][0].x(); //goalInBaseMsg[0].pose.position.x + reach;
        double miny = target_poses_transformed[0].getOrigin().y()-bbox[arm[0].data][1].y();//goalInBaseMsg[0].pose.position.y - reach;
        double maxy = target_poses_transformed[0].getOrigin().y()-bbox[arm[0].data][0].y();//goalInBaseMsg[0].pose.position.y + reach;
        //}(-1.008435,-1.813615)-(1.591565,0.786385)

        // max of mins and vice versa gives the overlap of approx inverse capability maps
        ROS_INFO("GOAL TARGETS SIZE: %zu", target_poses_transformed.size());
        for (size_t i=0; i < target_poses_transformed.size(); ++i)
        {
            // take 1.5m for approx upper bound of arms reach
            minx = std::max(target_poses_transformed[i].getOrigin().x()-bbox[arm[i].data][1].x(),minx);
            maxx = std::min(target_poses_transformed[i].getOrigin().x()-bbox[arm[i].data][0].x(),maxx);
            miny = std::max(target_poses_transformed[i].getOrigin().y()-bbox[arm[i].data][1].y(),miny);
            maxy = std::min(target_poses_transformed[i].getOrigin().y()-bbox[arm[i].data][0].y(),maxy);
            ROS_INFO("%zu BOUNDING BOX : (%f,%f)-(%f,%f) size : %f %f ",i, minx,miny, maxx, maxy, maxx - minx, maxy - miny);
            //pubBaseMarker(goalInBaseMsg[i].pose.position.x,goalInBaseMsg[i].pose.position.y,goalInBaseMsg[i].pose.position.z,1,0,0);
        }

        ROS_INFO("BOUNDING BOX : (%f,%f)-(%f,%f) size : %f %f ", minx,miny, maxx, maxy, maxx - minx, maxy - miny);

        //discretize on a grid around 0,0
        double discretisation = 0.075;
        //discretisation = 0.2;
        //discretize and padd by one grid cell
        minx = floor(minx * (1 / discretisation) - 1) * discretisation;
        maxx =  ceil(maxx * (1 / discretisation) + 1) * discretisation;

        miny = floor(miny * (1 / discretisation) - 1) * discretisation;
        maxy =  ceil(maxy * (1 / discretisation) + 1) * discretisation;

        std::vector<btVector3> bbpts;
        bbpts.push_back(preTrans * btVector3(minx,miny,0));
        bbpts.push_back(preTrans * btVector3(minx,maxy,0));
        bbpts.push_back(preTrans * btVector3(maxx,maxy,0));
        bbpts.push_back(preTrans * btVector3(maxx,miny,0));


        //std::vector<tf::Pose> search_poses;
        for (double xgen = minx; xgen <= maxx; xgen += discretisation)
        {
            for (double ygen = miny; ygen <= maxy; ygen += discretisation)
            {
                tf::Pose act;
                act.setOrigin(btVector3(xgen,ygen,0));
                //act.setRotation(btQuaternion(btVector3(0, 0, 1), -0.1));
                act.setRotation(btQuaternion(0,0,0,1));

                act = preTrans * act;

                //act = rotateAroundPose(act, , btQuaternion qa)

                //float po[2] = {xgen,ygen};

                //if (collisionFree(po))
                //if (collisionFree(act))
                //{
                search_poses.push_back(act);
                //pubBaseMarker(xgen,ygen);
                //pubBaseMarker(act);
                //pubBaseMarkerBlob(act,0,1,0.5);
                //}
                //else
                //{
                //ROS_INFO("tick");
                //pubBaseMarkerBlob(xgen,ygen,1,0,0,0.5);
                //}
                //pubBaseMarker(act);
                //pubBaseMarkerBlob(act,1,.7,0.3);
                ////pubBaseMarkerBlob(xgen,ygen,0,0,1,0.5);
                //if (0)
                //    for (int u = 0; u < 15; u++)
                //    {
                //        ros::Duration(0.001).sleep();
                //        ros::spinOnce();
                //    }
            }
        }

    }



    void executeCB(const find_base_pose::FindBasePoseGoalConstPtr &goal)
    {
        // helper variables
        ros::Rate r(5000);
        bool success = true;

        result_.base_poses.clear();

        // publish info to the console for the user
        //  ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

        // start executing the action
        //for(int i=1; i<=goal->order; i++)

        if (goal->target_poses.size() == 0)
        {
            as_.setAborted();
            ROS_ERROR("find_base_pose called without target trajectories");
            return; // todo: set result accordingly
        }

        ROS_INFO("New goal");

        std::vector<tf::Stamped<tf::Pose > > goalInBase;
        goalInBase.resize(goal->target_poses.size());

        std::vector<geometry_msgs::PoseStamped> goalInBaseMsg;
        goalInBaseMsg.resize(goal->target_poses.size());

        double reach = 1.3;

        for (size_t i=0; i < goal->target_poses.size(); ++i)
        {
            tf::Stamped<tf::Pose> act;
            poseStampedMsgToTF(goal->target_poses[i], act);
            //get in baselink and convert to wrist pose
            //goalInBase[i] = tool2wrist(getPoseIn("base_link", act));
            goalInBase[i] = tool2wrist(getPoseIn("base_link", act));
            poseStampedTFToMsg(goalInBase[i], goalInBaseMsg[i]);

            for (int k = 0 ; k < 5 ; k++)
            {
                //pubBaseMarkerArrow(goal->target_poses[i],goal->arm[i].data,1-goal->arm[i].data,0);
                pubBaseMarkerArrow(goalInBaseMsg[i],goal->arm[i].data,1-goal->arm[i].data,0);
                ros::spinOnce();
                ros::spinOnce();
                ros::spinOnce();
                ros::Duration(0.0001).sleep();
            }

            ROS_INFO("FRAME: %s", goalInBaseMsg[i].header.frame_id.c_str());
            ROS_INFO("POSE: %f %f %f", goalInBaseMsg[i].pose.position.x, goalInBaseMsg[i].pose.position.y, goalInBaseMsg[i].pose.position.z);
            ROS_INFO("ROT: %f %f %f %f", goalInBaseMsg[i].pose.orientation.x, goalInBaseMsg[i].pose.orientation.y, goalInBaseMsg[i].pose.orientation.z, goalInBaseMsg[i].pose.orientation.w);
        }

        std::vector<tf::Pose> search_poses;

        tf::Pose preTrans;
        ros::Rate rat(1000);
        std::vector<float> angles;
        angles.push_back(0);
        angles.push_back(-30 * M_PI / 180.0f );
        angles.push_back(+30 * M_PI / 180.0f );
        if (0)
            for (std::vector<float>::iterator it = angles.begin(); it != angles.end(); ++it)
            {
                preTrans.setRotation(btQuaternion(btVector3(0, 0, 1), *it));
                generateCandidatePoses(search_poses,goalInBase, preTrans, goal->arm);
                rat.sleep();
            }
        for (float angle = -180 * M_PI / 180.0f ; angle <= 180 * M_PI / 180.0f; angle+= 30 * M_PI / 180.0f)
        {
            preTrans.setRotation(btQuaternion(btVector3(0, 0, 1), angle));
            generateCandidatePoses(search_poses,goalInBase, preTrans, goal->arm);
            //rat.sleep();
        }



        ROS_INFO("SIZE Of searchspace : %i .. sorting", (int)search_poses.size());

        std::sort(search_poses.begin(), search_poses.end(), pose_compare);

        std::vector<tf::Pose>::iterator it;
        //for (it = search_poses.begin(); it!=search_poses.end(); ++it)
        //{
        //ROS_INFO("POS: %f %f (dist %f)", it->getOrigin().x(), it->getOrigin().y(), it->getOrigin().length());
        //}

        ROS_INFO("SIZE Of searchspace : %i .. sorted", (int)search_poses.size());

        double start_angles[2][7];
        double sol_angles[7];
        for (int i=0; i < 7; ++i)
        {
            start_angles[0][i] = 0;
            start_angles[1][i] = 0;
        }

        //double distResolution = 0.2;
        //double currdist = 0;

        /*float max_dist = 0;

        float bbx[2] = {1000,-1000};
        float bby[2] = {1000,-1000};
        float bbz[2] = {1000,-1000};

        int cnt = 0;

        //for generating bounding boxes

        if (0)
        while (ros::ok()) {

            cnt++;

            if (cnt % 1000 == 0)
              ROS_INFO("CNT %i", cnt);

            double x, y, z;
            x = (rand() % 3000 - 1500) / 1500.0f;
            y = (rand() % 3000 - 1500) / 1500.0f;
            z = (rand() % 2000 ) / 1000.0f;
            geometry_msgs::PoseStamped currgoal = goalInBaseMsg[0];
            currgoal.header.frame_id = "base_link";
            currgoal.pose.position.x = x;
            currgoal.pose.position.y = y;
            currgoal.pose.position.z = z;
            currgoal.pose.orientation.x = (rand() % 200 - 100) / 100.0f;
            currgoal.pose.orientation.y = (rand() % 200 - 100) / 100.0f;
            currgoal.pose.orientation.z = (rand() % 200 - 100) / 100.0f;
            currgoal.pose.orientation.w = (rand() % 200 - 100) / 100.0f;
            //bool good = (insideBB(0,currgoal) && run_ik(0,currgoal,start_angles[0],sol_angles));
            bool good = run_ik(0,currgoal,start_angles[0],sol_angles);
            if (good) {
              //pubBaseMarkerBlob(x,y,z,0,1,0);
              btVector3 dive(x,y,0);
              if (dive.length() > max_dist)
                max_dist = dive.length();
              ROS_INFO("xyz %f %f %f, max len %f", x,y,z, max_dist);

              if (x < bbx[0])
                bbx[0] = x;
              if (x > bbx[1])
                bbx[1] = x;
              if (y < bby[0])
                bby[0] = y;
              if (y > bby[1])
                bby[1] = y;
              if (z < bbz[0])
                bbz[0] = z;
              if (z > bbz[1])
                bbz[1] = z;

              ROS_INFO("BBOX : %f %f  %f %f  %f %f", bbx[0], bbx[1], bby[0], bby[1], bbz[0], bbz[1]);
              ROS_INFO("BBOX size : %f %f  %f ", bbx[1] - bbx[0], bby[1] - bby[0], bbz[1] - bbz[0]);

            }
            //else
            //              pubBaseMarkerBlob(x,y,z,1,0,0);

            ros::spinOnce();ros::spinOnce();ros::spinOnce();
        }
        */

        bool found = false;
        //while (!found)
        int counter = 0;

        float mindist = 10000;
        for (it = search_poses.begin(); (it!=search_poses.end()) && (!found); ++it)
            //for (it = search_poses.begin(); it!=search_poses.end(); ++it)
        {

            //double x_add = it->getOrigin().x();
            //double y_add = it->getOrigin().y();

            //btVector3 a(x_add, y_add,0);

            bool good = true;
            geometry_msgs::PoseStamped currgoal;
            //for (int z=0; z < 1000; z++) // for benchmarking

            for (size_t i=0; (i < goal->target_poses.size()) & good; ++i)
            {
                currgoal = goalInBaseMsg[i];
                tf::Pose actP;
                tf::poseMsgToTF(currgoal.pose,actP);
                //ROS_INFO("TRANSFORM %f %f %f", it->getOrigin().x(), it->getOrigin().y(),it->getOrigin().z());
                //ROS_INFO("TRANSFORM AXIS %f %f %f ROT %f ", it->getRotation().getAxis().x(), it->getRotation().getAxis().y(), it->getRotation().getAxis().z(), it->getRotation().getAngle());
                //ROS_INFO("BEFORE %f %f %f", currgoal.pose.position.x, currgoal.pose.position.y, currgoal.pose.position.z);
                //btTransform tr =
                actP = it->inverseTimes(actP);
                //actP.setOrigin(tr.getOrigin());
                //actP.setRotation(tr.getRotation());

                //currgoal.pose.position.x = goalInBaseMsg[i].pose.position.x - x_add;
                //currgoal.pose.position.y = goalInBaseMsg[i].pose.position.y - y_add;
                tf::poseTFToMsg(actP,currgoal.pose);
                //ROS_INFO("AFTER %f %f %f", currgoal.pose.position.x, currgoal.pose.position.y, currgoal.pose.position.z);
                good &= run_ik(goal->arm[i].data,currgoal ,start_angles[0],sol_angles);
            }

            //ROS_INFO("x %f y %f d %f good %i", x_add, y_add, a.length(), good);

            if (good)
                if (collisionFree(*it))
                {
                    if (!found)
                    {
                        //ROS_INFO("x %f y %f d %f d+angle %f good %i", x_add, y_add, a.length(), a.length() + 0.5 * it->getRotation().getAngle(), good);
                        ROS_INFO("XXXXXXXXXXX FOUND A COMBINED SOLUTION: %f %f %f", it->getOrigin().x(), it->getOrigin().y(), it->getRotation().getAngle());
                    }
                    found = true;

                    geometry_msgs::PoseStamped psm;
                    tf::poseTFToMsg(*it, psm.pose);
                    psm.header.frame_id = "/base_link";
                    psm.header.stamp = ros::Time::now();
                    result_.base_poses.push_back(psm);
                    //ROS_INFO("x %f y %f d %f d+angle %f good %i", x_add, y_add, a.length(), a.length() + 0.5 * it->getRotation().getAngle(), good);
                    //ROS_INFO("XXXXXXXXXXX FOUND A COMBINED SOLUTION: %f %f %f", x_add, y_add, it->getRotation().getAngle());
                    //ROS_INFO("SOL ANGLES : %f %f %f %f %f %f %f", sol_angles[0], sol_angles[1], sol_angles[2], sol_angles[3], sol_angles[4], sol_angles[5], sol_angles[6]);
                    //pubBaseMarker(x_add,y_add);
                    //pubBaseMarkerBlob(x_add,y_add,0.05,0.7,0.5,0.2);
                    tf::Pose ps;
                    ps.setOrigin(it->getOrigin() + btVector3(0,0,0.01));
                    ps.setRotation(it->getRotation());
                    pubBaseMarker(ps,0.7,0.5,0.2);
                    float col_factor = ps.getOrigin().length() + 0.5 * it->getRotation().getAngle();
                    if (col_factor < mindist)
                        mindist = col_factor;
                    col_factor -= mindist;
                    col_factor *= 0.05;
                    ps.setOrigin(it->getOrigin() + btVector3(0,0,col_factor));
                    pubBaseMarkerArrow(ps,1-col_factor*20.0,0,col_factor * 5.0);
                    //ROS_INFO("pose in base %f %f rot %f", currgoal.pose.position.x, currgoal.pose.position.y,0);
                    for (int r = 0; r < 10; r++)
                    {
                        ros::spinOnce();
                        ros::Duration(0.0001).sleep();
                        ros::spinOnce();
                    }
                }

            // check that preempt has not been requested by the client
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                success = false;
                break;
            }
            //if (counter++ % 100 == 1)
            //r.sleep();
        }


        if (success)
        {
            //    result_.sequence = feedback_.sequence;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            as_.setSucceeded(result_);
        }
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "find_base_pose");

    tf::TransformListener tflistener;

    //pr2_arm_kinematics::PR2ArmKinematics pr2_arm_kinematics;



    // ik_r = new pr2_arm_kinematics::PR2ArmIK();
    //urdf::Model robot_model;
    //std::string root_name, tip_name, xml_string;
    //pr2_arm_kinematics::loadRobotModel(ros::NodeHandle(),robot_model,root_name,tip_name,xml_string);
    //ik_r->init(robot_model,root_name,tip_name);


    // solver = new pr2_arm_kinematics::PR2ArmIKSolver(robot_model, root_name, tip_name, 0.1, 2);
    // plugin_ = new pr2_arm_kinematics::PR2ArmKinematicsPlugin();

    ros::param::set("/find_base_pose/root_name", "torso_lift_link");
    ros::param::set("/find_base_pose/tip_name", "r_wrist_roll_link");

    ros::Rate rt(10);

    while (!(ros::param::has("/find_base_pose/root_name") && ros::param::has("/find_base_pose/tip_name") ))
    {
        rt.sleep();
        ros::spinOnce();
        ROS_INFO("Waiting for params to show");
    }


    pr2_arm_kinematics::PR2ArmKinematics kinem;
    kinemar = &kinem;

    ros::param::set("/find_base_pose/root_name", "torso_lift_link");
    ros::param::set("/find_base_pose/tip_name", "l_wrist_roll_link");

    std::string tip;
    ros::param::get("/find_base_pose/tip_name",tip);
    ROS_INFO("TIP NAME %s", tip.c_str());

    pr2_arm_kinematics::PR2ArmKinematics kineml;
    kinemal = &kineml;

    ROS_ERROR("ABOVE ERRORS (Tried to advertise) can be neglected!");

    tflistener.waitForTransform("base_link", "torso_lift_link", ros::Time::now(), ros::Duration(5.0));
    tflistener.waitForTransform("base_link", "map", ros::Time::now(), ros::Duration(5.0));

    //FindBasePoseAction findbasepose(ros::this_node::getName());
    FindBasePoseAction findbasepose("find_base_pose_action");

    ROS_INFO("NAME: %s", ros::this_node::getName().c_str());

    ROS_INFO("DONE with test");

    ros::spin();

    return 0;
}
