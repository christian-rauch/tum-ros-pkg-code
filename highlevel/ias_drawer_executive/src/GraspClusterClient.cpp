
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

#include <signal.h>
#include <sensor_msgs/PointCloud2.h>
#include <object_manipulation_msgs/GraspPlanning.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

int side_ = 0;

tf::Stamped<tf::Pose> tool2wrist(tf::Stamped<tf::Pose> toolPose, double dist = -.18)
{
    tf::Stamped<tf::Pose> wrist2tool;
    //wrist2tool.frame_id_ = (side_==0) ? "r_wrist_roll_link" : "l_wrist_roll_link";
    wrist2tool.stamp_ = ros::Time();
    wrist2tool.setOrigin(btVector3(dist,0,0));
    wrist2tool.setRotation(btQuaternion(0,0,0,1));
    tf::Stamped<tf::Pose> ret;
    ret = toolPose;
    ret *= wrist2tool;
    return ret;
}


tf::Stamped<tf::Pose> wrist2tool(tf::Stamped<tf::Pose> toolPose, double dist = .18)
{
    tf::Stamped<tf::Pose> wrist2tool;
    //wrist2tool.frame_id_ = (side_==0) ? "r_wrist_roll_link" : "l_wrist_roll_link";
    wrist2tool.stamp_ = ros::Time();
    wrist2tool.setOrigin(btVector3(dist,0,0));
    wrist2tool.setRotation(btQuaternion(0,0,0,1));
    tf::Stamped<tf::Pose> ret;
    ret = toolPose;
    ret *= wrist2tool;
    return ret;
}



tf::TransformListener *listener_=0;

tf::Stamped<tf::Pose> getPoseIn(const char target_frame[], tf::Stamped<tf::Pose>src)
{

    if (src.frame_id_ == "NO_ID_STAMPED_DEFAULT_CONSTRUCTION") {
        ROS_ERROR("Frame not in TF: %s", "NO_ID_STAMPED_DEFAULT_CONSTRUCTION");
        tf::Stamped<tf::Pose> pose;
        return pose;
    }

    if (!listener_)
        listener_ = new tf::TransformListener();

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


int marker_id = 0;
ros::Publisher vis_pub;
visualization_msgs::MarkerArray mar;

void publishMarker(tf::Stamped<tf::Pose> mark)
{
        ros::NodeHandle node_handle;
        if (!marker_id) {
           vis_pub = node_handle.advertise<visualization_msgs::MarkerArray>( "/grasp_marker_array", 10, true );
        }

        visualization_msgs::Marker marker;
        marker.header.frame_id = mark.frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "grasps";
        marker.id =  ++marker_id;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.orientation.x = mark.getRotation().x();
        marker.pose.orientation.y = mark.getRotation().y();
        marker.pose.orientation.z = mark.getRotation().z();
        marker.pose.orientation.w = mark.getRotation().w();
        marker.pose.position.x = mark.getOrigin().x();
        marker.pose.position.y = mark.getOrigin().y();
        marker.pose.position.z = mark.getOrigin().z();
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        ROS_INFO("PUBLISH MARKER %i", marker.id);

        mar.markers.push_back(marker);
}


int main (int argc, char **argv)
{

    //signal(SIGINT,quit);

    ros::init(argc, argv, "grasp_cluster_client");

    system("rosrun dynamic_reconfigure dynparam set  /camera_synchronizer_node '{projector_mode: 3, narrow_stereo_trig_mode: 3}'");

    ros::NodeHandle n_;

    sensor_msgs::PointCloud2 pc2 = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/stereo_table_cluster_detector/pointcloud_minmax_3d_node/cloud_cluster"));
    sensor_msgs::PointCloud pc;
    sensor_msgs::convertPointCloud2ToPointCloud(pc2,pc);

    ros::ServiceClient client = n_.serviceClient<object_manipulation_msgs::GraspPlanning>("/plan_point_cluster_grasp");

    object_manipulation_msgs::GraspPlanning gp;

    gp.request.arm_name = "right_arm";
    //gp.request.target.reference_frame_id = pc2.header.frame_id;
    gp.request.target.cluster = pc;
    ROS_INFO("CALLING");
    if (client.call(gp)) {
        ROS_INFO("RETURNED: %zu Grasps", gp.response.grasps.size());
        for (size_t i = 0; i < gp.response.grasps.size(); ++i) {
            ROS_INFO("GRASP %zu: frame %s", i, gp.response.grasps[i].grasp_posture.header.frame_id.c_str());

            tf::Stamped<tf::Pose> grasp;
            tf::Stamped<tf::Pose> pregrasp;
            grasp.frame_id_ = gp.response.grasps[i].grasp_posture.header.frame_id = "/base_link";
            grasp.setOrigin(btVector3(gp.response.grasps[i].grasp_pose.position.x,gp.response.grasps[i].grasp_pose.position.y,gp.response.grasps[i].grasp_pose.position.z));
            grasp.setRotation(btQuaternion(gp.response.grasps[i].grasp_pose.orientation.x,gp.response.grasps[i].grasp_pose.orientation.y,gp.response.grasps[i].grasp_pose.orientation.z,gp.response.grasps[i].grasp_pose.orientation.w));
            pregrasp = wrist2tool(grasp,.13);
            grasp = wrist2tool(grasp);

            grasp=getPoseIn("/map",grasp);
            pregrasp=getPoseIn("/map",pregrasp);

            ROS_INFO("PREGRASP[map] %f %f %f %f %f %f %f", pregrasp.getOrigin().x(),pregrasp.getOrigin().y(),pregrasp.getOrigin().z(),
                pregrasp.getRotation().x(),pregrasp.getRotation().y(),pregrasp.getRotation().z(),pregrasp.getRotation().w());
            ROS_INFO("GRASP[map] %f %f %f %f %f %f %f", grasp.getOrigin().x(),grasp.getOrigin().y(),grasp.getOrigin().z(),
                grasp.getRotation().x(),grasp.getRotation().y(),grasp.getRotation().z(),grasp.getRotation().w());

            publishMarker(grasp);

        }
    }

    vis_pub.publish( mar );

    ros::AsyncSpinner spinner(2); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
