#include <ros/ros.h>

#include <iostream>
#include <fstream>
using namespace std;

#include <ias_drawer_executive/Approach.h>
#include <ias_drawer_executive/RobotDriver.h>
#include <ias_drawer_executive/Gripper.h>
#include <ias_drawer_executive/Geometry.h>
#include <ias_drawer_executive/Pressure.h>
#include <ias_drawer_executive/RobotArm.h>
#include <ias_drawer_executive/Torso.h>
#include <ias_drawer_executive/Head.h>
#include <ias_drawer_executive/OperateHandleController.h>
#include <ias_drawer_executive/Geometry.h>

#include <ias_drawer_executive/Current.h>

#include <ias_drawer_executive/ObjectLocalizer.h>

#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>

#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <boost/thread.hpp>

#include <visualization_msgs/Marker.h>


#include "rosbag/bag.h"
#include "rosbag/query.h"
#include "rosbag/view.h"

#include <ias_drawer_executive/Kinect.h>


ros::Publisher perc_announcer; // perception announcer, sends a marker whenever something is perceived
bool perc_init = false;
int marker_id = 1;



void announce(const std::string text, const tf::Stamped<tf::Pose> &poseStamped)
{
    if (!perc_init)
    {
        perc_init = true;
        ros::NodeHandle node_handle;
        perc_announcer =  node_handle.advertise<visualization_msgs::Marker>( "/perception_announcer", 0 , true);
    }

    tf::Pose pose;
    pose.setOrigin(poseStamped.getOrigin());
    pose.setRotation(poseStamped.getRotation());

    visualization_msgs::Marker marker;
    marker.header.frame_id = poseStamped.frame_id_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "perception_announcer";
    marker.id =  marker_id++;
    marker.text = text;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.x = pose.getRotation().x();
    marker.pose.orientation.y = pose.getRotation().y();
    marker.pose.orientation.z = pose.getRotation().z();
    marker.pose.orientation.w = pose.getRotation().w();
    marker.pose.position.x = pose.getOrigin().x();
    marker.pose.position.y = pose.getOrigin().y();
    marker.pose.position.z = pose.getOrigin().z();
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.4;

    ROS_INFO("PUBLISH MARKER %i", marker.id);

    perc_announcer.publish( marker );
    ros::Duration(0.001).sleep();

    marker.id =  marker_id++;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = text;

    marker.color.r = 0.0;
    marker.color.g = 0.5;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    //marker.pose.position.z = pose.getOrigin().z() + 0.05;

    perc_announcer.publish( marker );
    ros::Duration(0.001).sleep();
}


void announce_box(const std::string text, const btVector3 min, const btVector3 max)
{
    if (!perc_init)
    {
        perc_init = true;
        ros::NodeHandle node_handle;
        perc_announcer =  node_handle.advertise<visualization_msgs::Marker>( "/perception_announcer", 0 , true);
    }

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "perception_announcer";
    marker.id =  marker_id++;
    marker.text = text;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.pose.position.x = 0.5 * (max.x() + min.x());
    marker.pose.position.y = 0.5 * (max.y() + min.y());
    marker.pose.position.z = 0.5 * (max.z() + min.z());
    marker.scale.x = max.x() - min.x();
    marker.scale.y = max.y() - min.y();
    marker.scale.z = max.z() - min.z();
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.4;
    marker.color.a = 0.2;

    ROS_INFO("PUBLISH MARKER %i", marker.id);

    perc_announcer.publish( marker );
    ros::Duration(0.001).sleep();

    marker.id =  marker_id++;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = text;

    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.4;
    marker.color.a = 0.2;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.pose.position.z = marker.pose.position.z + 0.05;

    perc_announcer.publish( marker );
    ros::Duration(0.001).sleep();
}

bool cloud_pub_initialized = false;
ros::Publisher cloud_pub;

void pubCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{

    //ROS_INFO("start pub");

    if (!cloud_pub_initialized)
    {
        ros::NodeHandle node_handle;
        cloud_pub = node_handle.advertise<sensor_msgs::PointCloud2>("/debug_cloud",0,true);
        cloud_pub_initialized=true;
    }

    sensor_msgs::PointCloud2 out; //in map frame

    pcl::toROSMsg(*cloud,out);
    out.header.frame_id = "/map";
    cloud_pub.publish(out);

    ROS_INFO("published %i x %i points", out.height, out.width);
    ros::Duration(0.001).sleep();

}



void printPose(const char title[], tf::Stamped<tf::Pose> pose)
{
    ROS_INFO("rosrun tf static_transform_publisher %f %f %f %f %f %f %f %s %s 100", pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z()
             , pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z(), pose.getRotation().w(), pose.frame_id_.c_str(), title);
}

void printPoseSimple(tf::Stamped<tf::Pose> pose)
{
    ROS_INFO("{%f,%f,%f,%f,%f,%f,%f},", pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z(),
             pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z(), pose.getRotation().w());
}


int development_tools(int argc, char** argv)
{


    if (atoi(argv[1]) == -901)
    {
        tf::Stamped<tf::Pose> lid;
        lid.frame_id_ = argv[4];
        lid.setOrigin(btVector3(0 ,0,0));
        lid.setRotation(btQuaternion(0,0,0,1));

        lid = Geometry::getPoseIn(argv[3], lid);

        printf("\ntf::Stamped<tf::Pose> %s;\n",argv[2]);
        printf("%s.frame_id_ = \"%s\";\n",argv[2],argv[3]);
        printf("%s.setOrigin(btVector3(%f, %f, %f));\n",argv[2], lid.getOrigin().x(), lid.getOrigin().y(), lid.getOrigin().z());
        printf("%s.setRotation(btQuaternion(%f, %f, %f, %f));\n",argv[2], lid.getRotation().x(), lid.getRotation().y(), lid.getRotation().z(), lid.getRotation().w());

        printf("\nRobotArm::getInstance(%s)->universal_move_toolframe_ik_pose(%s);\n",argv[5],argv[2]);

        printf("\nbin/ias_drawer_executive -3 %s %f %f %f %f %f %f %f\n", argv[5] ,lid.getOrigin().x(), lid.getOrigin().y(), lid.getOrigin().z(), lid.getRotation().x(), lid.getRotation().y(), lid.getRotation().z(), lid.getRotation().w());
    }


    if (atoi(argv[1]) == -911)
    {
        {
            tf::Stamped<tf::Pose> lid;
            lid.frame_id_ = "r_gripper_tool_frame";
            lid.setOrigin(btVector3(0 ,0,0));
            lid.setRotation(btQuaternion(0,0,0,1));

            lid = Geometry::getPoseIn("/map", lid);

            printf("\ntf::Stamped<tf::Pose> r%s;\n",argv[2]);
            printf("r%s.frame_id_ = \"%s\";\n",argv[2],"/map");
            printf("r%s.setOrigin(btVector3(%f, %f, %f));\n",argv[2], lid.getOrigin().x(), lid.getOrigin().y(), lid.getOrigin().z());
            printf("r%s.setRotation(btQuaternion(%f, %f, %f, %f));\n",argv[2], lid.getRotation().x(), lid.getRotation().y(), lid.getRotation().z(), lid.getRotation().w());

            printf("\nRobotArm::getInstance(0)->universal_move_toolframe_ik_pose(r%s);\n",argv[2]);
        }

        {
            tf::Stamped<tf::Pose> lid;
            lid.frame_id_ = "l_gripper_tool_frame";
            lid.setOrigin(btVector3(0 ,0,0));
            lid.setRotation(btQuaternion(0,0,0,1));

            lid = Geometry::getPoseIn("/map", lid);

            printf("\ntf::Stamped<tf::Pose> l%s;\n",argv[2]);
            printf("l%s.frame_id_ = \"%s\";\n",argv[2],"/map");
            printf("l%s.setOrigin(btVector3(%f, %f, %f));\n",argv[2], lid.getOrigin().x(), lid.getOrigin().y(), lid.getOrigin().z());
            printf("l%s.setRotation(btQuaternion(%f, %f, %f, %f));\n",argv[2], lid.getRotation().x(), lid.getRotation().y(), lid.getRotation().z(), lid.getRotation().w());

            printf("\nRobotArm::getInstance(1)->universal_move_toolframe_ik_pose(l%s);\n",argv[2]);
        }
        printf("\nRobotArm::moveBothArms(l%s,r%s);\n",argv[2],argv[2]);

    }

    if (atoi(argv[1]) == -912)
    {
        {
            tf::Stamped<tf::Pose> lid;
            lid.frame_id_ = "r_gripper_tool_frame";
            lid.setOrigin(btVector3(0 ,0,0));
            lid.setRotation(btQuaternion(0,0,0,1));

            lid = Geometry::getPoseIn("base_link", lid);

            printf("\ntf::Stamped<tf::Pose> r%s;\n",argv[2]);
            printf("r%s.frame_id_ = \"%s\";\n",argv[2],"base_link");
            printf("r%s.setOrigin(btVector3(%f, %f, %f));\n",argv[2], lid.getOrigin().x(), lid.getOrigin().y(), lid.getOrigin().z());
            printf("r%s.setRotation(btQuaternion(%f, %f, %f, %f));\n",argv[2], lid.getRotation().x(), lid.getRotation().y(), lid.getRotation().z(), lid.getRotation().w());

            printf("\nRobotArm::getInstance(0)->universal_move_toolframe_ik_pose(r%s);\n",argv[2]);
        }

        {
            tf::Stamped<tf::Pose> lid;
            lid.frame_id_ = "l_gripper_tool_frame";
            lid.setOrigin(btVector3(0 ,0,0));
            lid.setRotation(btQuaternion(0,0,0,1));

            lid = Geometry::getPoseIn("base_link", lid);

            printf("\ntf::Stamped<tf::Pose> l%s;\n",argv[2]);
            printf("l%s.frame_id_ = \"%s\";\n",argv[2],"base_link");
            printf("l%s.setOrigin(btVector3(%f, %f, %f));\n",argv[2], lid.getOrigin().x(), lid.getOrigin().y(), lid.getOrigin().z());
            printf("l%s.setRotation(btQuaternion(%f, %f, %f, %f));\n",argv[2], lid.getRotation().x(), lid.getRotation().y(), lid.getRotation().z(), lid.getRotation().w());

            printf("\nRobotArm::getInstance(1)->universal_move_toolframe_ik_pose(l%s);\n",argv[2]);
        }
        printf("\nRobotArm::moveBothArms(l%s,r%s);\n",argv[2],argv[2]);

    }


    if (atoi(argv[1]) == -910)
    {
        tf::Stamped<tf::Pose> lid;
        lid.frame_id_ = argv[4];
        lid.setOrigin(btVector3(atof(argv[6]),atof(argv[7]),atof(argv[8])));
        lid.setRotation(btQuaternion(0,0,0,1));

        lid = Geometry::getPoseIn(argv[3], lid);

        printf("\ntf::Stamped<tf::Pose> %s;\n",argv[2]);
        printf("%s.frame_id_ = \"%s\";\n",argv[2],argv[3]);
        printf("%s.setOrigin(btVector3(%f, %f, %f));\n",argv[2], lid.getOrigin().x(), lid.getOrigin().y(), lid.getOrigin().z());
        printf("%s.setRotation(btQuaternion(%f, %f, %f, %f));\n",argv[2], lid.getRotation().x(), lid.getRotation().y(), lid.getRotation().z(), lid.getRotation().w());

        printf("\nRobotArm::getInstance(%s)->universal_move_toolframe_ik_pose(%s);\n",argv[5],argv[2]);

        printf("\nbin/ias_drawer_executive -3 %s %f %f %f %f %f %f %f\n", argv[5] ,lid.getOrigin().x(), lid.getOrigin().y(), lid.getOrigin().z(), lid.getRotation().x(), lid.getRotation().y(), lid.getRotation().z(), lid.getRotation().w());
    }

    if (atoi(argv[1]) == -940)
    {
        tf::Stamped<tf::Pose> lid;
        lid.frame_id_ = "base_link";
        lid.setOrigin(btVector3(0 ,0,0));
        lid.setRotation(btQuaternion(0,0,0,1));

        lid = Geometry::getPoseIn("/map", lid);

        printf("\nRobotDriver::moveBase4(%f, %f, %f, %f);\n",lid.getOrigin().x(), lid.getOrigin().y(), lid.getRotation().z(), lid.getRotation().w());

    }
    return 0;
}

/*
tf::Stamped<tf::Pose> getPotPose()
{

    //if (atoi(argv[1]) == -617)
    {
        ros::NodeHandle node_handle;
        ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>( "/cop_detection", 0 , true);
        ros::Publisher best_pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>( "/cop_detection_best", 0, true );

        std::vector<tf::Stamped<tf::Pose> > poses;
        std::vector<double > votes;

        while (ros::ok())
        {
            tf::Stamped<tf::Pose> pose;
            pose = OperateHandleController::getCopPose("PotWW", "/openni_rgb_optical_frame");
            ros::Duration(0.05).sleep();
            if (pose.frame_id_ == "NO_ID_STAMPED_DEFAULT_CONSTRUCTION")
            {
                ROS_ERROR("dropped %s", pose.frame_id_.c_str());
                continue;
            }

            if (pose.getOrigin().length() < 0.1)
                continue;
            pose = Geometry::getPoseIn("/map", pose);

            geometry_msgs::PoseStamped pose_stamped, best_pose_stamped;

            tf::poseStampedTFToMsg(pose,pose_stamped);
            pose_pub.publish(pose_stamped);
            ros::Duration(0.05).sleep();

            if ((btVector3(-1.836059, 2.612557, 1.016513) - pose.getOrigin()).length() > 0.5)
                continue;

            {
                poses.push_back(pose);
                votes.push_back(.0);

                ROS_INFO("tick");

                if (poses.size() > 1)
                {
                    for (size_t k = 0; k < poses.size(); ++k)
                    {
                        votes[k] = 0;
                    }
                    double max_score = 0;
                    for (size_t k = 0; k < poses.size(); ++k)
                    {
                        for (size_t j = 0; j < poses.size(); ++j)
                        {
                            btTransform a,b,c;
                            a.setOrigin(poses[k].getOrigin());
                            a.setRotation(poses[k].getRotation());
                            b.setOrigin(poses[j].getOrigin());
                            b.setRotation(poses[j].getRotation());
                            c = b.inverseTimes(a);
                            double score = c.getOrigin().length() + c.getRotation().getAngle() * .001;
                            votes[j] += score;
                            votes[k] += score;
                            if (votes[k] > max_score)
                                max_score = votes[k];
                            if (votes[j] > max_score)
                                max_score = votes[j];
                        }
                    }
                    double minscore = 1000000;
                    tf::Stamped<tf::Pose> best;
                    for (size_t k = 0; k < poses.size(); ++k)
                    {
                        double act = votes[k] / max_score;
                        ROS_INFO("%zu score %f: %f", k, votes[k], act);
                        if (act < minscore)
                        {
                            minscore = act;
                            best = poses[k];
                        }
                    }
                    tf::poseStampedTFToMsg(best,best_pose_stamped);
                    ROS_INFO("min score = %f", minscore);
                    best_pose_pub.publish(best_pose_stamped);
                    if (poses.size() > 3)
                        return best;
                    ros::Duration(0.05).sleep();
                }
            }
        }

    }

    tf::Stamped<tf::Pose> t;
    return t;

}
*/


void pubMarker(tf::Stamped<tf::Pose> pose, double radius)
{
    ros::NodeHandle node_handle;
    ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "/robot_arm_marker", 0 );

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "goalpoints";
    marker.id =  10000;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.x = pose.getRotation().x();
    marker.pose.orientation.y = pose.getRotation().y();
    marker.pose.orientation.z = pose.getRotation().z();
    marker.pose.orientation.w = pose.getRotation().w();
    marker.pose.position.x = pose.getOrigin().x();
    marker.pose.position.y = pose.getOrigin().y();
    marker.pose.position.z = pose.getOrigin().z();
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = 0.3;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;

    ROS_INFO("PUBLISH MARKER %i", marker.id);

    ros::Rate rate(10.0);
    int k = 0;
    while (k < 10)
    {
        marker.id = ++marker.id;
        vis_pub.publish( marker );
        ros::spinOnce();
        rate.sleep();
    }
}


/*void getKinectCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, ros::Time after = ros::Time(0,0))
{

    sensor_msgs::PointCloud2 pc;// = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect/cloud_throttled"));
    bool found = false;
    while (!found)
    {
        //pc  = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect/cloud_throttled"));
        pc  = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/rgb/points"));
        if ((after == ros::Time(0,0)) || (pc.header.stamp > after))
            found = true;
        else
        {
            ROS_ERROR("getKinectCloudXYZ cloud too old : stamp %f , target time %f",pc.header.stamp.toSec(), after.toSec());
        }
    }
    tf::Stamped<tf::Pose> net_stamped = RobotArm::getPose("/map","/openni_rgb_optical_frame");
    tf::Transform net_transform;
    net_transform.setOrigin(net_stamped.getOrigin());
    net_transform.setRotation(net_stamped.getRotation());

    sensor_msgs::PointCloud2 pct; //in map frame

    pcl_ros::transformPointCloud("/map",net_transform,pc,pct);
    pct.header.frame_id = "/map";

    pcl::fromROSMsg(pct, *cloud);
}

void getKinectCloudXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, ros::Time after = ros::Time(0,0))
{

    sensor_msgs::PointCloud2 pc;
    bool found = false;
    while (!found)
    {
        //pc  = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect/cloud_throttled"));
        pc  = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/rgb/points"));
        if ((after == ros::Time(0,0)) || (pc.header.stamp > after))
            found = true;
        else
        {
            ROS_ERROR("getKinectCloudXYZ cloud too old : stamp %f , target time %f",pc.header.stamp.toSec(), after.toSec());
        }
    }
    tf::Stamped<tf::Pose> net_stamped = RobotArm::getPose("/map","/openni_rgb_optical_frame");
    tf::Transform net_transform;
    net_transform.setOrigin(net_stamped.getOrigin());
    net_transform.setRotation(net_stamped.getRotation());

    sensor_msgs::PointCloud2 pct; //in map frame

    pcl_ros::transformPointCloud("/map",net_transform,pc,pct);
    pct.header.frame_id = "/map";

    pcl::fromROSMsg(pct, *cloud);
}
*/

void discardKinectFrame()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    Kinect::getInstance()->getCloud(cloud,"/map");
}



void getPointsInBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, btVector3 min, btVector3 max, pcl::PointCloud<pcl::PointXYZRGB>::Ptr inBox)
{

    Eigen::Vector4f min_pt, max_pt;

    min_pt = Eigen::Vector4f(min.x(),min.y(),min.z(), 1);
    max_pt = Eigen::Vector4f(max.x(),max.y(),max.z(), 1);

    //{

    //btVector3 roi_min(min_pt[0],min_pt[1],min_pt[2]);
    //btVector3 roi_max(max_pt[0],max_pt[1],max_pt[2]);

    //announce_box("roi", roi_min, roi_max);

    //}


    ROS_INFO("min %f %f %f" ,min_pt[0],min_pt[1],min_pt[2]);
    ROS_INFO("max %f %f %f" ,max_pt[0],max_pt[1],max_pt[2]);

    ROS_INFO("cloud size : %zu", cloud->points.size());

    boost::shared_ptr<std::vector<int> > indices( new std::vector<int> );
    pcl::getPointsInBox(*cloud,min_pt,max_pt,*indices);

    pcl::ExtractIndices<pcl::PointXYZRGB> ei;
    ei.setInputCloud(cloud);
    ei.setIndices(indices);
    ei.filter(*inBox);

    pubCloud(inBox);

    ROS_INFO("cloud size after box filtering: %zu", inBox->points.size());
}


//get circle2d in xy plane within searchspace min->max, takin pointcloud on its own
bool getCircle(const btVector3 min, const btVector3 max, btVector3 &center, double &radius)
{

    ros::NodeHandle node_handle;
    ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>( "/cop_detection", 0 , true);
    ros::Publisher pose_pubr = node_handle.advertise<geometry_msgs::PoseStamped>( "/cop_detectionr", 0 , true);
    ros::Publisher pose_publ = node_handle.advertise<geometry_msgs::PoseStamped>( "/cop_detectionl", 0 , true);

    sensor_msgs::PointCloud2 pc = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect/cloud_throttled"));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pot_cyl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    tf::Stamped<tf::Pose> net_stamped = Geometry::getPose("/map","/openni_rgb_optical_frame");
    tf::Transform net_transform;
    net_transform.setOrigin(net_stamped.getOrigin());
    net_transform.setRotation(net_stamped.getRotation());

    sensor_msgs::PointCloud2 pct; //in map frame

    pcl_ros::transformPointCloud("/map",net_transform,pc,pct);
    pct.header.frame_id = "/map";

    pcl::fromROSMsg(pct, *cloud);

    Eigen::Vector4f min_pt, max_pt;

    min_pt = Eigen::Vector4f(min.x(),min.y(),min.z(), 1);
    max_pt = Eigen::Vector4f(max.x(),max.y(),max.z(), 1);
    {
        tf::Stamped<tf::Pose> res;
        res.setOrigin(min);
        res.setRotation(btQuaternion(0,0,0,1));
        res.frame_id_ = "/map";

        geometry_msgs::PoseStamped res_msg;
        tf::poseStampedTFToMsg(res,res_msg);

        pose_pubr.publish(res_msg);
    }

    {
        tf::Stamped<tf::Pose> res;
        res.setOrigin(max);
        res.setRotation(btQuaternion(0,0,0,1));
        res.frame_id_ = "/map";

        geometry_msgs::PoseStamped res_msg;
        tf::poseStampedTFToMsg(res,res_msg);

        pose_publ.publish(res_msg);
    }



    ROS_INFO("min %f %f %f" ,min.x(),min.y(),min.z());
    ROS_INFO("max %f %f %f" ,max.x(),max.y(),max.z());

    ROS_INFO("cloud size : %zu", cloud->points.size());

    boost::shared_ptr<std::vector<int> > indices( new std::vector<int> );
    pcl::getPointsInBox(*cloud,min_pt,max_pt,*indices);

    pcl::ExtractIndices<pcl::PointXYZ> ei;
    ei.setInputCloud(cloud);
    ei.setIndices(indices);
    ei.filter(*pot_cyl_cloud);

    ROS_INFO("cloud size : %zu", pot_cyl_cloud->points.size());

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_CIRCLE2D);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (pot_cyl_cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return false;
    }
    else
    {
        ROS_INFO("MODEL :");
        //coefficients.get()
        ROS_INFO("%f %f %f %f", coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);
        center = btVector3(coefficients->values[0], coefficients->values[1],0.5*(min.z() + max.z()));
        radius = coefficients->values[2];


        tf::Stamped<tf::Pose> res;
        res.setOrigin(center);
        res.setRotation(btQuaternion(0,0,0,1));
        res.frame_id_ = "/map";

        geometry_msgs::PoseStamped res_msg;
        tf::poseStampedTFToMsg(res,res_msg);

        pose_pub.publish(res_msg);

        //ros::Duration(1.0).sleep();

        return true;
    }
}

//!get circle2d in xy plane within searchspace min->max, from pointcloud in parameter
bool getCircleFromCloud(pcl::PointCloud<pcl::PointXYZRGB> &pot_cyl_cloud, btVector3 &center, double &radius)
{

    ros::NodeHandle node_handle;
    ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>( "/cop_detection", 0 , true);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_CIRCLE2D);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (pot_cyl_cloud.makeShared());
    seg.segment (*inliers, *coefficients);

    seg.setRadiusLimits(radius - .01, radius + .01);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return false;
    }
    else
    {

        double z = 0;
        for (size_t k=0; k < inliers->indices.size(); ++k)
        {
            z += pot_cyl_cloud.points[inliers->indices[k]].z;
        }
        z /=  inliers->indices.size();
        ROS_INFO("MODEL :");
        //coefficients.get()
        ROS_INFO("%f %f %f %f SIZE : %zu", coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3], inliers->indices.size());
        center = btVector3(coefficients->values[0], coefficients->values[1],z);
        radius = coefficients->values[2];

        tf::Stamped<tf::Pose> res;
        res.setOrigin(center);
        res.setRotation(btQuaternion(0,0,0,1));
        res.frame_id_ = "/map";

        geometry_msgs::PoseStamped res_msg;
        tf::poseStampedTFToMsg(res,res_msg);

        pose_pub.publish(res_msg);

        //ros::Duration(1.0).sleep();

        return true;
    }
}




bool getCirclesFromCloud(pcl::PointCloud<pcl::PointXYZRGB> &pot_cyl_cloud, double radius_goal, double radius_tolerance,
                         std::vector<btVector3> &center,
                         std::vector<double> &radius,
                         std::vector<int> &numinliers,
                         size_t  iterations = 5)
{


    ros::NodeHandle node_handle;

    //ros::Publisher cloud_pub = node_handle.advertise<sensor_msgs::PointCloud2>("/debug_cloud",0,true);

    center.clear();
    radius.clear();
    numinliers.clear();

    center.resize(iterations);
    radius.resize(iterations);
    numinliers.resize(iterations);

    pcl::PointCloud<pcl::PointXYZRGB> act = pot_cyl_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> flip;

    for (size_t k = 0; k < iterations; k++)
    {

        //sensor_msgs::PointCloud2 out; //in map frame
        //pcl::toROSMsg(act,out);
        //out.header.frame_id = "/map";
        //cloud_pub.publish(out);
        pubCloud(act.makeShared());

        ros::NodeHandle node_handle;
        ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>( "/cop_detection", 0 , true);

        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_CIRCLE2D);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.01);

        seg.setInputCloud (act.makeShared());
        seg.segment (*inliers, *coefficients);

        seg.setRadiusLimits(radius_goal - radius_tolerance, radius_goal + radius_tolerance);

        if (inliers->indices.size () == 0)
        {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            return false;
        }
        else
        {
            double z = 0;
            for (size_t g=0; g < inliers->indices.size(); ++g)
            {
                z += act.points[inliers->indices[g]].z;
            }
            z /=  inliers->indices.size();
            ROS_INFO("MODEL :");
            //coefficients.get()
            ROS_INFO("%f %f %f %f SIZE : %zu", coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3], inliers->indices.size());
            center[k] = btVector3(coefficients->values[0], coefficients->values[1],z);
            radius[k] = coefficients->values[2];
            numinliers[k] = inliers->indices.size();

            tf::Stamped<tf::Pose> res;
            res.setOrigin(center[k]);
            res.setRotation(btQuaternion(0,0,0,1));
            res.frame_id_ = "/map";

            geometry_msgs::PoseStamped res_msg;
            tf::poseStampedTFToMsg(res,res_msg);

            pose_pub.publish(res_msg);

        }
        flip = act;
        pcl::ExtractIndices<pcl::PointXYZRGB> ei;
        ei.setInputCloud(flip.makeShared());
        ei.setIndices(inliers);
        ei.setNegative(true);
        ei.filter(act);

        ROS_INFO("BLIP");
        //ros::Duration(1).sleep();
    }
    return true;
}

bool getLidPose(tf::Stamped<tf::Pose> &lid, btVector3 min = btVector3(-2.134, 2.625,0.955), btVector3 max = btVector3(-1.663, 3.215,0.972))
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    Kinect::getInstance()->getCloud(cloud);

    //btVector3 min(-2.134, 2.625,0.953);
    //btVector3 max(-1.663, 3.215,0.972);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inBox (new pcl::PointCloud<pcl::PointXYZRGB>);

    getPointsInBox(cloud,min,max,inBox);

    pubCloud(inBox);

    std::vector<btVector3> center;
    std::vector<double> radius;
    std::vector<int> numinliers;

    getCirclesFromCloud(*inBox, .2, .04,center,radius,numinliers,1);

    if (numinliers[0] < 300)
        return false;

    lid.setOrigin(center[0]);
    lid.setRotation(btQuaternion(0,0,0,1));
    lid.frame_id_ = "/map";

    lid = Geometry::getPoseIn("base_link", lid);
    lid.setRotation(btQuaternion(0,0,1,0));
    lid = Geometry::getPoseIn("map", lid);

    announce("Lid", lid);

    return true;
}


void calcPotGraspPoints(tf::Stamped<tf::Pose> det, tf::Stamped<tf::Pose> &leftpreg, tf::Stamped<tf::Pose> &leftg, tf::Stamped<tf::Pose> &rightpreg, tf::Stamped<tf::Pose> &rightg)
{
    btTransform potww;
    potww.setOrigin(det.getOrigin());
    potww.setRotation(det.getRotation());
    btTransform leftrel, rightrel;
    btTransform leftgrasp, rightgrasp;

    rightrel.setOrigin(btVector3(0.136, -0.012, -0.014));
    rightrel.setRotation(btQuaternion(-0.179, -0.684, 0.685, -0.175));

    leftrel.setOrigin(btVector3(-0.130, -0.008, -0.013));
    leftrel.setRotation(btQuaternion(-0.677, 0.116, 0.168, 0.707));

    btTransform pregrasp(btQuaternion(0,0,0,1), btVector3(-.06,0,0));
    btTransform leftpregrasp,rightpregrasp;

    rightpregrasp = potww * rightrel * pregrasp;
    leftpregrasp = potww * leftrel * pregrasp;

    rightgrasp = potww * rightrel;
    leftgrasp = potww * leftrel;

    leftg = det;
    rightg = det;
    leftg.frame_id_ = "/map";
    rightg.frame_id_ = "/map";

    leftg.setOrigin(leftgrasp.getOrigin());
    leftg.setRotation(leftgrasp.getRotation());
    rightg.setOrigin(rightgrasp.getOrigin());
    rightg.setRotation(rightgrasp.getRotation());

    leftpreg.frame_id_ = "/map";
    rightpreg.frame_id_ = "/map";
    leftpreg.setOrigin(leftpregrasp.getOrigin());
    leftpreg.setRotation(leftpregrasp.getRotation());
    rightpreg.setOrigin(rightpregrasp.getOrigin());
    rightpreg.setRotation(rightpregrasp.getRotation());
}


//! pot detector based on pcl
bool getPotPose(tf::Stamped<tf::Pose> &potPose, tf::Stamped<tf::Pose> &leftpreg, tf::Stamped<tf::Pose> &leftg, tf::Stamped<tf::Pose> &rightpreg, tf::Stamped<tf::Pose> &rightg, bool via_lid=true)
{

    ros::Time query_time = ros::Time::now();

    double table_height = 0.865;
    double pot_cyl_min = 0.02;
    double pot_cyl_max = 0.05;
    double pot_handle_min = 0.06;
    double pot_handle_max = 0.09;
    double pot_radius = 0.20 / 2.0;
    double pot_handle_radius_max = 0.28 / 2.0;
    double pot_handle_radius_min = 0.25 / 2.0;
    //-1.77 3.09
    //-2.06 2.41
    //btVector3 roi_min(-2.134, 2.625, table_height + pot_cyl_min);
    //btVector3 roi_max(-1.663, 3.215, table_height + pot_cyl_max);
    btVector3 roi_min(-2.06, 2.41, table_height + pot_cyl_min);
    btVector3 roi_max(-1.7, 3.09, table_height + pot_cyl_max);

    announce_box("pot search", roi_min, roi_max);
    announce_box("pot search", roi_min, roi_max);

    ros::NodeHandle node_handle;

    ros::Publisher cloud_pub = node_handle.advertise<sensor_msgs::PointCloud2>("/debug_cloud",0,true);
    ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>( "/cop_detection", 0 , true);
    ros::Publisher pose_pubr = node_handle.advertise<geometry_msgs::PoseStamped>( "/cop_detectionr", 0 , true);
    ros::Publisher pose_publ = node_handle.advertise<geometry_msgs::PoseStamped>( "/cop_detectionl", 0 , true);


    //  sensor_msgs::PointCloud2 pc = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect/cloud_throttled"));

    //tf::Stamped<tf::Pose> net_stamped = RobotArm::getPose("/map","/openni_rgb_optical_frame");
    //tf::Transform net_transform;
    //net_transform.setOrigin(net_stamped.getOrigin());
    //net_transform.setRotation(net_stamped.getRotation());

    //sensor_msgs::PointCloud2 pct; //in map frame

    //pcl_ros::transformPointCloud("/map",net_transform,pc,pct);
    //pct.header.frame_id = "/map";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    Kinect::getInstance()->getCloud(cloud, "/map", query_time);
    //pcl::fromROSMsg(pct, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Vector4f  	min_pt, max_pt;

    // find the pot center
    btVector3 center;


    double radius;
    if (!via_lid)
    {
        //! find circle from pot base cylinder

        pcl::PointCloud<pcl::PointXYZ>::Ptr pot_cyl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Vector4f min_pt, max_pt;

        //min_pt = Eigen::Vector4f(-2.134, 2.625, 0.95 - 0.03, 0);
        //max_pt = Eigen::Vector4f(-1.663, 3.215, 0.95 + 0.03, 0);

        min_pt = Eigen::Vector4f(roi_min.x(), roi_min.y(), roi_min.z(),1);
        max_pt = Eigen::Vector4f(roi_max.x(), roi_max.y(), roi_max.z(),1);

        boost::shared_ptr<std::vector<int> > indices( new std::vector<int> );
        pcl::getPointsInBox(*cloud,min_pt,max_pt,*indices);
        pcl::ExtractIndices<pcl::PointXYZ> ei;
        ei.setInputCloud(cloud);
        ei.setIndices(indices);

        ei.filter(*pot_cyl_cloud);

        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_CIRCLE2D);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.01);

        seg.setInputCloud (pot_cyl_cloud);

        seg.setProbability(0.9999);
        seg.setOptimizeCoefficients(true);

        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            return false;
        }
        else
        {
            ROS_INFO("MODEL :");
            //coefficients.get()
            ROS_INFO("%f %f %f %f", coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);
            center = btVector3(coefficients->values[0], coefficients->values[1],0.96);
            radius = coefficients->values[2];
        }
    }

    //min_pt = Eigen::Vector4f(center.x(),center.y(),1,1) - Eigen::Vector4f(.25,.25,.0, 0);
    //max_pt = Eigen::Vector4f(center.x(),center.y(),1,1) + Eigen::Vector4f(.25,.25,.1, 0);

    if (via_lid)
    {
        tf::Stamped<tf::Pose> lidPose;
        bool got_lid_pose = false;
        while (!got_lid_pose)
            got_lid_pose = getLidPose(lidPose);
        center = lidPose.getOrigin();
    }
    ROS_INFO("CENTER %f %f %f",center.x(),center.y(),center.z());


    min_pt = Eigen::Vector4f(center.x(),center.y(),table_height,1) - Eigen::Vector4f(pot_handle_radius_max,pot_handle_radius_max,-pot_handle_min, 0);
    max_pt = Eigen::Vector4f(center.x(),center.y(),table_height,1) + Eigen::Vector4f(pot_handle_radius_max,pot_handle_radius_max,pot_handle_max, 0);

    boost::shared_ptr<std::vector<int> > indices( new std::vector<int> );
    pcl::getPointsInBox(*cloud,min_pt,max_pt,*indices);
    pcl::ExtractIndices<pcl::PointXYZ> ei;
    ei.setInputCloud(cloud);
    ei.setIndices(indices);
    ei.filter(*cloud_filtered);

    indices->clear();
    double perc = 0;

    ROS_INFO("cloud filtered size : %zu", cloud_filtered->points.size());

    if (cloud_filtered->points.size() < 10)
        return false;

    ROS_INFO("center %f %f %f", center.x(),center.y(),center.z());

    //double pot_radius;

    //ros::param::param<double>("pot_radius", pot_radius, .155);

    ROS_INFO("pot_radius=%f", pot_radius);

    for (size_t k = 0; k < cloud_filtered->points.size(); ++k)
    {
        btVector3 act(cloud_filtered->points[k].x,cloud_filtered->points[k].y,cloud_filtered->points[k].z);
        btVector3 rel = center - act;
        rel.setZ(0);
        double radius = rel.length();
        //ROS_INFO("radius : %f", radius);

        //if (((pot_radius > 0) && (radius > pot_radius)) || ((pot_radius < 0) && (radius < fabs(pot_radius))))
        //if ((radius > .155) && (radius < .1925))
        if ((radius > pot_handle_radius_min) && (radius < pot_handle_radius_max))
        {
            indices->push_back(k);
            perc+=1.0;
        }
    }

    ROS_INFO("Percentage of points : %f/%zu= %f",perc,cloud_filtered->points.size(), perc / cloud_filtered->points.size());

    if (cloud_filtered->points.size() < 150)
    {
        ROS_ERROR("did not get enough pot handle candidates");
        return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
    ei.setInputCloud(cloud_filtered);
    ei.setIndices(indices);
    ei.setIndices(indices);
    ei.filter(*cloud_filtered2);

    double atansum = 0;
    double atannum = 0;
    for (size_t k = 0; k < cloud_filtered2->points.size(); ++k)
    {
        //y/x, i
        btVector3 act(cloud_filtered2->points[k].x- center.x(),cloud_filtered2->points[k].y - center.y(),0);
        if (act.x() != 0.0)
        {
            double at = atan2(act.y(),act.x());
            //ROS_INFO("atan %f %f = %f", act.x(), act.y(), at);
            if (at < 0)
                at += M_PI;
            //double at = atan(act.x()/act.y());
            atansum += at;
            atannum += 1.0;
        }
    }
    ROS_INFO("atansum %f num %f = %f", atansum, atannum, atansum/atannum);


    if (atansum < 20)
    {
        ROS_ERROR("did not get enough pot handle inliers");
        return false;
    }



    tf::Stamped<tf::Pose> det;
    det.frame_id_ = "/map";
    det.setOrigin(center);
    atansum /= atannum;
    //if (atansum < 0)
//                  atansum += M_PI;
    det.setRotation(btQuaternion(btVector3(0,0,1),atansum));

    potPose = det;

    ROS_INFO("angle : %f", det.getRotation().getAngle());

    //if (0)
    {
        sensor_msgs::PointCloud2 filtered_msg;
        pcl::toROSMsg(*cloud_filtered2,filtered_msg);
        cloud_pub.publish(filtered_msg);
    }

    /// calc grasp poses
    calcPotGraspPoints(det, leftpreg, leftg, rightpreg, rightg);

    {
        geometry_msgs::PoseStamped det_msg;
        tf::poseStampedTFToMsg(det,det_msg);
        pose_pub.publish(det_msg);

        geometry_msgs::PoseStamped rightg_msg, leftg_msg;
        tf::poseStampedTFToMsg(rightg,rightg_msg);
        tf::poseStampedTFToMsg(leftg,leftg_msg);

        pose_pubr.publish(rightg_msg);
        pose_publ.publish(leftg_msg);

        announce("Pot", potPose);

        printPose("pot", potPose);

        announce("Pot_L", leftg);

        announce("Pot_R", rightg);
    }

    //pubMarker(det,radius);

    ROS_INFO("XXXXXXXXXXXXXXXXXXXXXXXXXX");

    return true;
}



void findBoundary1(pcl::PointCloud<pcl::PointXYZRGB>& in,pcl::PointCloud<pcl::PointXYZRGB>& edges, pcl::PointCloud<pcl::PointXYZRGB>&other, std::vector<int>& edge_indices )
{

    pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr knntree (new pcl::KdTreeFLANN<pcl::PointXYZRGB>  () );
    pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree2 (new pcl::KdTreeFLANN<pcl::PointXYZRGB>  () );
    knntree->setInputCloud(in.makeShared());

    pcl::PointCloud<pcl::Normal> normals;
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setSearchMethod (tree2);
    ne.setKSearch (20);
    ne.setInputCloud (in.makeShared());
    ne.compute (normals);

//        getNormals(in, normals);          printf("normals.size     %d\n", normals.points.size());

    pcl::BoundaryEstimation<pcl::PointXYZRGB ,pcl::Normal, pcl::Boundary> be;
    be.setInputCloud(in.makeShared());
    be.setInputNormals(normals.makeShared());
    Eigen::Vector3f u = Eigen::Vector3f::Zero ();
    Eigen::Vector3f v = Eigen::Vector3f::Zero ();
    //be.compute(
    for (size_t idx = 0; idx < in.points.size(); idx++) //            printf("it %d\n", idx);
    {
        be.getCoordinateSystemOnPlane(normals.points[idx], u, v);
        pcl::Vector3fMap n4uv = normals.points[idx].getNormalVector3fMap();//               std::cout<<" n4uv "<<std::endl<<n4uv<<std::endl;

        std::vector<int> nb_idxs;
        std::vector<float> nb_dists;
        //               knntree->nearestKSearch(idx, 100, nb_idxs, nb_dists);
        knntree->radiusSearch(idx,0.01, nb_idxs, nb_dists, 100);

        //               pcl::Vector3fMap n4uv =
        normals.points[idx].getNormalVector3fMap ();
        bool pt = false;
        pt = be.isBoundaryPoint (in, idx, nb_idxs, u, v, M_PI * 0.75);//
        //printf("it %zu\n", idx);
        if (pt)
        {
            edges.points.push_back(in.points[idx]);
            edge_indices.push_back(idx);
        }
        else
        {
            other.points.push_back(in.points[idx]);
        }

    }
}

void findBoundary2(pcl::PointCloud<pcl::PointXYZRGB>& in,pcl::PointCloud<pcl::PointXYZRGB>& edges, pcl::PointCloud<pcl::PointXYZRGB>&other, std::vector<int>& edge_indices )
{

    pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr knntree (new pcl::KdTreeFLANN<pcl::PointXYZRGB>  () );
    pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree2 (new pcl::KdTreeFLANN<pcl::PointXYZRGB>  () );

    ROS_INFO("before knntree");
    knntree->setInputCloud(in.makeShared());
    ROS_INFO("before knntree");

    pcl::PointCloud<pcl::Normal> normals;
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setSearchMethod (tree2);
    ne.setKSearch (50);
    ne.setInputCloud (in.makeShared());
    ROS_INFO("before normals");
    ne.compute (normals);
    ROS_INFO("before normals");

//        getNormals(in, normals);          printf("normals.size     %d\n", normals.points.size());

    pcl::BoundaryEstimation<pcl::PointXYZRGB ,pcl::Normal, pcl::Boundary> be;
    be.setInputCloud(in.makeShared());
    be.setInputNormals(normals.makeShared());
    Eigen::Vector3f u = Eigen::Vector3f::Zero ();
    Eigen::Vector3f v = Eigen::Vector3f::Zero ();
    pcl::PointCloud<pcl::Boundary> boundary;

    be.setSearchMethod(tree2);
    be.setKSearch (50);

    ROS_INFO("before boundary");
    be.compute(boundary);
    ROS_INFO("after boundary");

    for (size_t idx = 0; idx < boundary.points.size(); idx++) //            printf("it %d\n", idx);
    {
        if (boundary.points[idx].boundary_point)
        {
            edges.points.push_back(in.points[idx]);
            edge_indices.push_back(idx);
        }
        else
        {
            other.points.push_back(in.points[idx]);
        }

    }
}

int Current::manipulateKnob(int direction)
{

    btVector3 correction(0.006,0,0);

    tf::Stamped<tf::Pose> knob_pre_grasp;
    knob_pre_grasp.frame_id_ = "/map";
    knob_pre_grasp.setOrigin(btVector3(0.815572 - .1, 2.204640, 1.408645));
    knob_pre_grasp.setRotation(btQuaternion(-0.005306, -0.020663, 0.023385, 0.999499));

    tf::Stamped<tf::Pose> knob_grasp;
    knob_grasp.frame_id_ = "/map";
    knob_grasp.setOrigin(btVector3(0.815572 + correction.x(), 2.204640, 1.408645));
    knob_grasp.setRotation(btQuaternion(-0.005306, -0.020663, 0.023385, 0.999499));


    tf::Stamped<tf::Pose> k3;
    k3.frame_id_ = "/map";
    k3.setOrigin(btVector3(0.811484 + correction.x(), 2.207182, 1.416687));
    k3.setRotation(btQuaternion(0.978148, 0.024668, 0.015891, -0.205827));

    tf::Stamped<tf::Pose> k3post;
    k3post.frame_id_ = "/map";
    k3post.setOrigin(btVector3(0.811484 - .1, 2.207182, 1.416687));
    k3post.setRotation(btQuaternion(0.978148, 0.024668, 0.015891, -0.205827));

//RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(k3);

    //- Translation: [0.818, 2.214, 1.413]
    //- Rotation: in Quaternion [0.988, 0.026, -0.029, 0.148]

    tf::Stamped<tf::Pose> knob_rot;
    knob_rot.frame_id_ = "/map";
    knob_rot.setOrigin(btVector3(0.818 + correction.x(), 2.214, 1.413));
    knob_rot.setRotation(btQuaternion(0.988, 0.026, -0.029, 0.148));
    //knob_rot.setOrigin(btVector3(0.811130, 2.201299, 1.409015));
    //knob_rot.setRotation(btQuaternion(0.431538, -0.021031, 0.064140, 0.899566));

    tf::Stamped<tf::Pose> knob_post_rot;
    knob_post_rot.frame_id_ = "/map";
    knob_post_rot.setOrigin(btVector3(0.818 - .1, 2.214, 1.413));
    knob_post_rot.setRotation(btQuaternion(0.988, 0.026, -0.029, 0.148));
    //knob_post_rot.setOrigin(btVector3(0.811130 - .1, 2.201299, 1.409015));
    //knob_post_rot.setRotation(btQuaternion(0.431538, -0.021031, 0.064140, 0.899566));

    knob_post_rot = k3post;
    knob_rot = k3;

    RobotHead::getInstance()->lookAtThreaded("/map",0.815572, 2.204640, 1.408645);

    OperateHandleController::plateAttackPose();

    RobotDriver::moveBase4(0.206569, 2.269677, 0.021550, 0.999766);

    announce("Knob", knob_grasp);


    if (direction == 1)
    {

        bool got_it = false;
        while (!got_it)
        {
            RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(knob_pre_grasp);
            Gripper::getInstance(0)->open(.056108);
            RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(knob_grasp);
            Gripper::getInstance(0)->close();
            got_it = Gripper::getInstance(0)->getAmountOpen() > 0.016;

            ROS_INFO("manipulateKnob %f",Gripper::getInstance(0)->getAmountOpen());
        }


        RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(knob_rot);
        Gripper::getInstance(0)->open(.056108);
        RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(knob_post_rot);
    }
    if (direction == -1)
    {

        bool got_it = false;
        while (!got_it)
        {
            Gripper::getInstance(0)->open(.056108);
            RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(knob_post_rot);
            RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(knob_rot);
            Gripper::getInstance(0)->close();
            got_it = Gripper::getInstance(0)->getAmountOpen() > 0.016;

            ROS_INFO("manipulateKnob %f",Gripper::getInstance(0)->getAmountOpen());
        }

        RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(knob_grasp);
        Gripper::getInstance(0)->open(.056108);
        RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(knob_pre_grasp);

    }
    return 0;
}

int Current::getPotOut_openDrawer(int jump)
{

    Torso *torso = Torso::getInstance();

    double lift = 0.29;

    RobotHead::getInstance()->lookAtThreaded("/map",-1.636, 2.664, 0.354);

    boost::thread t(&Torso::pos, torso, .29);

    boost::thread t1(&OperateHandleController::plateTuckPose);

    RobotDriver::moveBase4(-1.011, 2.110,  0.999, -0.033);

    t1.join();
    t.join();

    announce("Handle",Geometry::make_pose(-1.636, 2.664, 0.354 + lift, -0.023, 0.721, 0.691, -0.054,"/map"));

    RobotHead::getInstance()->lookAtThreaded("/r_gripper_tool_frame",0,0,0);

    // open drawer
    Gripper::getInstance(0)->openThreaded();
    RobotArm::getInstance(0)->universal_move_toolframe_ik(-1.536, 2.664, 0.354 + lift, -0.023, 0.721, 0.691, -0.054, "/map");
    RobotArm::getInstance(0)->universal_move_toolframe_ik(-1.636, 2.664, 0.354 + lift, -0.023, 0.721, 0.691, -0.054,"/map");
    Gripper::getInstance(0)->close();

    ROS_ERROR("getPotOut_openDrawerAMOUNT OPEN: %f", Gripper::getInstance(1)->getAmountOpen());

    ros::Duration(0.5).sleep();

    if (Gripper::getInstance(0)->getAmountOpen() < 0.01)
    {
        RobotArm::getInstance(0)->universal_move_toolframe_ik(-1.536, 2.664, 0.354 + lift, -0.023, 0.721, 0.691, -0.054, "/map");
        OperateHandleController::plateTuckPoseRight();
        return Current::getPotOut_openDrawer(jump);
    }

    RobotArm::getInstance(0)->universal_move_toolframe_ik(-1.159, 2.680, 0.348 + lift, -0.043, 0.708, 0.705, -0.004,"/map");
    Gripper::getInstance(0)->openThreaded();

    //! drawer should be open now

    //RobotHead::getInstance()->lookAtThreaded("/map",-1.373, 2.951, 0.70);
    RobotHead::getInstance()->lookAtThreaded("/map",-1.636, 2.664, 0.354);


    RobotDriver::moveBase4(-0.756, 2.093,1,0);

    return 0;
}


int Current::getPotOut_pickPlacePot(int jump)
{

    double modifier = -.06;

    RobotHead::getInstance()->lookAtThreaded("/map",-1.636, 2.664, 0.354);


    boost::thread t2(&OperateHandleController::plateTuckPose);

    Torso *torso = Torso::getInstance();

    double lift = 0.29;

    boost::thread tt(&Torso::pos, torso,  + lift);
    tt.join();

    lift += modifier;

    Gripper::getInstance(0)->closeThreaded();
    Gripper::getInstance(1)->closeThreaded();
    RobotDriver::moveBase4(-0.812520, 2.580165, 0.999515, -0.031016);

    tf::Stamped<tf::Pose> ravoid, rappr, lappr, rpick, lpick, rplace, lplace, rplaceup, lplaceup;

    rpick.frame_id_ = "/map";
    rpick.setOrigin(btVector3(-1.373, 2.951, 0.280  + lift));
    rpick.setRotation(btQuaternion(-0.661, 0.035, 0.748, 0.056));
    rappr = rpick;
    rappr.setOrigin(rappr.getOrigin() + btVector3(0,0,0.3));

    lpick.frame_id_ = "/map";
    lpick.setOrigin(btVector3(-1.403, 2.711, 0.290  + lift));
    lpick.setRotation(btQuaternion(-0.627, 0.039, 0.778, -0.019));
    lappr = lpick;
    lappr.setOrigin(lappr.getOrigin() + btVector3(0,0,0.3));


    //tf::Stamped<tf::Pose> rpick;
    rpick.frame_id_ = "/map";
    rpick.setOrigin(btVector3(-1.378332, 2.929956, 0.513234));
    rpick.setRotation(btQuaternion(-0.659041, 0.029168, 0.750170, 0.045374));

//RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(rpick);

//tf::Stamped<tf::Pose> lpick;
    lpick.frame_id_ = "/map";
    lpick.setOrigin(btVector3(-1.399699, 2.720151, 0.52270));
    lpick.setRotation(btQuaternion(-0.625505, 0.034901, 0.779373, -0.010184));

//RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(rpick);

//RobotArm::moveBothArms(lpick,rpick);


    ravoid = rpick;
    ravoid.setOrigin(btVector3(-1.223, 2.960, 0.593  + lift));
    ravoid.setRotation(btQuaternion(-0.137, 0.840, -0.069, 0.521));

    tf::Stamped<tf::Pose> pot = lpick;
    pot.setOrigin((lpick.getOrigin() + rpick.getOrigin()) * 0.5);

    announce("Pot", pot);

    //OperateHandleController::closeGrippers(false);

    RobotHead::getInstance()->lookAtThreaded("/r_gripper_tool_frame",0,0,0);

    RobotArm::getInstance()->startTrajectory(RobotArm::getInstance(0)->goalTraj(-0.91654420464731023, 0.55680337139226421, -1.0197552147091691, -1.6391094508469026, -33.205223487262245, -1.5794848996581, 26.217879010657214),true);

    RobotArm::moveBothArms(lappr, rappr);

    //RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(rappr);

    OperateHandleController::openGrippers(true);

    RobotArm::moveBothArms(lpick, rpick);

    OperateHandleController::closeGrippers();

    //exit(0);

    boost::thread t3(&Torso::up, torso);

    OperateHandleController::spinnerL(0,0,0.5);

    RobotHead::getInstance()->lookAtThreaded("/map",-1.866, 3.027, 1);


    t3.join();

    //OperateHandleController::spinnerL(0,0,0.1);

    //RobotDriver::moveBase4(-1.235, 2.64,1,-0.029); // this was the one used

    rplace.frame_id_ = "/map";
    //rplace.setOrigin(btVector3(-1.866, 3.027, 1));
    //rplace.setRotation(btQuaternion(-0.652, 0.052, 0.729, 0.200));
    //rplace.setOrigin(btVector3(-1.796 - 0.02, 2.993, 0.997 + modifier));
    rplace.setOrigin(btVector3(-1.796 - 0.02, 2.993 + 0.02, 0.997 + modifier));
    rplace.setRotation(btQuaternion(-0.582, 0.259, 0.7019, 0.317));
    rplaceup = rplace;
    rplaceup.setOrigin(rplaceup.getOrigin() + btVector3(0,0,0.1));

    lplace.frame_id_ = "/map";
    //lplace.setOrigin(btVector3(-1.920, 2.802, 1));
    //lplace.setRotation(btQuaternion(-0.639, 0.028, 0.767, 0.052));
    //lplace.setOrigin(btVector3(-1.946 - 0.02 , 2.817, 1.010 + modifier));
    lplace.setOrigin(btVector3(-1.946 - 0.02, 2.817 + 0.02, 1.010 + modifier));
    lplace.setRotation(btQuaternion(-0.629, 0.166, 0.731, 0.208));
    lplaceup = lplace;
    lplaceup.setOrigin(lplaceup.getOrigin() + btVector3(0,0,0.1));

    // TODO findbasemove
    {
        tf::Stamped<tf::Pose> result;
        std::vector<int> arm;
        std::vector<tf::Stamped<tf::Pose> > goal;
        arm.push_back(1);
        arm.push_back(1);
        arm.push_back(0);
        arm.push_back(0);
        goal.push_back(lplace);
        goal.push_back(lplaceup);
        goal.push_back(rplace);
        goal.push_back(rplaceup);
        //bool RobotArm::findBaseMovement(tf::Stamped<tf::Pose> &result, std::vector<int> arm, std::vector<tf::Stamped<tf::Pose> > goal, bool drive, bool reach)

        RobotArm::findBaseMovement(result, arm, goal, true, false);
    }


    RobotArm::moveBothArms(lplaceup,rplaceup);

    RobotArm::moveBothArms(lplace,rplace);

    OperateHandleController::openGrippers();

    RobotArm::moveBothArms(lplaceup,rplaceup);


    RobotDriver::moveBase4(-0.868230, 2.383544, 0.997340, 0.072885);

    OperateHandleController::plateAttackPose();

    OperateHandleController::plateTuckPose();

    tf::Stamped<tf::Pose> predrawerclose;
    predrawerclose.frame_id_ = "/map";
    predrawerclose.setOrigin(btVector3(-1.420218, 2.811923, 0.620214));
    predrawerclose.setRotation(btQuaternion(0.027181, -0.678301, 0.732514, -0.050909));

    RobotHead::getInstance()->lookAtThreaded("/r_gripper_tool_frame",0,0,0);


    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(predrawerclose);

    tf::Stamped<tf::Pose> postdrawerclose;
    postdrawerclose.frame_id_ = "/map";
    //postdrawerclose.setOrigin(btVector3(-1.660690, 2.574179, 0.667216));
    postdrawerclose.setOrigin(btVector3(-1.610690, 2.574179, 0.667216));
    postdrawerclose.setRotation(btQuaternion(0.018551, -0.684924, 0.724965, -0.070439));

    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(postdrawerclose);

    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(predrawerclose);

    OperateHandleController::plateTuckPose();

    return 0;
}

void getPotOut(int jump)
{
    RobotHead::getInstance()->lookAtThreaded("/r_gripper_tool_frame",0,0,0);

    switch (jump)
    {
    case 0:
        Current::getPotOut_openDrawer(jump);
    case 1:
        Current::getPotOut_pickPlacePot(jump);
    }
}

int Current::openDrawerUnderOven(int jump)
{
    RobotHead::getInstance()->lookAtThreaded("/map",0.527460,2.304037,0.832586);

    Gripper::getInstance(1)->openThreaded();
    Gripper::getInstance(0)->openThreaded();

    Torso *torso = Torso::getInstance();

    double lift = 0.29;


    boost::thread t(&Torso::pos, torso, lift);

    boost::thread t1(&OperateHandleController::plateTuckPose);

    RobotDriver::moveBase4(-0.031 - 0.1, 2.211,   0.034, 0.999);

    t1.join();

    announce("Handle",Geometry::make_pose(0.77, 2.367, 0.846, -0.719, -0.023, 0.032, 0.693, "/map"));

    RobotArm::getInstance(1)->universal_move_toolframe_ik(0.7, 2.367, 0.846, -0.719, -0.023, 0.032, 0.693, "/map");
    //RobotArm::getInstance(1)->universal_move_toolframe_ik(0.759, 2.367, 0.846, -0.719, -0.023, 0.032, 0.693, "/map");
    RobotArm::getInstance(1)->universal_move_toolframe_ik(0.77, 2.367, 0.846, -0.719, -0.023, 0.032, 0.693, "/map");
    //RobotArm::getInstance(1)->universal_move_toolframe_ik(0.77, 2.367, 0.846, -0.719, -0.023, 0.032, 0.693, "/map");
    Gripper::getInstance(1)->close();


    ros::Duration(0.5).sleep();

    ROS_ERROR("openFavouriteopenDrawerUnderOvenDrawer OPEN: %f", Gripper::getInstance(1)->getAmountOpen());

    if (Gripper::getInstance(1)->getAmountOpen() < 0.01)
    {
        RobotArm::getInstance(1)->universal_move_toolframe_ik(0.7, 2.367, 0.846, -0.719, -0.023, 0.032, 0.693, "/map");
        OperateHandleController::plateTuckPoseRight();
        return Current::openDrawerUnderOven(jump);
    }


    RobotArm::getInstance(1)->universal_move_toolframe_ik( 0.291, 2.335, 0.841,  -0.732, -0.008, 0.039, 0.680 , "/map"); // drawer_open

    return 0;
}


int Current::getLidOut(int jump)
{

    tf::Stamped<tf::Pose> lid;
    lid.frame_id_ = "/map";
    lid.setOrigin(btVector3(0.527460,2.304037,0.832586));
    lid.setRotation(btQuaternion(-0.155035,0.112869,0.756283,0.625509));

    announce("Lid", lid);

    tf::Stamped<tf::Pose> highlid = lid;
    highlid.setOrigin(lid.getOrigin() + btVector3(0,0,0.2));

    tf::Stamped<tf::Pose> prelid;
    prelid.frame_id_ = "/map";
    prelid.setOrigin(btVector3(0.542697,2.212567,0.871439));
    prelid.setRotation(btQuaternion(-0.155615,0.112088,0.756168,0.625644));

    tf::Stamped<tf::Pose> lid_pre_grip;
    lid_pre_grip.frame_id_ = "/map";
    lid_pre_grip.setOrigin(btVector3(0.543219, 2.230091, 0.836666));
    lid_pre_grip.setRotation(btQuaternion(-0.132676, 0.078832, 0.814762, 0.558879));

    tf::Stamped<tf::Pose> lid_final_grasp;
    lid_final_grasp.frame_id_ = "/map";
    lid_final_grasp.setOrigin(btVector3(0.500972, 2.299158, 0.834633));
    lid_final_grasp.setRotation(btQuaternion(-0.108869, 0.051036, 0.840173, 0.528821));

    Gripper::getInstance(0)->openThreaded();

    OperateHandleController::plateAttackPoseRight();

    RobotArm::getInstance(0)->raise_elbow = true;

    RobotArm::getInstance(0)->preset_angle = 2.7;

    double time = RobotArm::getInstance(0)->time_to_target;
    RobotArm::getInstance(0)->time_to_target = 0.5;
    RobotArm::getInstance(1)->time_to_target = 0.5;


    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(highlid);

    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(lid_pre_grip);

    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(lid_final_grasp);


    Gripper::getInstance(0)->close();


    //!TODO: check if we got something!

    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(highlid);


    tf::Stamped<tf::Pose> lidcarry;
    lidcarry.frame_id_ = "base_link";
    lidcarry.setOrigin(btVector3(0.465009,-0.200021,1.1));
    lidcarry.setRotation(btQuaternion(-0.151885,0.117935,0.735543,0.649614));

    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(lidcarry);

    RobotArm::getInstance(0)->time_to_target = time;
    RobotArm::getInstance(1)->time_to_target = time;


    RobotArm::getInstance(0)->raise_elbow = false;

    {
        tf::Stamped<tf::Pose> lidcarry;
        lidcarry.frame_id_ = "base_link";
        lidcarry.setOrigin(btVector3(0.088186, -0.539420, 1.110697));
        lidcarry.setRotation(btQuaternion(-0.061714, 0.110614, 0.436300, 0.890841));

        boost::thread t(&RobotArm::universal_move_toolframe_ik_pose,RobotArm::getInstance(0),lidcarry);
    }
    return 0;
}

int Current::getBowlOut(int jump)
{
    RobotHead::getInstance()->lookAtThreaded("/map",0.614, 1.945, 0.84);

    //RobotDriver::moveBase4(-0.031 - 0.1, 2.211,   0.034, 0.999); // bowl out

    Gripper::getInstance(1)->openThreaded();

    if (jump < 100)
    {

        tf::Stamped<tf::Pose> act = RobotArm::getInstance(1)->getToolPose("/map");
        //act.getOrigin() += btVector3(-.1,0,0);
        act.getOrigin() += btVector3(0,+.25,0);

        RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(act);
    }


    OperateHandleController::plateAttackPoseLeft();

    RobotArm::getInstance(1)->raise_elbow = true;

    btVector3 min(0.614, 1.945, 0.84);
    btVector3 max(0.796, 2.245, 0.92);
    btVector3 center;
    double radius;
    getCircle(min, max, center, radius);

    ROS_INFO("center %f %f %f", center.x(), center.y(), center.z());

    {
        tf::Stamped<tf::Pose> bowl;
        bowl.setOrigin(center);
        bowl.setRotation(btQuaternion(0,0,0,1));
        bowl.frame_id_ = "/map";

        announce("Bowl", bowl);
    }

    tf::Stamped<tf::Pose> grasp;
    grasp.setOrigin(center + btVector3(0,0.08,-0.035));
    grasp.setRotation(btQuaternion(-0.128, 0.591, -0.167, 0.778));
    grasp.frame_id_ = "/map";

    tf::Stamped<tf::Pose> grasphigh = grasp;
    grasphigh.getOrigin() += btVector3(0,0,0.1);

    tf::Stamped<tf::Pose> grasphighpre = grasp;
    grasphighpre.getOrigin() += btVector3(-.2,.1,0.1);

    ROS_INFO("grasphigh %f %f %f", grasphigh.getOrigin().x(), grasphigh.getOrigin().y(), grasphigh.getOrigin().z());

    RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(grasphighpre);
    RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(grasphigh);
    RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(grasp);

    Gripper::getInstance(1)->close();

    ros::Duration(0.5).sleep();

    ROS_ERROR("getBowlOut LID AMOUNT OPEN: %f", Gripper::getInstance(1)->getAmountOpen());

    if (Gripper::getInstance(1)->getAmountOpen() < 0.005)
    {
        RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(grasphigh);
        RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(grasphighpre);
        return Current::getBowlOut(101);
    }

    RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(grasphigh);

    RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(grasphighpre);

    RobotArm::getInstance(1)->raise_elbow = false;

    //RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(grasp);

    //Gripper::getInstance(1)->open();
    //RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(grasphigh);

    //OperateHandleController::plateAttackPose();

    tf::Stamped<tf::Pose> bowl_carry;
    bowl_carry.frame_id_ = "base_link";
    bowl_carry.setOrigin(btVector3(0.209078, 0.4470, 1.053729));
    bowl_carry.setRotation(btQuaternion(-0.153977, 0.579149, -0.128882, 0.790106));

    //RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(bowl_carry);

    boost::thread t(&RobotArm::universal_move_toolframe_ik_pose,RobotArm::getInstance(1),bowl_carry);



    return 0;
}

int Current::closeDrawerUnderOven(int jump)
{
    /*RobotArm::getInstance(1)->universal_move_toolframe_ik(0.759, 2.367, 0.846, -0.719, -0.023, 0.032, 0.693, "/map");
    Gripper::getInstance(1)->open();
    RobotArm::getInstance(1)->universal_move_toolframe_ik(0.7, 2.367, 0.846, -0.719, -0.023, 0.032, 0.693, "/map");

    OperateHandleController::plateTuckPoseLeft();*/

    {
        RobotDriver::moveBase4(-0.031 - 0.1, 2.211,   0.034, 0.999); // bowl out

        tf::Stamped<tf::Pose> bowl_carry;
        bowl_carry.frame_id_ = "base_link";
        bowl_carry.setOrigin(btVector3(0.209078, 0.4470, 1.053729));
        bowl_carry.setRotation(btQuaternion(-0.153977, 0.579149, -0.128882, 0.790106));

        RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(bowl_carry);

        //boost::thread t(&RobotArm::universal_move_toolframe_ik_pose,RobotArm::getInstance(1),bowl_carry);
        RobotHead::getInstance()->lookAtThreaded("/l_gripper_tool_frame",0, 0, .1);


        tf::Stamped<tf::Pose> pre_slam;
        pre_slam.frame_id_ = "/map";
        pre_slam.setOrigin(btVector3(0.154007, 2.283781, 0.764292));
        pre_slam.setRotation(btQuaternion(0.224605, 0.621592, -0.546879, 0.513906));

        RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(pre_slam);

        tf::Stamped<tf::Pose> post_slam;
        post_slam.frame_id_ = "/map";
        //post_slam.setOrigin(btVector3(0.693888, 2.316993, 0.792044));
        post_slam.setOrigin(btVector3(0.643888, 2.316993, 0.792044));
        post_slam.setRotation(btQuaternion(0.224605, 0.621592, -0.546879, 0.513906));

        RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(post_slam);

        //RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(bowl_carry);

        boost::thread t(&RobotArm::universal_move_toolframe_ik_pose,RobotArm::getInstance(1),bowl_carry);

    }
    return 0;
}

int Current::pourPopCorn(int jump)
{

    double modifier = -.06; // small pot vs high pot

    RobotHead::getInstance()->lookAtThreaded("/map",-1.906074,2.872285,1.171132);


    //RobotDriver::moveBase4(-1.187991, 2.401353, 0.999250, -0.038681);
    RobotDriver::moveBase4(-1.230, 2.782, 0.999, -0.042); // move base to island

//            t.join();

    tf::Stamped<tf::Pose> potPose, pre_grasp_r, grasp_r, pre_grasp_l, grasp_l;
    bool gotPotPose = false;
    while (!gotPotPose)
    {
        //gotPotPose = getPotPose(potPose, pre_grasp_l, grasp_l, pre_grasp_r, grasp_r);
        gotPotPose = getPotPose(potPose, pre_grasp_l, grasp_l, pre_grasp_r, grasp_r);
    }

    // pour the popcorn
    {

        RobotHead::getInstance()->lookAtThreaded("l_gripper_tool_frame",0,0,0);

        {
            tf::Stamped<tf::Pose> bowl_carry;
            bowl_carry.frame_id_ = "base_link";
            bowl_carry.setOrigin(btVector3(0.209078, 0.4470, 1.053729));
            bowl_carry.setRotation(btQuaternion(-0.153977, 0.579149, -0.128882, 0.790106));

            RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(bowl_carry);
        }

        tf::Stamped<tf::Pose> poura = RobotArm::getInstance(1)->getToolPose("/map");
        poura.getOrigin().setX(potPose.getOrigin().x()) ;
        poura.getOrigin().setY(potPose.getOrigin().y() - .1);
        poura.getOrigin().setZ(poura.getOrigin().getZ() + .1 + modifier);


        tf::Stamped<tf::Pose> bowl_pour_b;
        bowl_pour_b.frame_id_ = "/map";
        bowl_pour_b.setOrigin(btVector3(-1.892476 + 1.91 + potPose.getOrigin().x(), 2.942343 - 2.927 + potPose.getOrigin().y(), 1.180554 + .1 + modifier));
        bowl_pour_b.setRotation(btQuaternion(-0.167260, 0.680787, 0.601136, -0.383652));


        RobotArm::getInstance(0)->raise_elbow = true;

        // TODO findbasemove
        {
            tf::Stamped<tf::Pose> result;
            std::vector<int> arm;
            std::vector<tf::Stamped<tf::Pose> > goal;
            arm.push_back(1);
            arm.push_back(1);
            goal.push_back(bowl_pour_b);
            goal.push_back(poura);
            //bool RobotArm::findBaseMovement(tf::Stamped<tf::Pose> &result, std::vector<int> arm, std::vector<tf::Stamped<tf::Pose> > goal, bool drive, bool reach)

            RobotArm::findBaseMovement(result, arm, goal, true, false);
        }

        RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(poura);

        RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(bowl_pour_b);

        RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(poura);

        RobotArm::getInstance(0)->raise_elbow = false;

        tf::Stamped<tf::Pose> put_down;
        put_down.frame_id_ = "/map";
        put_down.setOrigin(btVector3(-1.610931, 1.983166 + .12, 0.929598));
        put_down.setRotation(btQuaternion(-0.353326, -0.545381, 0.643021, -0.405270));


        tf::Stamped<tf::Pose> put_down_inter;
        put_down_inter.frame_id_ = "/map";
        put_down_inter.setOrigin(0.5 * (put_down.getOrigin() + bowl_pour_b.getOrigin()));
        put_down_inter.setRotation(btQuaternion(-0.353326, -0.545381, 0.643021, -0.405270));

        RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(put_down_inter);

        RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(put_down);

        Gripper::getInstance(1)->openThreaded();

        ros::Duration(1).sleep();

        tf::Stamped<tf::Pose> put_down_high;
        put_down_high.frame_id_ = "/map";
        put_down_high.setOrigin(btVector3(-1.612672, 1.985903 + .12, 0.931158 + .1));
        put_down_high.setRotation(btQuaternion(-0.353345, -0.544065, 0.643510, -0.406245));

        RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(put_down_high);
    }
    return 0;

}

/*
int Current::putLidOn(int jump)
{

    tf::Stamped<tf::Pose> lidcarry;
    lidcarry.frame_id_ = "base_link";
    lidcarry.setOrigin(btVector3(0.088186, -0.539420, 1.110697));
    lidcarry.setRotation(btQuaternion(-0.061714, 0.110614, 0.436300, 0.890841));

    double modifier = -.06; // small pot vs high pot

    //pr2_controllers_msgs::JointTrajectoryGoal goalAT = RobotArm::getInstance(0)->goalTraj(-2.13282, -0.3, -1.209236618713309, -1.8705012213796224, -0.2192995424769604, -1.2529613898971834, 8.2846172962242548,1);
    //boost::thread t2T(&RobotArm::startTrajectory, RobotArm::getInstance(0), goalAT,true);
    //boost::thread t1(&RobotArm::startTrajectory,RobotArm::getInstance(0),tg);

    RobotHead::getInstance()->lookAtThreaded("/map",-1.906074,2.872285,1.171132);

    RobotDriver::moveBase4(-1.230, 2.782, 0.999, -0.042); // move base to island


    //tf::Stamped<tf::Pose> curr = RobotArm::getInstance(0)->getToolPose("base_link");
    //curr.getOrigin() += btVector3(0,0,.2);
    //boost::thread t9(&RobotArm::universal_move_toolframe_ik_pose, RobotArm::getInstance(0), curr);
    RobotArm::getInstance(0)->raise_elbow = true;

    tf::Stamped<tf::Pose> potPose, pre_grasp_r, grasp_r, pre_grasp_l, grasp_l;
    bool gotPotPose = false;

    discardKinectFrame();
    discardKinectFrame();

    while (!gotPotPose)
    {
        gotPotPose = getPotPose(potPose, pre_grasp_l, grasp_l, pre_grasp_r, grasp_r);
    }

    RobotHead::getInstance()->lookAtThreaded("r_gripper_tool_frame",0,0,0);

    //t9.join();

    double adj_x = 0.005;
    double adj_y = -.015;

    tf::Stamped<tf::Pose> goal;
    goal = RobotArm::getInstance(0)->getToolPose("/map");
    //goal.setOrigin(btVector3(potPose.getOrigin().x() + adj_x ,potPose.getOrigin().y() + adj_y,goal.getOrigin().z()));

    //RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(goal);

    //goal.setOrigin(btVector3(potPose.getOrigin().x() - .0,potPose.getOrigin().y() -.02 ,1.06 + modifier));
    goal.setOrigin(btVector3(potPose.getOrigin().x() + adj_x,potPose.getOrigin().y() + adj_y,1.06 + modifier - 0.01));

    tf::Stamped<tf::Pose> goal_base_rot = Geometry::getPoseIn("base_link", goal);
    goal_base_rot.setRotation(btQuaternion(0,0,0,1));
    goal_base_rot = Geometry::getPoseIn("/map", goal_base_rot);

    tf::Stamped<tf::Pose> goalappr_rot = goal;
    tf::Stamped<tf::Pose> goalappr_shift= goal;

    goalappr_rot =  Geometry::rotateAroundPose(goal,goal_base_rot,0,0.15,0);
    goalappr_rot.getOrigin() += btVector3(0.02,0,0.04);

    tf::Stamped<tf::Pose> goalappr_rot_high = goalappr_rot;

    //tf::Stamped<tf::Pose> goalappr_rot_shift = goalappr_rot;

    goalappr_rot_high.getOrigin() += btVector3(0,0,0.1);

    goalappr_shift.getOrigin() += btVector3(-0.05,0,0);

    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(goalappr_rot_high);
    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(goalappr_rot);
    //RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(goalappr_rot_shift);
    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(goalappr_shift);
    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(goal);

    tf::Stamped<tf::Pose> pot_pushed_to_cooker;
    pot_pushed_to_cooker.frame_id_ = "/map";
    pot_pushed_to_cooker.setOrigin(btVector3(-1.901817, 2.941642, 1.015725));
    pot_pushed_to_cooker.setRotation(btQuaternion(-0.117109, -0.076221, 0.877845, -0.458110));

    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(pot_pushed_to_cooker);

    //pot_pushed_to_cooker.getOrigin() += btVector3(0,0,0.1);

    //RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(pot_pushed_to_cooker);

    //goal.setOrigin(btVector3(potPose.getOrigin().x() + adj_x,potPose.getOrigin().y() + adj_y,1.06 + modifier - 0.02 + 0.1));

    //RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(goal);

    //RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(lidcarry);

    RobotArm::getInstance(0)->raise_elbow = false;

    //exit(0);

    Gripper::getInstance(0)->open();

    goal.setOrigin(btVector3(potPose.getOrigin().x() - .0,potPose.getOrigin().y() -.02 ,goal.getOrigin().z() + .1));

    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(goal);

    RobotArm::getInstance((0))->universal_move_toolframe_ik_pose(lidcarry);
    return 0;
}
*/

int Current::putLidOn(int jump)
{

    tf::Stamped<tf::Pose> lidcarry;
    lidcarry.frame_id_ = "base_link";
    lidcarry.setOrigin(btVector3(0.088186, -0.539420, 1.110697));
    lidcarry.setRotation(btQuaternion(-0.061714, 0.110614, 0.436300, 0.890841));

    double modifier = -.06; // small pot vs high pot

    //pr2_controllers_msgs::JointTrajectoryGoal goalAT = RobotArm::getInstance(0)->goalTraj(-2.13282, -0.3, -1.209236618713309, -1.8705012213796224, -0.2192995424769604, -1.2529613898971834, 8.2846172962242548,1);
    //boost::thread t2T(&RobotArm::startTrajectory, RobotArm::getInstance(0), goalAT,true);
    //boost::thread t1(&RobotArm::startTrajectory,RobotArm::getInstance(0),tg);

    RobotHead::getInstance()->lookAtThreaded("/map",-1.906074,2.872285,1.171132);

    RobotDriver::moveBase4(-1.230, 2.782, 0.999, -0.042); // move base to island

    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(lidcarry);

    Gripper::getInstance(0)->closeThreaded();


    //tf::Stamped<tf::Pose> curr = RobotArm::getInstance(0)->getToolPose("base_link");
    //curr.getOrigin() += btVector3(0,0,.2);
    //boost::thread t9(&RobotArm::universal_move_toolframe_ik_pose, RobotArm::getInstance(0), curr);
    RobotArm::getInstance(0)->raise_elbow = true;

    tf::Stamped<tf::Pose> potPose, pre_grasp_r, grasp_r, pre_grasp_l, grasp_l;
    bool gotPotPose = false;

    discardKinectFrame();
    discardKinectFrame();

    while (!gotPotPose)
    {
        gotPotPose = getPotPose(potPose, pre_grasp_l, grasp_l, pre_grasp_r, grasp_r);
    }

    //RobotHead::getInstance()->lookAtThreaded("r_gripper_tool_frame",0,0,0);

    //t9.join();

    double adj_x = 0.005;
    double adj_y = -.015;

    tf::Stamped<tf::Pose> goal;
    goal = RobotArm::getInstance(0)->getToolPose("/map");
    //goal.setOrigin(btVector3(potPose.getOrigin().x() + adj_x ,potPose.getOrigin().y() + adj_y,goal.getOrigin().z()));

    //RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(goal);

    //goal.setOrigin(btVector3(potPose.getOrigin().x() - .0,potPose.getOrigin().y() -.02 ,1.06 + modifier));
    goal.setOrigin(btVector3(potPose.getOrigin().x() + adj_x,potPose.getOrigin().y() + adj_y,1.06 + modifier - 0.01));
    //goal.getOrigin() += btVector3(0,0,0.04);

    tf::Stamped<tf::Pose> goal_base_rot = Geometry::getPoseIn("base_link", goal);
    goal_base_rot.setRotation(btQuaternion(0,0,0,1));
    goal_base_rot = Geometry::getPoseIn("/map", goal_base_rot);

    tf::Stamped<tf::Pose> goalappr_rot = goal;
    tf::Stamped<tf::Pose> goalappr_rot_backwards = goal;
    tf::Stamped<tf::Pose> goalappr_shift= goal;

    goalappr_rot =  Geometry::rotateAroundPose(goal,goal_base_rot,0,0.25,0);
    goalappr_rot.getOrigin() += btVector3(0.02,0,0.04);

    goalappr_rot_backwards =  Geometry::rotateAroundPose(goal,goal_base_rot,0,-0.15,0);
    goalappr_rot_backwards.getOrigin() -= btVector3(0.02,0,0.00);

    tf::Stamped<tf::Pose> goalappr_rot_high = goalappr_rot;

    //tf::Stamped<tf::Pose> goalappr_rot_shift = goalappr_rot;

    goalappr_rot_high.getOrigin() += btVector3(0,0,0.1);

    goalappr_shift.getOrigin() += btVector3(-0.05,0,0);

    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(goalappr_rot_high);
    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(goalappr_rot);
    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(goalappr_rot_backwards);
    //RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(goalappr_rot_shift);
    //RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(goalappr_shift);
    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(goal);

    tf::Stamped<tf::Pose> pot_pushed_to_cooker;
    pot_pushed_to_cooker.frame_id_ = "/map";
    pot_pushed_to_cooker.setOrigin(btVector3(-1.901817, 2.941642, 1.015725));
    pot_pushed_to_cooker.setRotation(btQuaternion(-0.117109, -0.076221, 0.877845, -0.458110));

    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(pot_pushed_to_cooker);

    if (jump == 17)
    {
        pot_pushed_to_cooker.getOrigin() += btVector3(0,0,0.1);

        RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(pot_pushed_to_cooker);

        //goal.setOrigin(btVector3(potPose.getOrigin().x() + adj_x,potPose.getOrigin().y() + adj_y,1.06 + modifier - 0.02 + 0.1));

        //RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(goal);

        RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(lidcarry);

        RobotArm::getInstance(0)->raise_elbow = false;

        exit(0);
    }

    Gripper::getInstance(0)->open();

    goal.setOrigin(btVector3(potPose.getOrigin().x() - .0,potPose.getOrigin().y() -.02 ,goal.getOrigin().z() + .1));

    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(goal);

    RobotArm::getInstance((0))->universal_move_toolframe_ik_pose(lidcarry);
    return 0;
}


void getLidAndBowl(int jump)
{
    switch (jump)

    {

        //get around cable
    case -2:
    {
        RobotDriver::moveBase4(-0.346670, 2.307890, 0.757347, 0.653002);
    }

    case -1:
    {
        Current::manipulateKnob(1);
        //Current::manipulateKnob(-1);
    }

    case 0:
    {
        Current::openDrawerUnderOven(jump);
    }

// get the lid out
    case 1:
    {
        Current::getLidOut(jump);
    }

    // get the bowl out
    case 2:
    {
        Current::getBowlOut(jump);
    }

    //close drawer
    case 3:
    {

        Current::closeDrawerUnderOven(jump);

    }

    //get around cable
    case 25:
    {
        RobotDriver::moveBase4(-0.346670, 2.307890, 0.757347, 0.653002);
    }

    // pour popcorn and put lid on pot
    case 4:
    {

        Current::pourPopCorn(jump);
    }

    case 5:
    {
        Current::putLidOn(jump);
    }

    }
}

int Current::removeLid(int jump)
{

    RobotHead::getInstance()->lookAtThreaded("/map", -1.86 , 2.76, 0.956078350673);

//    RobotHead::getInstance()->lookAtThreaded("/map", -1.89231883149, 2.93624266139, 0.956078350673);

    OperateHandleController::plateAttackPose();
    Gripper::getInstance(1)->openThreaded();

    RobotDriver::moveBase4(-1.243378, 2.790111, 0.998631, -0.052253);


    tf::Stamped<tf::Pose> lid;
    getLidPose(lid);

    btTransform lidRel;
    //lidRel.setOrigin(btVector3(0.026, 0.013, 0.042));
    lidRel.setOrigin(btVector3(0.026, 0.033, 0.042));
    lidRel.setRotation(btQuaternion(0.655, 0.742, -0.102, 0.097));

    //btTransform lidBt;
    //lidBt.setOrigin(lid.getOrigin());
    //lidBt.setRotation(lid.getRotation());

    btTransform lidAbs = lid * lidRel;

    btTransform lidPreRel;
    lidPreRel.setOrigin(btVector3(0.034, -0.043, 0.083));
    lidPreRel.setRotation(btQuaternion(0.649, 0.733, -0.142, 0.142));

    btTransform lidPreAbs = lid * lidPreRel;

    announce("Lid_grasp", Geometry::make_pose(lidAbs,"/map"));

    RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(Geometry::make_pose(lidPreAbs,"/map"));

    RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(Geometry::make_pose(lidAbs,"/map"));

    Gripper::getInstance(1)->close();

    ros::Duration(0.5).sleep();

    ROS_ERROR("REMOVE LID AMOUNT OPEN: %f", Gripper::getInstance(1)->getAmountOpen());

    if (Gripper::getInstance(1)->getAmountOpen() < 0.03)
    {
        RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(Geometry::make_pose(lidPreAbs,"/map"));
        OperateHandleController::plateAttackPoseLeft();
        return Current::removeLid(jump);
    }

    tf::Stamped<tf::Pose> lidhigh = Geometry::make_pose(lidAbs,"/map");
    lidhigh.getOrigin() += btVector3(0,0,.1);

    RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(lidhigh);

    tf::Stamped<tf::Pose> lidside;
    lidside.frame_id_ = "/map";
    lidside.setOrigin(btVector3(-1.719, 2.383, 0.918));
    lidside.setRotation(btQuaternion(0.287, 0.951, -0.031, 0.113));


    tf::Stamped<tf::Pose> lidsidehigh;
    lidsidehigh.frame_id_ = "/map";
    lidsidehigh.setOrigin(btVector3(-1.719, 2.383, 0.918 + 0.15));
    lidsidehigh.setRotation(btQuaternion(0.287, 0.951, -0.031, 0.113));

    RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(lidsidehigh);

    RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(lidside);


    Gripper::getInstance(1)->open();


    RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(lidsidehigh);





    //RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(Geometry::make_pose(lidAbs,"/map"));

    //Gripper::getInstance(1)->open();


    //RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(Geometry::make_pose(lidPreAbs,"/map"));

    //exit(0);





    /*tf::Stamped<tf::Pose> potPose, pre_grasp_r, grasp_r, pre_grasp_l, grasp_l;
    bool gotPotPose = false;
    while (!gotPotPose)
        gotPotPose = getPotPose(potPose, pre_grasp_l, grasp_l, pre_grasp_r, grasp_r);

    potPose.getOrigin().setZ(1.084787);

    tf::Stamped<tf::Pose> prelid;
    prelid.frame_id_ = "/map";
    prelid.setOrigin(potPose.getOrigin() - btVector3(0,.1,-.015));
    prelid.setRotation(btQuaternion(-0.006873, 0.028028, 0.721502, 0.691810));

    tf::Stamped<tf::Pose> lid;
    lid.frame_id_ = "/map";
    lid.setOrigin(potPose.getOrigin() - btVector3(-0.02,-.03,0)) ;
    lid.getOrigin().setZ(1.075707);
    lid.setRotation(btQuaternion(-0.054255, 0.079032, 0.718619, 0.688765));

    announce("Lid", lid);

    RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(prelid);

    RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(lid);

    Gripper::getInstance(1)->close();

    tf::Stamped<tf::Pose> lidhigh;
    lidhigh.frame_id_ = "/map";
    lidhigh.setOrigin(potPose.getOrigin() + btVector3(0,0,0.07));
    lidhigh.setRotation(btQuaternion(-0.053563, 0.077269, 0.718727, 0.688906));

    RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(lidhigh);


    tf::Stamped<tf::Pose> lidside;
    lidside.frame_id_ = "/map";
    lidside.setOrigin(btVector3(-1.799645, 2.544812, 1.148517));
    lidside.setRotation(btQuaternion(-0.066076, 0.066084, 0.707530, 0.700477));

    RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(lidside);
    tf::Stamped<tf::Pose> liddown;
    liddown.frame_id_ = "/map";
    liddown.setOrigin(btVector3(-1.751367, 2.353818, 0.922780));
    liddown.setRotation(btQuaternion(-0.073592, 0.048190, 0.918971, 0.384387));

    RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(liddown);

    Gripper::getInstance(1)->open();

    tf::Stamped<tf::Pose> lidexit;
    lidexit.frame_id_ = "/map";
    lidexit.setOrigin(btVector3(-1.686919, 2.283597, 0.933412));
    lidexit.setRotation(btQuaternion(-0.088774, 0.080089, 0.911498, 0.393544));

    RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(lidexit);*/

    return 0;
}


tf::Stamped<tf::Pose> hack_r, hack_l;


int Current::takePotFromIsland(int jump)
{
    //RobotHead::getInstance()->lookAtThreaded("/map", -1.89231883149, 2.93624266139, 0.956078350673);
    RobotHead::getInstance()->lookAtThreaded("/map", -1.86 , 2.76, 0.956078350673);

    // -1.77 2.76 .95

    boost::thread ty(&OperateHandleController::plateAttackPose);

    Gripper::getInstance(0)->openThreaded(0.04);
    Gripper::getInstance(1)->openThreaded(0.04);


    RobotArm::getInstance(0)->raise_elbow = true;
    RobotArm::getInstance(1)->raise_elbow = true;


    //RobotDriver::moveBase4(-1.318529, 2.688299, 0.998893, -0.047046);
    RobotDriver::moveBase4(-1.243378, 2.790111, 0.998631, -0.052253); // takepotfrom

    //RobotDriver::moveBase4(-1.243155, 2.693516, 0.999085, -0.042705);

    tf::Stamped<tf::Pose> potPose, pre_grasp_r, grasp_r, pre_grasp_l, grasp_l;
    bool gotPotPose = false;
    while (!gotPotPose)
    {
        gotPotPose = getPotPose(potPose, pre_grasp_l, grasp_l, pre_grasp_r, grasp_r,true);
        gotPotPose = getPotPose(potPose, pre_grasp_l, grasp_l, pre_grasp_r, grasp_r,true);
    }

    RobotArm::moveBothArms(pre_grasp_l,pre_grasp_r);

    grasp_l.getOrigin() += btVector3(0,0,-0.01);
    grasp_r.getOrigin() += btVector3(0,0,-0.01);

    hack_l = grasp_l;
    hack_r = grasp_r;

    RobotArm::moveBothArms(grasp_l,grasp_r);
    OperateHandleController::closeGrippers();

    ros::Duration(0.5).sleep();

    ROS_ERROR("takePotFromIsland  AMOUNT OPEN: %f", Gripper::getInstance(0)->getAmountOpen());
    ROS_ERROR("takePotFromIsland  AMOUNT OPEN: %f", Gripper::getInstance(1)->getAmountOpen());

    if ((Gripper::getInstance(1)->getAmountOpen() < 0.007) || (Gripper::getInstance(0)->getAmountOpen() < 0.007) ||
            (Gripper::getInstance(1)->getAmountOpen() > 0.03) || (Gripper::getInstance(0)->getAmountOpen() > 0.03))
    {

        Gripper::getInstance(0)->openThreaded(0.04);
        Gripper::getInstance(1)->open(0.04);

        RobotArm::moveBothArms(pre_grasp_l,pre_grasp_r);

        OperateHandleController::plateAttackPose();
        return Current::takePotFromIsland(jump);
    }



    //OperateHandleController::openGrippers();
    //RobotArm::moveBothArms(pre_grasp_l,pre_grasp_r);
    //OperateHandleController::plateAttackPose();
    //big pot
    //tf::Stamped<tf::Pose> rcarry0;
    //rcarry0.frame_id_ = "base_link";
    //rcarry0.setOrigin(btVector3(0.553647, -0.319272, 1.162564));
    //rcarry0.setRotation(btQuaternion(0.547706, 0.480213, -0.537497, -0.424865));

    tf::Stamped<tf::Pose> rcarry0;
    rcarry0.frame_id_ = "base_link";
    rcarry0.setOrigin(btVector3(0.564247, -0.253055, 1.162559));
    rcarry0.setRotation(btQuaternion(-0.520823, -0.509241, 0.559830, 0.394977));
    //big pot
    //tf::Stamped<tf::Pose> lcarry0;
    //lcarry0.frame_id_ = "base_link";
    //lcarry0.setOrigin(btVector3(0.528605, 0.034132, 1.164678));
    //lcarry0.setRotation(btQuaternion(0.527690, -0.480534, -0.526408, 0.462088));
    tf::Stamped<tf::Pose> lcarry0;
    lcarry0.frame_id_ = "base_link";
    lcarry0.setOrigin(btVector3(0.524614, 0.019969, 1.164658));
    lcarry0.setRotation(btQuaternion(-0.414234, 0.548428, -0.363319, 0.628997));

    grasp_l= Geometry::getPoseIn("base_link", grasp_l);

    lcarry0.setRotation(grasp_l.getRotation());


    RobotArm::moveBothArms(lcarry0,rcarry0);

    RobotArm::getInstance(0)->raise_elbow = false;
    RobotArm::getInstance(1)->raise_elbow = false;

    return 0;
}

void printPose(const char title[], tf::Pose pose)
{
    ROS_INFO("%s %f %f %f %f %f %f %f", title, pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z()
             , pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z(), pose.getRotation().w());
}


int Current::pushPot(int jump)
{
    RobotHead::getInstance()->lookAtThreaded("/map", -1.89231883149, 2.93624266139, 0.956078350673);

    //OperateHandleController::plateAttackPose();
    boost::thread ty(&OperateHandleController::plateAttackPose);

    Gripper::getInstance(0)->openThreaded(0.04);
    Gripper::getInstance(1)->openThreaded(0.04);

    //RobotDriver::moveBase4(-1.318529, 2.688299, 0.998893, -0.047046);
    RobotDriver::moveBase4(-1.243378 - 0.075, 2.790111, 0.998631, -0.052253);
    //RobotDriver::moveBase4(-1.243155, 2.693516, 0.999085, -0.042705);
    RobotArm::getInstance(0);

    tf::Stamped<tf::Pose> potPose, pre_grasp_r, grasp_r, pre_grasp_l, grasp_l;
    //to make sure we dont move, discard one frame
    discardKinectFrame();

    bool hadtodrive = true;

    bool gotPotPose = false;

    while (hadtodrive)
    {
        while (!gotPotPose)
            gotPotPose = getPotPose(potPose, pre_grasp_l, grasp_l, pre_grasp_r, grasp_r,true);

        printPose("potPose",potPose);

        tf::Stamped<tf::Pose> result;
        std::vector<int> arm;
        std::vector<tf::Stamped<tf::Pose> > goal;
        arm.push_back(0);
        arm.push_back(0);
        arm.push_back(1);
        arm.push_back(1);
        goal.push_back(pre_grasp_r);
        goal.push_back(grasp_r);
        goal.push_back(pre_grasp_l);
        goal.push_back(grasp_l);
        //bool RobotArm::findBaseMovement(tf::Stamped<tf::Pose> &result, std::vector<int> arm, std::vector<tf::Stamped<tf::Pose> > goal, bool drive, bool reach)

        RobotArm::findBaseMovement(result, arm, goal, true, false);

        if (result.getOrigin().length() < 0.01)
            hadtodrive = false;
    }


    RobotArm::getInstance(0)->raise_elbow = true;
    RobotArm::getInstance(1)->raise_elbow = true;

    RobotArm::moveBothArms(pre_grasp_l,pre_grasp_r);

    grasp_l.getOrigin() += btVector3(0,0,-0.01);
    grasp_r.getOrigin() += btVector3(0,0,-0.01);

    RobotArm::moveBothArms(grasp_l,grasp_r);
    OperateHandleController::closeGrippers();

    ros::Duration(0.5).sleep();

    {
        btTransform btri = grasp_r;
        btTransform btli = grasp_l;
        btTransform relIntended = btri.inverseTimes(btli);
        tf::Stamped<tf::Pose> act_r, act_l;
        act_r = RobotArm::getInstance(0)->getToolPose("map");
        act_l = RobotArm::getInstance(1)->getToolPose("map");
        btTransform btr = act_r;
        btTransform btl = act_l;
        btTransform relAct = btr.inverseTimes(btl);

        ROS_ERROR("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
        printPose("intended", relIntended);
        printPose("relAct", relAct);
        ROS_ERROR("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");

        if (1)
        {
            gotPotPose = false;
            while (!gotPotPose)
                gotPotPose = getPotPose(potPose, pre_grasp_l, grasp_l, pre_grasp_r, grasp_r,true);
            printPose("potPose after grasp",potPose);
        }
    }

    ROS_ERROR("pushPot AMOUNT OPEN: %f", Gripper::getInstance(0)->getAmountOpen());
    ROS_ERROR("pushPot AMOUNT OPEN: %f", Gripper::getInstance(1)->getAmountOpen());

    if ((Gripper::getInstance(1)->getAmountOpen() < 0.007) || (Gripper::getInstance(0)->getAmountOpen() < 0.007))
    {

        Gripper::getInstance(0)->openThreaded(0.04);
        Gripper::getInstance(1)->open(0.04);

        RobotArm::moveBothArms(pre_grasp_l,pre_grasp_r);

        OperateHandleController::plateAttackPose();
        return Current::pushPot(jump);
    }


    // push to side
    tf::Stamped<tf::Pose> rpot_down;
    rpot_down.frame_id_ = "/map";
    rpot_down.setOrigin(btVector3(-1.867984, 2.791455, 0.941677));
    rpot_down.setRotation(btQuaternion(-0.435962, 0.552046, -0.310802, 0.639206));


    tf::Stamped<tf::Pose> lpot_down;
    lpot_down.frame_id_ = "/map";
    lpot_down.setOrigin(btVector3(-1.837314, 2.528506, 0.945535));
    lpot_down.setRotation(btQuaternion(-0.608696, -0.378505, 0.631323, 0.296063));

    RobotHead::getInstance()->lookAtThreaded("/map",-1.837314, 2.528506, 0.945535);


    //RobotDriver::moveBase4(-1.173220, 2.577223, 0.998607, -0.052696);

    RobotArm::moveBothArms(lpot_down,rpot_down);

    //OperateHandleController::openGrippers();
    Gripper::getInstance(0)->openThreaded(0.04);
    Gripper::getInstance(1)->open(0.04);

    tf::Stamped<tf::Pose> rpot_down_exit;
    rpot_down_exit.frame_id_ = "/map";
    rpot_down_exit.setOrigin(btVector3(-1.867984, 2.791455 + .1, 0.941677));
    rpot_down_exit.setRotation(btQuaternion(-0.435962, 0.552046, -0.310802, 0.639206));


    tf::Stamped<tf::Pose> lpot_down_exit;
    lpot_down_exit.frame_id_ = "/map";
    lpot_down_exit.setOrigin(btVector3(-1.837314, 2.528506  - .1, 0.945535));
    lpot_down_exit.setRotation(btQuaternion(-0.608696, -0.378505, 0.631323, 0.296063));


    RobotArm::moveBothArms(lpot_down_exit,rpot_down_exit);

    OperateHandleController::plateAttackPose();

    RobotArm::getInstance(0)->raise_elbow = false;
    RobotArm::getInstance(1)->raise_elbow = false;


    return 0;
}


int pourReadyPopcorn_(double base_x, double base_y, double base_oz, double base_ow, double height_ = 0.0)
{

    RobotArm::getInstance(0)->raise_elbow = true;
    RobotArm::getInstance(1)->raise_elbow = true;

    //! pour out the popcorn

    //RobotDriver::moveBase4(-0.941867, 1.919927, 0.999686, -0.025022);

    //RobotDriver::moveBase4(-1.094190, 1.929376, 0.997743, -0.067087);
    //RobotDriver::moveBase4(-1.094190, 1.6,  0.997743, -0.067087);
    RobotDriver::moveBase4(base_x, base_y, base_oz, base_ow);

    tf::Stamped<tf::Pose> r,l,rrot,lrot,mid;

    r = RobotArm::getInstance(0)->getToolPose("/base_link");
    l = RobotArm::getInstance(1)->getToolPose("/base_link");

    r.getOrigin() += btVector3(0.02, 0,0);
    l.getOrigin() += btVector3(0.02, 0,0);

    mid = r;
    mid.setRotation(btQuaternion(0,0,0,1));
    mid.setOrigin(0.5 * (r.getOrigin() + l.getOrigin()));

    printPose("r" , r);
    printPose("l" , l);

    rrot =  Geometry::rotateAroundPose(r, mid, 0, 2.4, 0);
    lrot =  Geometry::rotateAroundPose(l, mid, 0, 2.4, 0);

    printPose("r" , r);
    printPose("l" , l);

    OperateHandleController::spinnerL(0,0,-.1 - height_);

    double time = RobotArm::getInstance(0)->time_to_target;

    RobotArm::getInstance(0)->time_to_target = 5;
    RobotArm::getInstance(1)->time_to_target = 5;

    double down = -.12;

    down = -.2 - height_;

    lrot.setOrigin(l.getOrigin() + btVector3(0,0,down));
    rrot.setOrigin(r.getOrigin() + btVector3(0,0,down));

    RobotArm::moveBothArms(lrot,rrot);

    RobotArm::getInstance(0)->time_to_target = time;
    RobotArm::getInstance(1)->time_to_target = time;

    lrot.setOrigin(l.getOrigin() + btVector3(0,0,0));
    rrot.setOrigin(r.getOrigin() + btVector3(0,0,0));

    RobotArm::moveBothArms(lrot,rrot);

    RobotArm::moveBothArms(l,r);

    RobotArm::getInstance(0)->raise_elbow = false;
    RobotArm::getInstance(1)->raise_elbow = false;


    return 0;
}


int fotopourReadyPopcorn_(double base_x, double base_y, double base_oz, double base_ow, double height_ , double angle, bool do_exit = false)
{

    RobotArm::getInstance(0)->raise_elbow = true;
    RobotArm::getInstance(1)->raise_elbow = true;

    //! pour out the popcorn

    //RobotDriver::moveBase4(-0.941867, 1.919927, 0.999686, -0.025022);

    //RobotDriver::moveBase4(-1.094190, 1.929376, 0.997743, -0.067087);
    //RobotDriver::moveBase4(-1.094190, 1.6,  0.997743, -0.067087);
    RobotDriver::moveBase4(base_x, base_y, base_oz, base_ow);


    tf::Stamped<tf::Pose> r,l,rrot,lrot,mid;

    r = RobotArm::getInstance(0)->getToolPose("/base_link");
    l = RobotArm::getInstance(1)->getToolPose("/base_link");

    r.getOrigin() += btVector3(0.02, 0,0);
    l.getOrigin() += btVector3(0.02, 0,0);

    mid = r;
    mid.setRotation(btQuaternion(0,0,0,1));
    mid.setOrigin(0.5 * (r.getOrigin() + l.getOrigin()));

    printPose("r" , r);
    printPose("l" , l);

    rrot =  Geometry::rotateAroundPose(r, mid, 0, angle, 0);
    lrot =  Geometry::rotateAroundPose(l, mid, 0, angle, 0);

    printPose("r" , r);
    printPose("l" , l);

    OperateHandleController::spinnerL(0,0,-.1 - height_);

    double time = RobotArm::getInstance(0)->time_to_target;

    RobotArm::getInstance(0)->time_to_target = 5;
    RobotArm::getInstance(1)->time_to_target = 5;

    double down = -.12;

    down = -.2 - height_;

    lrot.setOrigin(l.getOrigin() + btVector3(0,0,down));
    rrot.setOrigin(r.getOrigin() + btVector3(0,0,down));
    RobotArm::moveBothArms(lrot,rrot);


    if (do_exit)
        exit(0);

    RobotArm::getInstance(0)->time_to_target = time;
    RobotArm::getInstance(1)->time_to_target = time;

    lrot.setOrigin(l.getOrigin() + btVector3(0,0,0));
    rrot.setOrigin(r.getOrigin() + btVector3(0,0,0));

    RobotArm::moveBothArms(lrot,rrot);

    RobotArm::moveBothArms(l,r);

    RobotArm::getInstance(0)->raise_elbow = false;
    RobotArm::getInstance(1)->raise_elbow = false;


    return 0;
}



//return pot to heater

void returnPotToHeater()
{
    tf::Stamped<tf::Pose> rpot_down;
    rpot_down.frame_id_ = "/map";
    rpot_down.setOrigin(btVector3(-1.867984, 2.791455, 0.941677));
    rpot_down.setRotation(btQuaternion(-0.435962, 0.552046, -0.310802, 0.639206));


    tf::Stamped<tf::Pose> lpot_down;
    lpot_down.frame_id_ = "/map";
    lpot_down.setOrigin(btVector3(-1.837314, 2.528506, 0.945535));
    lpot_down.setRotation(btQuaternion(-0.608696, -0.378505, 0.631323, 0.296063));

    RobotHead::getInstance()->lookAtThreaded("/map",-1.837314, 2.528506, 0.945535);

    RobotDriver::moveBase4(-1.173220, 2.577223, 0.998607, -0.052696);

    RobotArm::moveBothArms(lpot_down,rpot_down);

    //OperateHandleController::openGrippers();
    Gripper::getInstance(0)->openThreaded(0.04);
    Gripper::getInstance(1)->open(0.04);

    tf::Stamped<tf::Pose> rpot_down_exit;
    rpot_down_exit.frame_id_ = "/map";
    rpot_down_exit.setOrigin(btVector3(-1.867984, 2.791455 + .1, 0.941677));
    rpot_down_exit.setRotation(btQuaternion(-0.435962, 0.552046, -0.310802, 0.639206));


    tf::Stamped<tf::Pose> lpot_down_exit;
    lpot_down_exit.frame_id_ = "/map";
    lpot_down_exit.setOrigin(btVector3(-1.837314, 2.528506  - .1, 0.945535));
    lpot_down_exit.setRotation(btQuaternion(-0.608696, -0.378505, 0.631323, 0.296063));

    RobotArm::moveBothArms(lpot_down_exit,rpot_down_exit);

    OperateHandleController::plateAttackPose();
}


void returnPotToSink()
{

    RobotHead::getInstance()->lookAtThreaded("/map",1.08, .31, 0.945535);


    RobotDriver::moveBase4(0.030936, 0.289066, 0.022239, 0.999752);
    RobotArm::getInstance(0)->raise_elbow = true;
    RobotArm::getInstance(1)->raise_elbow = true;


    //RobotArm::moveBothArms(lpot_down,rpot_down);
    tf::Stamped<tf::Pose> rputdown;
    rputdown.frame_id_ = "/map";
    rputdown.setOrigin(btVector3(1.060072, 0.144450, 0.957838));
    rputdown.setRotation(btQuaternion(0.545655, 0.541955, -0.543372, -0.336590));


    tf::Stamped<tf::Pose> lputdown;
    lputdown.frame_id_ = "/map";
    lputdown.setOrigin(btVector3(1.001025, 0.414402, 0.966610));
    lputdown.setRotation(btQuaternion(-0.382869, 0.495838, -0.391748, 0.673862));

    tf::Stamped<tf::Pose> rputdownexit;
    rputdownexit.frame_id_ = "/map";
    rputdownexit.setOrigin(btVector3(1.058829, 0.145553 - .07, 0.958970));
    rputdownexit.setRotation(btQuaternion(0.546275, 0.541866, -0.543263, -0.335902));


    tf::Stamped<tf::Pose> lputdownexit;
    lputdownexit.frame_id_ = "/map";
    lputdownexit.setOrigin(btVector3(0.999645, 0.415450 + .07, 0.967175));
    lputdownexit.setRotation(btQuaternion(-0.383121, 0.495288, -0.392308, 0.673797));

    {
        tf::Stamped<tf::Pose> result;
        std::vector<int> arm;
        std::vector<tf::Stamped<tf::Pose> > goal;
        arm.push_back(0);
        arm.push_back(0);
        arm.push_back(1);
        arm.push_back(1);
        goal.push_back(rputdown);
        goal.push_back(rputdownexit);
        goal.push_back(lputdown);
        goal.push_back(lputdownexit);
        //bool RobotArm::findBaseMovement(tf::Stamped<tf::Pose> &result, std::vector<int> arm, std::vector<tf::Stamped<tf::Pose> > goal, bool drive, bool reach)

        RobotArm::findBaseMovement(result, arm, goal, true, false);
    }

    RobotArm::moveBothArms(lputdown,rputdown);

    //OperateHandleController::openGrippers();
    Gripper::getInstance(0)->openThreaded(0.04);
    Gripper::getInstance(1)->open(0.04);

    RobotArm::moveBothArms(lputdownexit,rputdownexit);

    Gripper::getInstance(0)->openThreaded();
    Gripper::getInstance(1)->openThreaded();

    //RobotArm::moveBothArms(lpot_down_exit,rpot_down_exit);

    lputdownexit.getOrigin() += btVector3(0,0,0.1);
    rputdownexit.getOrigin() += btVector3(0,0,0.1);

    RobotArm::moveBothArms(lputdownexit,rputdownexit);

    RobotArm::getInstance(0)->raise_elbow = false;
    RobotArm::getInstance(1)->raise_elbow = false;

    OperateHandleController::plateAttackPose();
}

// this is a dirty hack, we keep record of how the bowl pose deviates from the exepected pose and simply add this to the position we salt above
// would probably be cleaner to keep the bowl pose instead of the delta..
btVector3 diff(-.12,0,0);

tf::Stamped<tf::Pose> getBigBowlPose(const btVector3 &search);

int Current::pourReadyPopcorn(int jump)
{
    RobotHead::getInstance()->lookAtThreaded("/map",-1.692, 1.621, 0.899);
    //pourReadyPopcorn_(-1.094190, 1.6,  0.997743, -0.067087);
    //pourReadyPopcorn_(-1.044190, 1.6,  0.997743, -0.067087);
    //pourReadyPopcorn_(-1, 1.6,  0.997743, -0.067087);

    RobotDriver::moveBase4(-1.154662, 1.573712, 0.998081, -0.061899);

    btVector3 search(-1.72, 1.6, .88);
    tf::Stamped<tf::Pose> bowlPose = getBigBowlPose(search);

    diff = bowlPose.getOrigin() - btVector3(-1.583374,1.634822,1.087549);

    {
        tf::Stamped<tf::Pose> inBase = Geometry::getPoseIn("base_link", bowlPose);
        printPose("BowlPoseinbase", bowlPose);
        // define where the bowl should be in base coords to make the trajectory work for it
        tf::Stamped<tf::Pose> goalPose = Geometry::make_pose(0.600358,-0.125386,0.736068,-0.001765,-0.002289,-0.004847,0.999984,"base_link");

        // calc how much we have to move to bring the bowl into the desired position relative to the robot
        btVector3 rel = inBase.getOrigin() - goalPose.getOrigin();
        ROS_ERROR("rel %f %f", rel.x(), rel.y());

        tf::Stamped<tf::Pose> nav_goal;
        nav_goal.frame_id_ = "/base_link";
        nav_goal.setRotation(btQuaternion(0,0,0,1));
        nav_goal.setOrigin(btVector3(rel.x(), rel.y(), 0));
        nav_goal = Geometry::getPoseIn("map", nav_goal);

        //RobotDriver::getInstance()->driveInMap(nav_goal, false); //

        printPose("BowlPose", bowlPose);

        pourReadyPopcorn_(nav_goal.getOrigin().x(), nav_goal.getOrigin().y(), nav_goal.getRotation().z(), nav_goal.getRotation().w(), 0);

    }

    return 0;
}




//RobotDriver::moveBase4(-1.087042, 1.707556 - .025, 0.998003, -0.063155);
void salt_it(btVector3 saltloc, double base_x =-1.087042 + .1, double base_y = 1.707556 - .025, double base_oz = 0.998003, double base_ow = -0.063155)
{
    btVector3 expected(-.9, 3.8, 0.88);

    //expected = btVector3(atof(argv[2]),atof(argv[3]),atof(argv[4]));
    expected = saltloc;

    double time = RobotArm::getInstance(0)->time_to_target;
    RobotArm::getInstance(0)->time_to_target = 0.5;
    RobotArm::getInstance(1)->time_to_target = 0.5;


    RobotHead::getInstance()->lookAtThreaded("/map", expected);

    boost::thread t0(&OperateHandleController::plateTuckPose);

    RobotDriver::moveBase4(base_x - .15, base_y, base_oz, base_ow);

    t0.join();

    RobotHead::getInstance()->lookAtThreaded("/map", expected);


    //RobotDriver::moveBase4(-1.087042, 1.707556, 0.998003, -0.063155);
    //

    RobotArm *rarm = RobotArm::getInstance(0);
    RobotArm *larm = RobotArm::getInstance(1);

    RobotHead::getInstance()->lookAtThreaded("/map", expected);

    boost::thread t1(&OperateHandleController::plateAttackPose);

    Gripper::getInstance(1)->openThreaded();
    Gripper::getInstance(0)->openThreaded();

    bool found = false;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inBox (new pcl::PointCloud<pcl::PointXYZRGB>);

    std::vector<btVector3> center;
    std::vector<double> radius;
    std::vector<int> numinliers;

    while (!found && ros::ok())
    {

        Kinect::getInstance()->getCloud(cloud);

        double search_radius = 0.15;
        double height = 0.10;

        getPointsInBox(cloud,expected - btVector3(search_radius,search_radius,-0.05),expected + btVector3(search_radius,search_radius,height),inBox);

        getCirclesFromCloud(*inBox, .02, .01,center,radius,numinliers,1);

        //if (numinliers[0] < 300)
        //return false;

        tf::Stamped<tf::Pose> lid;

        lid.setOrigin(center[0]);
        lid.setRotation(btQuaternion(0,0,0,1));
        lid.frame_id_ = "/map";

        announce("Salt", lid);

        if ((radius[0] < 0.05) && (radius[0] > 0.01) && (numinliers[0] > 50))
            found = true;
    }

    /*
        tf::Stamped<tf::Pose> lid;
    getLidPose(lid);

    btTransform lidRel;
    //lidRel.setOrigin(btVector3(0.026, 0.013, 0.042));
    lidRel.setOrigin(btVector3(0.026, 0.033, 0.042));
    lidRel.setRotation(btQuaternion(0.655, 0.742, -0.102, 0.097));

    btTransform lidBt;
    lidBt.setOrigin(lid.getOrigin());
    lidBt.setRotation(lid.getRotation());

    btTransform lidAbs = lid * lidRel;

    btTransform lidPreRel;
    lidPreRel.setOrigin(btVector3(0.034, -0.043, 0.083));
    lidPreRel.setRotation(btQuaternion(0.649, 0.733, -0.142, 0.142));

    btTransform lidPreAbs = lid * lidPreRel;

    announce("Lid_grasp", Geometry::make_pose(lidAbs,"/map"));

    RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(Geometry::make_pose(lidPreAbs,"/map"));

    */

    tf::Stamped<tf::Pose> cen;
    cen.setOrigin(center[0]);
    cen.setRotation(btQuaternion(0,0,0,1));
    cen.frame_id_ = "map";
    cen = Geometry::getPoseIn("/base_link", cen);
    cen.setRotation(btQuaternion(0.000, -0.001, 0.657, 0.754));


    btTransform lidRel;
    //lidRel.setOrigin(btVector3(-0.039, 0.012, -0.062));
    //lidRel.setRotation(btQuaternion(-0.040, 0.999, 0.008, 0.014));
    lidRel.setOrigin(btVector3(-0.030, -0.033, -0.068));
    lidRel.setRotation(btQuaternion(-0.389, 0.921, -0.014, 0.000));

    btTransform lidAbs = cen * lidRel;

    cen = Geometry::getPoseIn("/map",Geometry::make_pose(lidAbs,"base_link"));

    pubCloud(inBox);

    cen.getOrigin() += btVector3(0,0,0.054);

    tf::Stamped<tf::Pose> inMap,inMapAppr,inMap_high;
    inMap = cen;
    //inMap = Geometry::approach(inMap, 0.015);
    inMap = Geometry::approach(inMap, 0.02);
    inMapAppr = Geometry::approach(inMap, 0.1);

    inMap_high = inMapAppr;
    inMap_high.getOrigin() += btVector3(0,0,.1);

    t1.join();
    {
        tf::Stamped<tf::Pose> result;
        std::vector<int> arm;
        std::vector<tf::Stamped<tf::Pose> > goal;
        arm.push_back(1);
        arm.push_back(1);
        arm.push_back(1);
        goal.push_back(inMap_high);
        goal.push_back(inMapAppr);
        goal.push_back(inMap);
        //bool RobotArm::findBaseMovement(tf::Stamped<tf::Pose> &result, std::vector<int> arm, std::vector<tf::Stamped<tf::Pose> > goal, bool drive, bool reach)

        RobotArm::findBaseMovement(result, arm, goal, true, false);
    }

    RobotHead::getInstance()->lookAtThreaded("/l_gripper_tool_frame", 0,0,0);

    //RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(inMap_high);
    RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(inMapAppr);
    //Gripper::getInstance(1)->closeThreaded();
    //boost::thread t1(&Gripper::close,Gripper::getInstance(1),0);
    RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(inMap);
    Gripper::getInstance(1)->close();

    ros::Duration(0.5).sleep();

    ROS_ERROR("salt_it  AMOUNT OPEN: %f", Gripper::getInstance(0)->getAmountOpen());
    ROS_ERROR("salt_it  AMOUNT OPEN: %f", Gripper::getInstance(1)->getAmountOpen());

    if (Gripper::getInstance(1)->getAmountOpen() < 0.015)
    {

        Gripper::getInstance(1)->open();

        RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(inMap_high);

        OperateHandleController::plateAttackPose();

        salt_it(saltloc, base_x, base_y, base_oz, base_ow);

        return;
    }

    //t1.join();

    RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(inMap_high);

    if (0)
    {
        RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(inMap);
        Gripper::getInstance(1)->openThreaded();
        RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(inMap_high);
    }

    tf::Stamped<tf::Pose> curr = larm->getToolPose("/base_link");
    curr.setOrigin(btVector3(0.6,-0.03,curr.getOrigin().z()));
    curr.setRotation(btQuaternion(-1,1,0,0));
    larm->universal_move_toolframe_ik_pose(curr);

    double state[7];
    RobotArm::getInstance(1)->getJointState(state);
    state[6] += 3.142;
    boost::thread tturn(&RobotArm::startTrajectory, larm,larm->goalTraj(state[0],state[1],state[2],state[3],state[4],state[5],state[6]),true);
    //RobotArm::getInstance(1)->startTrajectory(larm->goalTraj(state[0],state[1],state[2],state[3],state[4],state[5],state[6]));
    tturn.join();

    tf::Stamped<tf::Pose> right_appr_rel;
    right_appr_rel.frame_id_ = "l_gripper_tool_frame";
    right_appr_rel.setOrigin(btVector3(0.000251, -0.000434, 0.085977));
    right_appr_rel.setRotation(btQuaternion(0.001757, -0.000095, 0.999998, -0.000168));

    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(right_appr_rel);

    tf::Stamped<tf::Pose> right_final_rel;
    right_final_rel.frame_id_ = "l_gripper_tool_frame";
    right_final_rel.setOrigin(btVector3(-0.068345, -0.000794, 0.056093));
    right_final_rel.setRotation(btQuaternion(0.001791, -0.000075, 0.999998, 0.001085));

    //Gripper::getInstance(0)->closeThreaded();
    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(right_final_rel);
    Gripper::getInstance(0)->close();

    //fake it
    if (0)
    {
        double open = Gripper::getInstance(0)->getAmountOpen();
        Gripper::getInstance(0)->openThreaded(open + 0.005);
    }

    tf::Stamped<tf::Pose> right_mid = rarm->getToolPose("base_link"); // mid as in no additional rotation
    tf::Stamped<tf::Pose>  left_mid = larm->getToolPose("base_link");
    tf::Stamped<tf::Pose>  left_minus,left_plus,right_minus,right_plus, left_act, right_act; // positive and negative rotation

    double amount = 0.2;

    left_minus =  Geometry::rotateAroundPose(left_mid, left_mid, 0,0,-amount);
    left_plus =  Geometry::rotateAroundPose(left_mid, left_mid, 0,0,amount);
    right_minus =  Geometry::rotateAroundPose(right_mid, right_mid, 0,0,-amount);
    right_plus =  Geometry::rotateAroundPose(right_mid, right_mid, 0,0,amount);

    double center_x = 0.6 - (0.5 * (right_mid.getOrigin().x() + left_mid.getOrigin().x()));
    double center_y = -0.03 - (0.5 * (right_mid.getOrigin().y() + left_mid.getOrigin().y()));

    //RobotDriver::moveBase4(-1.087042, 1.707556 - .025, 0.998003, -0.063155);
    RobotDriver::moveBase4(base_x + diff.x(), base_y + diff.y(), base_oz, base_ow);

    RobotArm::getInstance(0)->time_to_target = 0;
    RobotArm::getInstance(1)->time_to_target = 0;

    {
        tf::Stamped<tf::Pose> centerPose;
        centerPose.frame_id_ = "/base_link";
        centerPose.setOrigin(btVector3(center_x, center_y,0) + ((left_mid.getOrigin() + right_mid.getOrigin()) * 0.5));
        centerPose.setRotation(btQuaternion(0,0,0,1));
        centerPose = Geometry::getPoseIn("/map", centerPose);
        ROS_ERROR("no error");
        printPose("salting center", centerPose);
        announce("above bowl", centerPose);
    }


    bool bit = false;
    double rad = 0.07;
    double max = 4*M_PI;
    double step = M_PI / 2.5;
    double steps =  max / step;
    double radstep = rad / steps;
    for (double angle = 0; angle < max + M_PI; angle += step)
    {

        if (angle >= max)
            rad = 0;

        double x_add = cos(angle) * rad + center_x;
        double y_add = sin(angle) * rad + center_y;
        if (bit)
        {
            left_act = left_plus;
            right_act = right_minus;
        }
        else
        {
            left_act = left_minus;
            right_act = right_plus;
        }
        left_act.getOrigin() += btVector3(x_add,y_add,0);
        right_act.getOrigin() += btVector3(x_add,y_add,0);

        RobotArm::moveBothArms(left_act, right_act);

        bit = !bit;
        rad -= radstep * 0.7;
        //RobotArm::moveBothArms(left_plus, right_minus);
    }

    RobotArm::moveBothArms(left_mid, right_mid);

    RobotArm::getInstance(0)->time_to_target = time;
    RobotArm::getInstance(1)->time_to_target = time;


    if (1)
    {
        Gripper::getInstance(0)->openThreaded();
        RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(right_appr_rel);
        boost::thread t0(&OperateHandleController::plateAttackPoseRight);
        RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(inMap);
        Gripper::getInstance(1)->openThreaded();
        RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(inMapAppr);
        OperateHandleController::plateAttackPose();
    }
}

int Current::salt(int jump)
{
    salt_it(btVector3( -1.65,1.23 ,.894)); // 6:27 pour 6:50 pot down
    return 0;
}



void takepot(int jump)
{

    switch (jump)
    {

    case 0:
    {
        Current::removeLid(jump);
    }


    case 1:
    {
        Current::takePotFromIsland(jump);

    }

    case 2:
    {
        Current::pourReadyPopcorn(jump);
    }



    }
}

tf::Stamped<tf::Pose> getBigBowlPose(const btVector3 &search)
{

    RobotHead::getInstance()->lookAt("/map",search.x(),search.y(),search.z(),true);
    RobotHead::getInstance()->lookAtThreaded("/map",search);

    tf::Stamped<tf::Pose> bowl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    Kinect::getInstance()->getCloud(cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inBox (new pcl::PointCloud<pcl::PointXYZRGB>);

    getPointsInBox(cloud, search - btVector3(.2,.2,-.03), search + btVector3(.2,.2,.05),inBox);

    pubCloud(inBox);

    btVector3 center;
    double radius;

    getCircleFromCloud(*inBox, center, radius);

    bool found = true;
    double roi_size = btVector3(.4,.4,0).length();
    double circle_center_distance = (center - search).length();
    if (circle_center_distance > roi_size / 2)
        found = false;
    if (radius > 0.15)
        found = false;

    if (!found)
    {
        return getBigBowlPose(search);
    }
    else
    {
        bowl.frame_id_ = "/map";
        bowl.setOrigin(center);
        bowl.setRotation(btQuaternion(0,0,0,1));
        bowl = Geometry::getPoseIn("/base_link",bowl);
        bowl.setRotation(btQuaternion(0,0,0,1));
        bowl = Geometry::getPoseIn("/map",bowl);

        announce("bowl", bowl);

        printPose("bowl", bowl);
    }

    return bowl;
}

void calcBigBowlGraspPoses(const tf::Stamped<tf::Pose> &bowl, tf::Stamped<tf::Pose> &l_pregrasp, tf::Stamped<tf::Pose> &l_grasp, tf::Stamped<tf::Pose> &r_pregrasp, tf::Stamped<tf::Pose> &r_grasp)
{
    tf::Stamped<tf::Pose> pregrasp;
    pregrasp.setRotation(btQuaternion(btVector3(0,0,1), 0.3));
    pregrasp.setOrigin(btVector3(-.05,0,0));

    tf::Stamped<tf::Pose> l_grip_bowl;
    l_grip_bowl.frame_id_ = "bowl";
    l_grip_bowl.setOrigin(btVector3(0.009556, 0.118237, -0.009350));
    l_grip_bowl.setRotation(btQuaternion(-0.205230, 0.662406, -0.227324, 0.683683));

    tf::Stamped<tf::Pose> r_grip_bowl;
    r_grip_bowl.frame_id_ = "bowl";
    r_grip_bowl.setOrigin(btVector3(-0.009556, -0.118237, -0.009350));
    r_grip_bowl.setRotation(btQuaternion(-0.662406,-0.205230, 0.683683, 0.227324));

    l_grasp = Geometry::getRel(bowl, l_grip_bowl);
    r_grasp = Geometry::getRel(bowl, r_grip_bowl);

    l_pregrasp = Geometry::getRel(bowl, Geometry::getRel(l_grip_bowl, pregrasp));
    r_pregrasp = Geometry::getRel(bowl, Geometry::getRel(r_grip_bowl, pregrasp));
}

bool graspBigBowl(const tf::Stamped<tf::Pose> &bowl, bool grasp = true)
{
    tf::Stamped<tf::Pose> l_grip_bowl_,r_grip_bowl_,l_pre_grip_bowl_,r_pre_grip_bowl_;

    calcBigBowlGraspPoses(bowl, l_pre_grip_bowl_, l_grip_bowl_, r_pre_grip_bowl_, r_grip_bowl_);

    //if (grasp) {
    announce("r", r_grip_bowl_);
    announce("l", l_grip_bowl_);
    //}

    RobotArm::getInstance(0)->raise_elbow = true;
    RobotArm::getInstance(1)->raise_elbow = true;

    if (grasp)
    {
        OperateHandleController::openGrippers(true);

        tf::Stamped<tf::Pose> rt = RobotArm::getInstance(0)->getToolPose("/map");
        tf::Stamped<tf::Pose> lt = RobotArm::getInstance(1)->getToolPose("/map");

        rt.setRotation(r_pre_grip_bowl_.getRotation());
        lt.setRotation(l_pre_grip_bowl_.getRotation());

        rt.getOrigin() += r_pre_grip_bowl_.getOrigin() + btVector3(0,0,0.2);
        rt.getOrigin() *= 0.5;

        lt.getOrigin() += l_pre_grip_bowl_.getOrigin() + btVector3(0,0,0.2);
        lt.getOrigin() *= 0.5;

        {
            tf::Stamped<tf::Pose> result;
            std::vector<int> arm;
            std::vector<tf::Stamped<tf::Pose> > goal;
            arm.push_back(1);
            arm.push_back(1);
            arm.push_back(1);
            arm.push_back(0);
            arm.push_back(0);
            arm.push_back(0);
            goal.push_back(lt);
            goal.push_back(l_pre_grip_bowl_);
            goal.push_back(l_grip_bowl_);
            goal.push_back(rt);
            goal.push_back(r_pre_grip_bowl_);
            goal.push_back(r_grip_bowl_);
            //bool RobotArm::findBaseMovement(tf::Stamped<tf::Pose> &result, std::vector<int> arm, std::vector<tf::Stamped<tf::Pose> > goal, bool drive, bool reach)

            RobotArm::findBaseMovement(result, arm, goal, true, false);
        }


        RobotArm::moveBothArms(lt,rt);

        RobotArm::moveBothArms(l_pre_grip_bowl_,r_pre_grip_bowl_);
        Gripper::getInstance(0)->closeThreaded(0.04);
        Gripper::getInstance(1)->closeThreaded(0.04);
        RobotArm::moveBothArms(l_grip_bowl_,r_grip_bowl_);
        OperateHandleController::closeGrippers();

        ros::Duration(0.5).sleep();

        ROS_ERROR("graspBigBowl  AMOUNT OPEN: %f", Gripper::getInstance(0)->getAmountOpen());
        ROS_ERROR("graspBigBowl  AMOUNT OPEN: %f", Gripper::getInstance(1)->getAmountOpen());

        if ((Gripper::getInstance(1)->getAmountOpen() < 0.004) || (Gripper::getInstance(0)->getAmountOpen() < 0.004))
        {

            Gripper::getInstance(0)->openThreaded(0.04);
            Gripper::getInstance(1)->open(0.04);

            RobotArm::moveBothArms(l_pre_grip_bowl_,r_pre_grip_bowl_);

            OperateHandleController::plateAttackPose();


            RobotArm::getInstance(0)->raise_elbow = false;
            RobotArm::getInstance(1)->raise_elbow = false;


            return false;
        }


        RobotArm::moveBothArms(Geometry::getRelInBase(l_grip_bowl_,btVector3(0,0,.1)),Geometry::getRelInBase(r_grip_bowl_,btVector3(0,0,.1)));
    }
    else
    {
        tf::Stamped<tf::Pose> rapp, lapp;
        rapp = r_grip_bowl_;
        lapp = l_grip_bowl_;
        //rapp.getOrigin() += btVector3(0,0,0.05);
        //lapp.getOrigin() += btVector3(0,0,0.05);
        //r_grip_bowl_.getOrigin() += btVector3(0,0,0.01);
        //l_grip_bowl_.getOrigin() += btVector3(0,0,0.01);
        ROS_ERROR("EXT");
        //RobotArm::moveBothArms(lapp,rapp);
        RobotArm::moveBothArms(l_grip_bowl_,r_grip_bowl_);
        OperateHandleController::openGrippers(false);
        RobotArm::moveBothArms(l_pre_grip_bowl_,r_pre_grip_bowl_);
        l_pre_grip_bowl_.getOrigin() += btVector3(0,0,0.07);
        r_pre_grip_bowl_.getOrigin() += btVector3(0,0,0.07);
        RobotArm::moveBothArms(l_pre_grip_bowl_,r_pre_grip_bowl_);
        OperateHandleController::plateAttackPose();
    }

    RobotArm::getInstance(0)->raise_elbow = false;
    RobotArm::getInstance(1)->raise_elbow = false;

    return true;
}


void getBigBowlOut()
{
    btVector3 search(.51, 1.12, .75);

    bool got_it = false;

    while (!got_it)
    {
        tf::Stamped<tf::Pose> bowlPose = getBigBowlPose(search);
        got_it = graspBigBowl(bowlPose, true);
    }

    OperateHandleController::plateCarryPose();
}


int openFavouriteDrawer(int jump)
{
    btVector3 search(.51, 1.12, .75);

    RobotHead::getInstance()->lookAtThreaded("/map",search);

    Gripper::getInstance(0)->openThreaded();


    RobotDriver::moveBase4(-0.187269, 1.122678, 0.023650, 0.999720);

    tf::Stamped<tf::Pose> drawer_pre_closed_grasp;
    drawer_pre_closed_grasp.frame_id_ = "/map";
    drawer_pre_closed_grasp.setOrigin(btVector3(0.6, 0.868231, 0.789463));
    drawer_pre_closed_grasp.setRotation(btQuaternion(0.694022, 0.007679, 0.072881, 0.716214));

    // open drawer

    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(drawer_pre_closed_grasp);

    tf::Stamped<tf::Pose> drawer_closed_grasp;
    drawer_closed_grasp.frame_id_ = "/map";
    drawer_closed_grasp.setOrigin(btVector3(0.691291, 0.868231, 0.789463));
    drawer_closed_grasp.setOrigin(btVector3(0.671291, 0.868231, 0.789463));
    drawer_closed_grasp.setRotation(btQuaternion(0.694022, 0.007679, 0.072881, 0.716214));

    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(drawer_closed_grasp);

    Gripper::getInstance(0)->close();

    ros::Duration(0.5).sleep();

    ROS_ERROR("openFavouriteDrawer OPEN: %f", Gripper::getInstance(0)->getAmountOpen());

    if (Gripper::getInstance(0)->getAmountOpen() < 0.01)
    {
        Gripper::getInstance(0)->close();
        RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(drawer_pre_closed_grasp);
        OperateHandleController::plateTuckPoseRight();
        return openFavouriteDrawer(jump);
    }

    tf::Stamped<tf::Pose> drawer_open_grasp;
    drawer_open_grasp.frame_id_ = "/map";
    drawer_open_grasp.setOrigin(btVector3(0.217701, 0.836088, 0.783115));
    drawer_open_grasp.setRotation(btQuaternion(0.690426, -0.012174, 0.045241, 0.721884));

    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(drawer_open_grasp);

    Gripper::getInstance(0)->openThreaded();

    tf::Stamped<tf::Pose> drawer_avoid_open_grasp;
    drawer_avoid_open_grasp.frame_id_ = "/map";
    drawer_avoid_open_grasp.setOrigin(btVector3(0.217025, 0.635642, 0.783132));
    drawer_avoid_open_grasp.setRotation(btQuaternion(0.690618, -0.012459, 0.045518, 0.721678));

    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(drawer_avoid_open_grasp);

    OperateHandleController::plateAttackPose();

    //get bowl out

    //getBigBowlOut();

    return 0;
}

int getBowlFromDrawer(int jump)
{
    getBigBowlOut();
    return 0;
}



int closeDrawer(int jump)
{

    btVector3 search(.51, 1.12, .75);

    RobotHead::getInstance()->lookAtThreaded("/map",search);

    Gripper::getInstance(0)->openThreaded();

    OperateHandleController::plateTuckPose();

    RobotDriver::moveBase4(-0.187269, 1.122678, 0.023650, 0.999720);

    tf::Stamped<tf::Pose> drawer_pre_closed_grasp;
    drawer_pre_closed_grasp.frame_id_ = "/map";
    drawer_pre_closed_grasp.setOrigin(btVector3(0.6, 0.868231, 0.789463));
    drawer_pre_closed_grasp.setRotation(btQuaternion(0.694022, 0.007679, 0.072881, 0.716214));

    tf::Stamped<tf::Pose> drawer_closed_grasp;
    drawer_closed_grasp.frame_id_ = "/map";
    drawer_closed_grasp.setOrigin(btVector3(0.671291, 0.868231, 0.789463));
    drawer_closed_grasp.setRotation(btQuaternion(0.694022, 0.007679, 0.072881, 0.716214));

    tf::Stamped<tf::Pose> drawer_open_grasp;
    drawer_open_grasp.frame_id_ = "/map";
    drawer_open_grasp.setOrigin(btVector3(0.217701, 0.836088, 0.783115));
    drawer_open_grasp.setRotation(btQuaternion(0.690426, -0.012174, 0.045241, 0.721884));

    tf::Stamped<tf::Pose> drawer_avoid_open_grasp;
    drawer_avoid_open_grasp.frame_id_ = "/map";
    drawer_avoid_open_grasp.setOrigin(btVector3(0.217025, 0.635642, 0.783132));
    drawer_avoid_open_grasp.setRotation(btQuaternion(0.690618, -0.012459, 0.045518, 0.721678));

    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(drawer_avoid_open_grasp);

    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(drawer_open_grasp);

    //Gripper::getInstance(0)->close();

    //Gripper::getInstance(0)->openThreaded();

    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(drawer_closed_grasp);

    Gripper::getInstance(0)->open();

    RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(drawer_pre_closed_grasp);


    OperateHandleController::plateAttackPose();

    return 0;

}

void putBigBowlOntoIsland()
{
    btVector3 search(-1.699046, 1.632452, 0.88);

    RobotHead::getInstance()->lookAtThreaded("/map",search);

    RobotDriver::moveBase4(-0.977623, 1.694076, 0.998947, -0.045763);

    tf::Stamped<tf::Pose> bowlPose;
    bowlPose.setOrigin(search);
    bowlPose.setRotation(Geometry::getRelativeTransform("/map", "base_link").getRotation());
    bowlPose.frame_id_ = "/map";

    graspBigBowl(bowlPose, false);
}


void putBigBowlOntoTable(btVector3 search, double base_x, double base_y, double base_oz, double base_ow)
{
    //btVector3 search(.45, -1.35, .88 - .13);

    RobotHead::getInstance()->lookAtThreaded("/map",search);

    //RobotDriver::moveBase4(-0.977623, 1.694076, 0.998947, -0.045763);
    //RobotDriver::moveBase4(0.073525, -0.679463, -0.682670, 0.730720); // putBigBowlOntoTable
    RobotDriver::moveBase4(base_x, base_y, base_oz, base_ow);

    tf::Stamped<tf::Pose> bowlPose;
    bowlPose.setOrigin(search);
    bowlPose.setRotation(Geometry::getRelativeTransform("/map", "base_link").getRotation());
    bowlPose.frame_id_ = "/map";

    graspBigBowl(bowlPose, false);
}

void putBigBowlOntoIsland_heater()
{
    btVector3 search(-1.91044139862 + 0.03, 2.9423532486, 0.88);

    RobotHead::getInstance()->lookAtThreaded("/map",search);

    //RobotDriver::moveBase4(-0.977623, 1.694076, 0.998947, -0.045763);

    tf::Stamped<tf::Pose> bowlPose;
    bowlPose.setOrigin(search);
    bowlPose.setRotation(Geometry::getRelativeTransform("/map", "base_link").getRotation());
    bowlPose.frame_id_ = "/map";

    ROS_ERROR("putting it down now");

    graspBigBowl(bowlPose, false);
}



void shake_it()
{

    RobotArm *rarm = RobotArm::getInstance(0);
    RobotArm *larm = RobotArm::getInstance(1);
    tf::Stamped<tf::Pose> r_tp = rarm->getToolPose("/base_link");
    tf::Stamped<tf::Pose> l_tp = larm->getToolPose("/base_link");

    tf::Stamped<tf::Pose> r_tp_map = Geometry::getPoseIn("/map",r_tp);
    tf::Stamped<tf::Pose> l_tp_map = Geometry::getPoseIn("/map",l_tp);

    tf::Stamped<tf::Pose> mid,mid_tilted;
    mid.frame_id_ = "/base_link";
    mid.setOrigin(0.5 * (r_tp.getOrigin() + l_tp.getOrigin()));
    mid.setRotation(btQuaternion(0,0,0,1));

    btTransform midbt;
    midbt.setOrigin(mid.getOrigin());
    midbt.setRotation(mid.getRotation());

    double angle = 0;
    double amplitude = 0.35;
    double angle_increment = M_PI / 5.0;
    double amplitude_diminisher = amplitude / ((2 * M_PI) / angle_increment);

    ROS_ERROR("angle_increment %f amplitude_diminisher %f", angle_increment, amplitude_diminisher);

    ros::Rate r(2);
    while (angle < 2 * M_PI)
    {
        mid_tilted =  Geometry::rotateAroundPose(mid,mid,cos(angle) * amplitude,sin(angle) * amplitude,0);
        btTransform mid_tiltedbt;
        mid_tiltedbt.setOrigin(mid_tilted.getOrigin());
        mid_tiltedbt.setRotation(mid_tilted.getRotation());

        //announce("mid", mid);

        //announce("mid", mid_tilted);

        r.sleep();

        btTransform r,l;

        r = r_tp;
        l = l_tp;

        l = mid_tiltedbt * mid.inverseTimes(l);
        r = mid_tiltedbt * mid.inverseTimes(r);

        tf::Stamped<tf::Pose> act_l_tp = l_tp;
        act_l_tp.setOrigin(l.getOrigin());
        act_l_tp.setRotation(l.getRotation());
        tf::Stamped<tf::Pose> act_r_tp = r_tp;
        act_r_tp.setOrigin(r.getOrigin());
        act_r_tp.setRotation(r.getRotation());

        //announce("r",act_r_tp);
        //announce("l",act_l_tp);

        RobotArm::moveBothArms(act_l_tp, act_r_tp, 0, false);

        ros::Duration(0.5).sleep();

        angle += angle_increment;
        amplitude -= amplitude_diminisher;
    }


    RobotArm::moveBothArms(l_tp_map,r_tp_map);
    /*
    r
        x: -1.8773600001
        y: 3.0805883138
        z: 0.946
      orientation:
        x: -0.320103064305
        y: 0.630521636559
        z: -0.39629712381
        w: 0.585598056446

      l

        x: -1.91300054796
        y: 2.81695646567
        z: 0.947
      orientation:
        x: -0.583435311095
        y: -0.362567567153
        z: 0.594931073057
        w: 0.417378743261
    */

    tf::Stamped<tf::Pose> r_putdown;
    r_putdown.frame_id_ = "/map";
    r_putdown.setOrigin(btVector3( -1.8773600001, 3.0805883138, 0.946));
    r_putdown.setRotation(btQuaternion(-0.320103064305,0.630521636559,   -0.39629712381,  0.585598056446));

    tf::Stamped<tf::Pose> l_putdown;
    l_putdown.frame_id_ = "/map";
    l_putdown.setOrigin(btVector3(-1.91300054796, 2.81695646567, 0.947));
    l_putdown.setRotation(btQuaternion(-0.583435311095, -0.362567567153, 0.594931073057, 0.417378743261));

    //r_putdown = hack_r;
    //l_putdown = hack_l;

    tf::Stamped<tf::Pose> r_putdown_exit = r_putdown;
    r_putdown_exit.getOrigin() += btVector3(0,0.1,0.0);
    tf::Stamped<tf::Pose> l_putdown_exit = l_putdown;
    l_putdown_exit.getOrigin() -= btVector3(0,0.1,0.0);


    RobotArm::getInstance(0)->raise_elbow = true;
    RobotArm::getInstance(1)->raise_elbow = true;

    RobotArm::moveBothArms(l_putdown,r_putdown);

    if (0) // re-align to get pot to the right spot
    {
        // rosrun tf static_transform_publisher -1.899720 2.966096 0.960000 0.000000 0.000000 0.769873 0.638197 /map pot 100
        btVector3 potTarget = btVector3(-1.899720, 2.966096, 0.960000);

        tf::Stamped<tf::Pose> potPose, pre_grasp_r, grasp_r, pre_grasp_l, grasp_l;

        ros::Duration(0.5).sleep();

        bool gotPotPose = false;
        while (!gotPotPose)
            gotPotPose = getPotPose(potPose, pre_grasp_l, grasp_l, pre_grasp_r, grasp_r, true);

        btVector3 diff = potTarget - potPose.getOrigin();

        ROS_ERROR("POT DIFF %f %f", diff.x(), diff.y());

        diff.setZ(0);

        tf::Stamped<tf::Pose> radj, ladj;
        radj = r_putdown;
        ladj = l_putdown;
        radj.getOrigin() += diff;
        ladj.getOrigin() += diff;

        l_putdown_exit.getOrigin() += diff;
        r_putdown_exit.getOrigin() += diff;

        tf::Stamped<tf::Pose> l_putdown_exit_high, r_putdown_exit_high;
        l_putdown_exit_high = l_putdown_exit;
        r_putdown_exit_high = r_putdown_exit;

        l_putdown_exit_high.getOrigin() += btVector3(0,0,.1);
        r_putdown_exit_high.getOrigin() += btVector3(0,0,.1);

        {
            tf::Stamped<tf::Pose> result;
            std::vector<int> arm;
            std::vector<tf::Stamped<tf::Pose> > goal;
            arm.push_back(1);
            arm.push_back(1);
            arm.push_back(1);
            arm.push_back(0);
            arm.push_back(0);
            arm.push_back(0);
            goal.push_back(ladj);
            goal.push_back(l_putdown_exit);
            goal.push_back(l_putdown_exit_high);
            goal.push_back(radj);
            goal.push_back(r_putdown_exit);
            goal.push_back(r_putdown_exit_high);
            //bool RobotArm::findBaseMovement(tf::Stamped<tf::Pose> &result, std::vector<int> arm, std::vector<tf::Stamped<tf::Pose> > goal, bool drive, bool reach)

            RobotArm::findBaseMovement(result, arm, goal, true, false);
        }

        RobotArm::moveBothArms(ladj,radj);

        Gripper::getInstance(0)->openThreaded(0.055);
        Gripper::getInstance(1)->openThreaded(0.055);

        RobotArm::moveBothArms(l_putdown_exit,r_putdown_exit);

        OperateHandleController::openGrippers(false);

        RobotArm::moveBothArms(l_putdown_exit_high,r_putdown_exit_high);

        // 9x9x9x
    }
    else
    {
        Gripper::getInstance(0)->openThreaded(0.055);
        Gripper::getInstance(1)->openThreaded(0.055);

        RobotArm::moveBothArms(l_putdown_exit,r_putdown_exit);

        Gripper::getInstance(0)->openThreaded(0.055);
        Gripper::getInstance(1)->openThreaded(0.055);

        l_putdown_exit.getOrigin() += btVector3(0,0,.1);
        r_putdown_exit.getOrigin() += btVector3(0,0,.1);

        RobotArm::moveBothArms(l_putdown_exit,r_putdown_exit);
    }

    RobotArm::getInstance(0)->raise_elbow = false;
    RobotArm::getInstance(1)->raise_elbow = false;

    OperateHandleController::plateAttackPose();


}


int pourReadyPopcornTable(int jump)
{

    RobotHead::getInstance()->lookAtThreaded("/map",.27, -1.33, 0.899);

    RobotDriver::moveBase4(.3098 + diff.x() , -0.716 + diff.y(), -0.659599, 0.751613);

    //pourReadyPopcorn_(0.279824 + 0.025, -0.691109 -.03, -0.659599, 0.751613, 0.12);

    //pourReadyPopcorn_(0.279824 + xshift, -0.691109 + yshift, -0.659599, 0.751613, 0.12);
    tf::Stamped<tf::Pose> bowlPose = getBigBowlPose(btVector3(.27,-1.35, .75));
    bowlPose = getBigBowlPose(btVector3(.27,-1.35, .75));

    diff = bowlPose.getOrigin() - btVector3(.271, -1.33, 1.00);

    //ROS_ERROR("BOWL DIFF %f %f", diff.x(), diff.y());
    //for bowl .271 -1.33 1.00


    if (0)
    {

        bowlPose = Geometry::getPoseIn("map", bowlPose);
        RobotDriver::moveBase4(.3098 + diff.x() , -0.716 + diff.y(), -0.659599, 0.751613);
        bowlPose = Geometry::getPoseIn("base_link", bowlPose);
        printPose("Bowl Pose", bowlPose);
        exit(0);
        //0.600358 -0.125386 0.736068 -0.001765 -0.002289 -0.004847 0.999984
        //0.602894 -0.119965 0.736622 0.000888 -0.000972 0.004101 0.999991
    }

    pourReadyPopcorn_(.3098 + diff.x() , -0.716 + diff.y(), -0.659599, 0.751613, 0.12);

    //pourReadyPopcorn_(-1.094190, 1.6,  0.997743, -0.067087);
    return 0;
}



int current(int argc, char** argv)
{

    ros::NodeHandle node_handle;

    development_tools(argc, argv);

    //run the whole damn thing
    if (atoi(argv[1]) == -700)
    {
        getPotOut(0);
        getLidAndBowl(-2);
        Current::removeLid(0);
        takepot(1);
    }

    if (atoi(argv[1]) == -701)
    {

        int jump = 0;

        switch (atoi(argv[2]))
        {

        case 0 : //0:15
            Current::getPotOut_openDrawer(jump);

        case 1 : //0:54 pot on // 1:30
            Current::getPotOut_pickPlacePot(jump);

        case 2 :
            if ((argc > 3) && (atoi(argv[3])==1))
                RobotDriver::moveBase4(-0.346670, 2.307890, 0.757347, 0.653002);

        case 3 :
            Current::manipulateKnob(1); // 1:54
            //Current::manipulateKnob(-1);

        case 4 :
            Current::openDrawerUnderOven(jump); // 2:23

        case 5 :
            Current::getLidOut(jump); // 2:45

        case 6 :
            Current::getBowlOut(jump); // 3:15

        case 7 :
            Current::closeDrawerUnderOven(jump); // 3:30

        case 8 :
            if ((argc > 3) && (atoi(argv[3])==1))
                RobotDriver::moveBase4(-0.346670, 2.307890, 0.757347, 0.653002);
        case 9 :
            Current::pourPopCorn(jump); //3:54

        case 10 :
            Current::putLidOn(jump); //4:10
            RobotDriver::moveBase4(-0.904410, 2.493494, 0.998334, -0.057692); // for waiting

        case 11 :
            Current::pushPot(jump); // 4:31

        case 12 :
            if ((argc > 3) && (atoi(argv[3])==1))
                RobotDriver::moveBase4(-0.346670, 2.307890, 0.757347, 0.653002);
            Current::manipulateKnob(-1); // 5:13

        case 13 :
            Current::removeLid(jump); //5:51

        case 14 :
            Current::takePotFromIsland(jump); // 6:06

        case 15 :
            //Current::pourReadyPopcorn(jump); // 6:27 pour 6:50 pot down
            fotopourReadyPopcorn_(-1, 1.6,  0.997743, -0.067087, .0 ,  2.4,false);
        }
    }


    if (atoi(argv[1]) == -616)
    {
        tf::Stamped<tf::Pose> r,l,mid;


        r = RobotArm::getInstance(0)->getToolPose("/base_link");
        l = RobotArm::getInstance(1)->getToolPose("/base_link");

        mid = r;
        mid.setRotation(btQuaternion(0,0,0,1));
        mid.setOrigin(0.5 * (r.getOrigin() + l.getOrigin()));

        printPose("r" , r);
        printPose("l" , l);

        r =  Geometry::rotateAroundPose(r, mid, atof(argv[2]),atof(argv[3]), atof(argv[4]));
        l =  Geometry::rotateAroundPose(l, mid, atof(argv[2]),atof(argv[3]), atof(argv[4]));

        printPose("r" , r);
        printPose("l" , l);

        RobotArm::moveBothArms(l,r);
    }

    /*
    //test cop based pot detection
    if (atoi(argv[1]) == -617)
    {
        ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>( "/cop_detection", 0 , true);
        ros::Publisher best_pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>( "/cop_detection_best", 0, true );

        std::vector<tf::Stamped<tf::Pose> > poses;
        std::vector<double > votes;

        while (ros::ok())
        {
            tf::Stamped<tf::Pose> pose;
            pose = OperateHandleController::getCopPose("PotWW", "/openni_rgb_optical_frame");
            ros::Duration(0.05).sleep();
            if (pose.frame_id_ == "NO_ID_STAMPED_DEFAULT_CONSTRUCTION")
            {
                ROS_ERROR("dropped %s", pose.frame_id_.c_str());
                continue;
            }

            if (pose.getOrigin().length() < 0.1)
                continue;
            pose = Geometry::getPoseIn("/map", pose);

            geometry_msgs::PoseStamped pose_stamped, best_pose_stamped;

            tf::poseStampedTFToMsg(pose,pose_stamped);
            pose_pub.publish(pose_stamped);
            ros::Duration(0.05).sleep();

            if ((btVector3(-1.836059, 2.612557, 1.016513) - pose.getOrigin()).length() > 0.5)
                continue;

            {
                poses.push_back(pose);
                votes.push_back(.0);

                ROS_INFO("tick");

                if (poses.size() > 1)
                {
                    for (size_t k = 0; k < poses.size(); ++k)
                    {
                        votes[k] = 0;
                    }
                    double max_score = 0;
                    for (size_t k = 0; k < poses.size(); ++k)
                    {
                        for (size_t j = 0; j < poses.size(); ++j)
                        {
                            btTransform a,b,c;
                            a.setOrigin(poses[k].getOrigin());
                            a.setRotation(poses[k].getRotation());
                            b.setOrigin(poses[j].getOrigin());
                            b.setRotation(poses[j].getRotation());
                            c = b.inverseTimes(a);
                            double score = c.getOrigin().length() + c.getRotation().getAngle() * .001;
                            votes[j] += score;
                            votes[k] += score;
                            if (votes[k] > max_score)
                                max_score = votes[k];
                            if (votes[j] > max_score)
                                max_score = votes[j];
                        }
                    }
                    double minscore = 1000000;
                    tf::Stamped<tf::Pose> best;
                    for (size_t k = 0; k < poses.size(); ++k)
                    {
                        double act = votes[k] / max_score;
                        ROS_INFO("%zu score %f: %f", k, votes[k], act);
                        if (act < minscore)
                        {
                            minscore = act;
                            best = poses[k];
                        }
                    }
                    tf::poseStampedTFToMsg(best,best_pose_stamped);
                    ROS_INFO("min score = %f", minscore);
                    best_pose_pub.publish(best_pose_stamped);
                    ros::Duration(0.05).sleep();
                }
            }
        }
    }
    */


    //! bowl detection test?
    if (atoi(argv[1]) == -619)
    {

        RobotHead::getInstance()->lookAtThreaded("/map",0.527460,2.304037,0.832586);

        switch (atoi(argv[2]))
        {

        case 0 :
        {

            Gripper::getInstance(1)->openThreaded();

            btVector3 min(0.614, 1.945, 0.84);
            btVector3 max(0.796, 2.245, 0.92);
            btVector3 center;
            double radius;
            getCircle(min, max, center, radius);

            ROS_INFO("center %f %f %f", center.x(), center.y(), center.z());

            tf::Stamped<tf::Pose> grasp;
            grasp.setOrigin(center + btVector3(0,0.08,-0.02));
            grasp.setRotation(btQuaternion(-0.128, 0.591, -0.167, 0.778));
            grasp.frame_id_ = "/map";

            tf::Stamped<tf::Pose> grasphigh = grasp;
            grasphigh.getOrigin() += btVector3(0,0,0.1);

            ROS_INFO("grasphigh %f %f %f", grasphigh.getOrigin().x(), grasphigh.getOrigin().y(), grasphigh.getOrigin().z());

            RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(grasphigh);
            RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(grasp);

            Gripper::getInstance(1)->close();
            RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(grasphigh);
            RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(grasp);

            Gripper::getInstance(1)->open();
            RobotArm::getInstance(1)->universal_move_toolframe_ik_pose(grasphigh);


            OperateHandleController::plateAttackPose();

        }

        }

        /*
        - Translation: [0.614, 1.945, 0.807]
        - Rotation: in Quaternion [-0.514, 0.481, 0.476, 0.527]
                    in RPY [-1.470, 1.487, 0.084]
        ruehr@pr2b:~/sshsandbox/ias_drawer_executive$ rosrun tf tf_echo map l_gripper_tool_frame
        At time 1319040599.730
        - Translation: [0.796, 2.245, 0.810]
        - Rotation: in Quaternion [0.719, 0.008, -0.695, -0.008]
                    in RPY [-2.569, 1.529, 0.573]
                    */

    }


    if (atoi(argv[1]) == -620)
    {

        if (atoi(argv[2]) == 0) {

            tf::Stamped<tf::Pose> potPose, pre_grasp_r, grasp_r, pre_grasp_l, grasp_l;

            bool via_lid = false;
            if (argc > 3)
                via_lid = true;

            while (ros::ok())
            {
                bool gotPotPose = false;
                while (!gotPotPose)
                    gotPotPose = getPotPose(potPose, pre_grasp_l, grasp_l, pre_grasp_r, grasp_r, via_lid);
            }
        } else {
            //9x9x9x
            ROS_INFO("Dip");
            PotLocalizer *pot = new PotLocalizer();

            double table_height = 0.865;
            tf::Vector3 roi_min(-2.06, 2.41, table_height);
            tf::Vector3 roi_max(-1.7, 3.09, table_height + .4);

            tf::Stamped<tf::Pose> pose;

            ros::Rate rt(5);

            while (ros::ok())
            {
                ObjectLocalizer::localize("Pot",&pose, 1, Keywords("min",roi_min)("max",roi_max));
                rt.sleep();
            }
        }
    }


    if (atoi(argv[1]) == -621)
    {

        ros::Publisher cloud_pub = node_handle.advertise<sensor_msgs::PointCloud2>("/debug_cloud",0,true);


        while (ros::ok())
        {


            sensor_msgs::PointCloud2 pc = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect/cloud_throttled"));

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

            tf::Stamped<tf::Pose> net_stamped = Geometry::getPose("/map","/openni_rgb_optical_frame");
            tf::Transform net_transform;
            net_transform.setOrigin(net_stamped.getOrigin());
            net_transform.setRotation(net_stamped.getRotation());

            sensor_msgs::PointCloud2 pct; //in map frame

            pcl_ros::transformPointCloud("/map",net_transform,pc,pct);
            pct.header.frame_id = "/map";

            pcl::fromROSMsg(pct, *cloud);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr boundary (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr not_boundary (new pcl::PointCloud<pcl::PointXYZRGB>);
            std::vector<int> edge_indices;

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr inBox (new pcl::PointCloud<pcl::PointXYZRGB>);
            {

                Eigen::Vector4f min_pt, max_pt;



                min_pt = Eigen::Vector4f(0.33,.82,0.75, 1);
                max_pt = Eigen::Vector4f(.72,1.4,  .8, 1);
                /*
                {
                    tf::Stamped<tf::Pose> res;
                    res.setOrigin(min);
                    res.setRotation(btQuaternion(0,0,0,1));
                    res.frame_id_ = "/map";

                    geometry_msgs::PoseStamped res_msg;
                    tf::poseStampedTFToMsg(res,res_msg);

                    pose_pubr.publish(res_msg);
                }

                {
                    tf::Stamped<tf::Pose> res;
                    res.setOrigin(max);
                    res.setRotation(btQuaternion(0,0,0,1));
                    res.frame_id_ = "/map";

                    geometry_msgs::PoseStamped res_msg;
                    tf::poseStampedTFToMsg(res,res_msg);

                    pose_publ.publish(res_msg);
                }*/


                ROS_INFO("min %f %f %f" ,min_pt[0],min_pt[1],min_pt[2]);
                ROS_INFO("max %f %f %f" ,max_pt[0],max_pt[1],max_pt[2]);

                ROS_INFO("cloud size : %zu", cloud->points.size());

                boost::shared_ptr<std::vector<int> > indices( new std::vector<int> );
                pcl::getPointsInBox(*cloud,min_pt,max_pt,*indices);

                pcl::ExtractIndices<pcl::PointXYZRGB> ei;
                ei.setInputCloud(cloud);
                ei.setIndices(indices);
                ei.filter(*inBox);
            }


            findBoundary2(*inBox,*boundary,*not_boundary, edge_indices );

            sensor_msgs::PointCloud2 out; //in map frame

            pcl::toROSMsg(*boundary,out);
            out.header.frame_id = "/map";

            btVector3 center;
            double radius_goal = .1356;
            double radius = radius_goal;
            getCircleFromCloud( *boundary, center, radius);

            //if (fabs(radius - radius_goal) > 0.01)
            //   continue

            cloud_pub.publish(out);

            ROS_INFO("published");

        }

    }


    if (atoi(argv[1]) == -622)
    {

        //ros::Publisher cloud_pub = node_handle.advertise<sensor_msgs::PointCloud2>("/debug_cloud",0,true);
        ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>( "/cop_detection", 0 , true);
        ros::Publisher pose_pubr = node_handle.advertise<geometry_msgs::PoseStamped>( "/cop_detectionr", 0 , true);
        ros::Publisher pose_publ = node_handle.advertise<geometry_msgs::PoseStamped>( "/cop_detectionl", 0 , true);

        while (ros::ok())
        {
            sensor_msgs::PointCloud2 pc = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect/cloud_throttled"));

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

            tf::Stamped<tf::Pose> net_stamped = Geometry::getPose("/map","/openni_rgb_optical_frame");
            tf::Transform net_transform;
            net_transform.setOrigin(net_stamped.getOrigin());
            net_transform.setRotation(net_stamped.getRotation());

            sensor_msgs::PointCloud2 pct; //in map frame

            pcl_ros::transformPointCloud("/map",net_transform,pc,pct);
            pct.header.frame_id = "/map";

            pcl::fromROSMsg(pct, *cloud);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr boundary (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr not_boundary (new pcl::PointCloud<pcl::PointXYZRGB>);
            std::vector<int> edge_indices;

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr inBox (new pcl::PointCloud<pcl::PointXYZRGB>);
            {

                Eigen::Vector4f min_pt, max_pt;

                //min_pt = Eigen::Vector4f(0.33,.82,0.75, 1);
                //max_pt = Eigen::Vector4f(.72,1.4,  .8, 1);

                btVector3 min(0.33,.82,.75);
                btVector3 max(1.2, 1.4,.9);


                min_pt = Eigen::Vector4f(min.x(),min.y(),min.z(), 1);
                max_pt = Eigen::Vector4f(max.x(),max.y(),max.z(), 1);

                min_pt = Eigen::Vector4f(0.33,.82,0.75, 1);
                max_pt = Eigen::Vector4f(.72,1.4,  .8, 1);



                {
                    tf::Stamped<tf::Pose> res;
                    res.setOrigin(min);
                    res.setRotation(btQuaternion(0,0,0,1));
                    res.frame_id_ = "/map";

                    geometry_msgs::PoseStamped res_msg;
                    tf::poseStampedTFToMsg(res,res_msg);

                    pose_pubr.publish(res_msg);
                }

                {
                    tf::Stamped<tf::Pose> res;
                    res.setOrigin(max);
                    res.setRotation(btQuaternion(0,0,0,1));
                    res.frame_id_ = "/map";

                    geometry_msgs::PoseStamped res_msg;
                    tf::poseStampedTFToMsg(res,res_msg);

                    pose_publ.publish(res_msg);
                }


                ROS_INFO("min %f %f %f" ,min_pt[0],min_pt[1],min_pt[2]);
                ROS_INFO("max %f %f %f" ,max_pt[0],max_pt[1],max_pt[2]);

                ROS_INFO("cloud size : %zu", cloud->points.size());

                boost::shared_ptr<std::vector<int> > indices( new std::vector<int> );
                pcl::getPointsInBox(*cloud,min_pt,max_pt,*indices);

                pcl::ExtractIndices<pcl::PointXYZRGB> ei;
                ei.setInputCloud(cloud);
                ei.setIndices(indices);
                ei.filter(*inBox);
            }


            findBoundary2(*inBox,*boundary,*not_boundary, edge_indices );

            //btVector3 center;
            //double radius_goal = .1356;
            //double radius = radius_goal;
            std::vector<btVector3> center;
            std::vector<double> radius;
            std::vector<int> numinliers;;

            //getCircleFromCloud( *boundary, center, radius);
            getCirclesFromCloud(*boundary, .136, .02,
                                center,
                                radius,
                                numinliers,
                                2);

            //if (fabs(radius - radius_goal) > 0.01)
            //   continue



            ROS_INFO("published");

        }

    }


    if (atoi(argv[1]) == -623)
    {

        ros::Publisher cloud_pub = node_handle.advertise<sensor_msgs::PointCloud2>("/debug_cloud",0,true);

        rosbag::Bag bag;
        bag.open("clouds.bag", rosbag::bagmode::Write);


        while (ros::ok())
        {

            sensor_msgs::PointCloud2 pc = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect/cloud_throttled"));

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

            tf::Stamped<tf::Pose> net_stamped = Geometry::getPose("/map","/openni_rgb_optical_frame");
            tf::Transform net_transform;
            net_transform.setOrigin(net_stamped.getOrigin());
            net_transform.setRotation(net_stamped.getRotation());

            sensor_msgs::PointCloud2 pct; //in map frame

            pcl_ros::transformPointCloud("/map",net_transform,pc,pct);
            pct.header.frame_id = "/map";

            pcl::fromROSMsg(pct, *cloud);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr boundary (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr not_boundary (new pcl::PointCloud<pcl::PointXYZRGB>);
            std::vector<int> edge_indices;


            findBoundary2(*cloud,*boundary,*not_boundary, edge_indices );

            sensor_msgs::PointCloud2 out; //in map frame

            pcl::toROSMsg(*boundary,out);
            out.header.frame_id = "/map";

            //btVector3 center;
            //double radius_goal = .1356;
            //double radius = radius_goal;
            //getCircleFromCloud( *boundary, center, radius);

            //if (fabs(radius - radius_goal) > 0.01)
            //   continue

            cloud_pub.publish(out);

            ROS_INFO("published");


            bag.write("/kinect/cloud_throttled", ros::Time::now(), pct);
            bag.write("/boundaries", ros::Time::now(), out);


        }
        bag.close();

    }


    if (atoi(argv[1]) == -624)
    {
        while (ros::ok())
        {

            tf::Stamped<tf::Pose> lid;
            getLidPose(lid);
            printPose("lid", lid);

        }
    }


    if (atoi(argv[1]) == -625)
    {

        while (ros::ok())
        {
            tf::Stamped<tf::Pose> a;
            a.frame_id_ = "base_link";
            a.setOrigin(btVector3(0.388559, -0.559736, 1.131493));
            a.setRotation(btQuaternion(-0.623655, -0.207955, -0.531256, 0.534393));

            RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(a);

            RobotArm::getInstance(0)->evil_switch = true;


            tf::Stamped<tf::Pose> b;
            b.frame_id_ = "base_link";
            b.setOrigin(btVector3(0.389019, 0.000308, 1.131259));
            b.setRotation(btQuaternion(-0.624204, -0.207787, -0.530962, 0.534110));

            RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(b);

            bool reached = false;
            while (!reached)
            {
                reached = (RobotArm::getInstance(0)->getToolPose(b.frame_id_.c_str()).getOrigin() - b.getOrigin()).length() < 0.05;
                ros::Duration(0.001).sleep();
            }

            ROS_INFO("DIST : %f ", (RobotArm::getInstance(0)->getToolPose(b.frame_id_.c_str()).getOrigin() - b.getOrigin()).length());

            RobotArm::getInstance(0)->evil_switch = false;
        }

    }

    if (atoi(argv[1]) == -626)
    {
        tf::Stamped<tf::Pose> a;
        a.frame_id_ = "base_link";
        a.setOrigin(btVector3(0.388559, -0.559736, 1.131493));
        a.setRotation(btQuaternion(-0.623655, -0.207955, -0.531256, 0.534393));

        tf::Stamped<tf::Pose> b;
        b.frame_id_ = "base_link";
        b.setOrigin(btVector3(0.389019, 0.000308, 1.131259));
        b.setRotation(btQuaternion(-0.624204, -0.207787, -0.530962, 0.534110));

        while (ros::ok())
        {
            RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(a);

            RobotArm::getInstance(0)->universal_move_toolframe_ik_pose_tolerance(b,0.25);
        }
    }


    if (atoi(argv[1]) == -627)
    {
        Current::putLidOn(17);
    }



    if (atoi(argv[1]) == -628)
    {
        tf::Stamped<tf::Pose> r,l,mid;

        r = RobotArm::getInstance(0)->getToolPose("/base_link");

        mid = r;
        mid.setRotation(btQuaternion(0,0,0,1));

        printPose("r" , r);

        r =  Geometry::rotateAroundPose(r, mid, atof(argv[2]),atof(argv[3]), atof(argv[4]));

        printPose("r" , r);

        //RobotArm::moveBothArms(l,r);
        RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(r);
    }


    if (atoi(argv[1]) == -629)
    {
        tf::Stamped<tf::Pose> r,l,mid;


        r = RobotArm::getInstance(0)->getToolPose("/base_link");
        l = RobotArm::getInstance(1)->getToolPose("/base_link");

        mid = r;
        mid.setRotation(btQuaternion(0,0,0,1));
        mid.setOrigin(0.5 * (r.getOrigin() + l.getOrigin()));

        printPose("r" , r);
        printPose("l" , l);

        r =  Geometry::rotateAroundPose(r, mid, atof(argv[2]),atof(argv[3]), atof(argv[4]));
        l =  Geometry::rotateAroundPose(l, mid, -atof(argv[2]),-atof(argv[3]), -atof(argv[4]));

        printPose("r" , r);
        printPose("l" , l);

        RobotArm::moveBothArms(l,r);
    }



    if (atoi(argv[1]) == -630)
    {
        Current::salt(0);
    }


    if (atoi(argv[1]) == -631) // -1.7 1.66 .88 just detect
    {
        OperateHandleController::plateAttackPose();

        btVector3 search(atof(argv[2]),atof(argv[3]), atof(argv[4]));
        tf::Stamped<tf::Pose> bowlPose = getBigBowlPose(search);
        //graspBigBowl(bowlPose, true);

        //OperateHandleController::plateCarryPose();

        //graspBigBowl(bowlPose, false);
    }

    if (atoi(argv[1]) == -632) // -1.7 1.66 .88
    {
        OperateHandleController::plateAttackPose();

        btVector3 search(atof(argv[2]),atof(argv[3]), atof(argv[4]));
        tf::Stamped<tf::Pose> bowlPose = getBigBowlPose(search);

        graspBigBowl(bowlPose, true);

        OperateHandleController::plateCarryPose();
    }

    if (atoi(argv[1]) == -633) // put down
    {
        btVector3 search(atof(argv[2]),atof(argv[3]), atof(argv[4]));
        tf::Stamped<tf::Pose> bowlPose;
        bowlPose.setOrigin(search);
        bowlPose.setRotation(Geometry::getRelativeTransform("/map", "base_link").getRotation());
        bowlPose.frame_id_ = "/map";

        graspBigBowl(bowlPose, false);
    }

    // pour ready popcorn
    if (atoi(argv[1]) == -635) // -1.7 1.66 .88
    {

        RobotDriver::moveBase4(-1.154662, 1.573712, 0.998081, -0.061899);

        btVector3 search(atof(argv[2]),atof(argv[3]), atof(argv[4]));
        tf::Stamped<tf::Pose> bowlPose = getBigBowlPose(search);

        {
            tf::Stamped<tf::Pose> inBase = Geometry::getPoseIn("base_link", bowlPose);
            printPose("BowlPoseinbase", bowlPose);
            // define where the bowl should be in base coords to make the trajectory work for it
            tf::Stamped<tf::Pose> goalPose = Geometry::make_pose(0.600358,-0.125386,0.736068,-0.001765,-0.002289,-0.004847,0.999984,"base_link");

            // calc how much we have to move to bring the bowl into the desired position relative to the robot
            btVector3 rel = inBase.getOrigin() - goalPose.getOrigin();
            ROS_ERROR("rel %f %f", rel.x(), rel.y());

            tf::Stamped<tf::Pose> nav_goal;
            nav_goal.frame_id_ = "/base_link";
            nav_goal.setRotation(btQuaternion(0,0,0,1));
            nav_goal.setOrigin(btVector3(rel.x(), rel.y(), 0));
            nav_goal = Geometry::getPoseIn("map", nav_goal);

            //RobotDriver::getInstance()->driveInMap(nav_goal, false); //

            printPose("BowlPose", bowlPose);

            pourReadyPopcorn_(nav_goal.getOrigin().x(), nav_goal.getOrigin().y(), nav_goal.getRotation().z(), nav_goal.getRotation().w(), 0);

        }

        if (0)
        {
            tf::Stamped<tf::Pose> bowlPose = getBigBowlPose(search);
            bowlPose = Geometry::getPoseIn("/base_link", bowlPose);
            printPose("bowlpose_in_base_link", bowlPose);
            exit(0);
        }

        //graspBigBowl(bowlPose, true);

        //OperateHandleController::plateCarryPose();

        //graspBigBowl(bowlPose, false);
    }

    /*if (atoi(argv[1]) == -637)
    {
        // try rotating pot to distribute popcorn more evenly

        RobotArm *rarm = RobotArm::getInstance(0);
        RobotArm *larm = RobotArm::getInstance(1);
        tf::Stamped<tf::Pose> r_tp = rarm->getToolPose("/base_link");
        tf::Stamped<tf::Pose> l_tp = larm->getToolPose("/base_link");
        tf::Stamped<tf::Pose> mid,mid_tilted;
        mid.frame_id_ = "/base_link";
        mid.setOrigin(0.5 * (r_tp.getOrigin() + l_tp.getOrigin()));
        mid.setRotation(btQuaternion(0,0,0,1));

        btTransform midbt;
        midbt.setOrigin(mid.getOrigin());
        midbt.setRotation(mid.getRotation());

        double angle = 0;

        ros::Rate r(2);
        while (ros::ok())
        {
            mid_tilted =  RobotArm::rotateAroundPose(mid,mid,cos(angle) * 0.25,sin(angle) * 0.25,0);
            btTransform mid_tiltedbt;
            mid_tiltedbt.setOrigin(mid_tilted.getOrigin());
            mid_tiltedbt.setRotation(mid_tilted.getRotation());

            announce("mid", mid);

            announce("mid", mid_tilted);

            r.sleep();

            btTransform r,l;

            r = r_tp;
            l = l_tp;

            l = mid_tiltedbt * mid.inverseTimes(l);
            r = mid_tiltedbt * mid.inverseTimes(r);

            tf::Stamped<tf::Pose> act_l_tp = l_tp;
            act_l_tp.setOrigin(l.getOrigin());
            act_l_tp.setRotation(l.getRotation());
            tf::Stamped<tf::Pose> act_r_tp = r_tp;
            act_r_tp.setOrigin(r.getOrigin());
            act_r_tp.setRotation(r.getRotation());

            announce("r",act_r_tp);
            announce("l",act_l_tp);

            RobotArm::moveBothArms(act_l_tp, act_r_tp, 0, false);


            ros::Duration(0.5).sleep();


            angle += M_PI / 5.0;
        }
    }*/

    if (atoi(argv[1]) == -702)
    {

        int jump = 0;

        switch (atoi(argv[2]))
        {

        case 0 : //0:15
            Current::getPotOut_openDrawer(jump);

        case 1 : //0:54 pot on // 1:30
            Current::getPotOut_pickPlacePot(jump);

        case 2 :
            if ((argc > 3) && (atoi(argv[3])==1))
                RobotDriver::moveBase4(-0.346670, 2.307890, 0.757347, 0.653002);

        case 3 :
            Current::manipulateKnob(1); // 1:54
            //Current::manipulateKnob(-1);

        case 4 :
            Current::openDrawerUnderOven(jump); // 2:23

        case 5 :
            Current::getLidOut(jump); // 2:45

        case 6 :
            Current::getBowlOut(jump); // 3:15

        case 7 :
            Current::closeDrawerUnderOven(jump); // 3:30

        case 8 :
            if ((argc > 3) && (atoi(argv[3])==1))
                RobotDriver::moveBase4(-0.346670, 2.307890, 0.757347, 0.653002);
        case 9 :
            Current::pourPopCorn(jump); //3:54

        case 10 :
            Current::putLidOn(jump); //4:10
            //RobotDriver::moveBase4(-0.904410, 2.493494, 0.998334, -0.057692); // for waiting
        case 11:

            Current::takePotFromIsland(jump); // 6:06
            shake_it();
            ROS_ERROR(" SHAKE DONE ");
            //putBigBowlOntoIsland_heater();

        case 12:
            getBowlFromDrawer(0);

        case 13:
            putBigBowlOntoIsland();

        case 14:
            closeDrawer(0);

        case 15 :
            Current::pushPot(jump); // 4:31

        case 16 :
            if ((argc > 3) && (atoi(argv[3])==1))
                RobotDriver::moveBase4(-0.346670, 2.307890, 0.757347, 0.653002);
            Current::manipulateKnob(-1); // 5:13

        case 17 :
            Current::removeLid(jump); //5:51

        case 18 :
            Current::takePotFromIsland(jump); // 6:06

        case 19 :
            Current::pourReadyPopcorn(jump); // 6:27 pour 6:50 pot down

        case 20 :
            Current::salt(jump); // 6:27 pour 6:50 pot down

        }
    }

    /*if (atoi(argv[1]) == -638)
    {
        getBowlFromDrawer(0);
        btVector3 search(.27, -1.33, .75);
        putBigBowlOntoTable(search, 0.129038, -0.702062, -0.655191, 0.755459);
        closeDrawer(0);
    }
    if (atoi(argv[1]) == -639)
    {
        //btVector3 search(.45, -1.35, .88 - .13);
        btVector3 search(.27, -1.33, .75);
        putBigBowlOntoTable(search, 0.129038, -0.702062, -0.655191, 0.755459);
    }*/

    if (atoi(argv[1]) == -640)
    {

        //btVector3 search(.637, -1.486, .742 + .022); // should be 2.2 cm above table
        //RobotDriver::moveBase4(0.168233, -0.847962, -0.512474, 0.85870);
        //salt_it(search, 0.168233, -0.847962, -0.512474, 0.85870);
        //salt_it(search, 0.073525, -0.679463, -0.682670, 0.730720);
        // .18 -1.29
        btVector3 search(.43, -1.46, .742 + 0.022);
        salt_it(search, 0.129038 + 0.09, -0.702062 - 0.04, -0.655191, 0.75545);
    }

    if (atoi(argv[1]) == -703)
    {

        int jump = 0;

        switch (atoi(argv[2]))
        {

        case 0 : //0:15
            Current::getPotOut_openDrawer(jump);

        case 1 : //0:54 pot on // 1:30
            Current::getPotOut_pickPlacePot(jump);

        case 2 :
            if ((argc > 3) && (atoi(argv[3])==1))
                RobotDriver::moveBase4(-0.346670, 2.307890, 0.757347, 0.653002);

        case 3 :
            Current::manipulateKnob(1); // 1:54
            //Current::manipulateKnob(-1);

        case 4 :
            Current::openDrawerUnderOven(jump); // 2:23

        case 5 :
            Current::getLidOut(jump); // 2:45

        case 6 :
            Current::getBowlOut(jump); // 3:15

        case 7 :
            Current::closeDrawerUnderOven(jump); // 3:30

        case 8 :
            if ((argc > 3) && (atoi(argv[3])==1))
                RobotDriver::moveBase4(-0.346670, 2.307890, 0.757347, 0.653002);
        case 9 :
            Current::pourPopCorn(jump); //3:54

        case 10 :
            Current::putLidOn(jump); //4:10
            //RobotDriver::moveBase4(-0.904410, 2.493494, 0.998334, -0.057692); // for waiting
        case 11:

            Current::takePotFromIsland(jump); // 6:06

            shake_it();

            ROS_ERROR(" SHAKE DONE ");
            //putBigBowlOntoIsland_heater();

        case 12:
            openFavouriteDrawer(0);
        case 13:
            getBowlFromDrawer(0);

        case 14:
            //putBigBowlOntoIsland();

            putBigBowlOntoTable(btVector3(.27, -1.33, .75), 0.129038, -0.702062, -0.655191, 0.755459);

        case 15:

            closeDrawer(0);

        case 16 :
            Current::pushPot(jump); // 4:31

        case 17 :
            if ((argc > 3) && (atoi(argv[3])==1))
                RobotDriver::moveBase4(-0.346670, 2.307890, 0.757347, 0.653002);
            Current::manipulateKnob(-1); // 5:13

        case 18 :
            Current::removeLid(jump); //5:51

        case 19 :
            Current::takePotFromIsland(jump); // 6:06

        case 20 :

            pourReadyPopcornTable(jump); // 6:27 pour 6:50 pot down
            //pourReadyPopcorn(0);
            //RobotHead::getInstance()->lookAtThreaded("/map", -1.7, 1.5, 0.899);
            //fotopourReadyPopcorn_(-1, 1.6,  0.997743, -0.067087, 0,2.4,false);

        case 21 :

            //returnPotToHeater();
            returnPotToSink();

        case 22 :
            //Current::salt(jump); // 6:27 pour 6:50 pot down

            salt_it(btVector3(.43, -1.46, .742 + 0.022 + 0.006), 0.129038 + 0.09, -0.702062 - 0.04, -0.655191, 0.75545);
            exit(0);
        case 23:
            tf::Stamped<tf::Pose> roffer;
            roffer.frame_id_ = "/map";
            roffer.setOrigin(btVector3(0.128651, -1.243839, 0.835045));
            roffer.setRotation(btQuaternion(0.976158, -0.112988, -0.179133, -0.047544));

            tf::Stamped<tf::Pose> loffer;
            loffer.frame_id_ = "/map";
            loffer.setOrigin(btVector3(0.423095, -1.223293, 0.8270));
            loffer.setRotation(btQuaternion(-0.229902, 0.959984, 0.038952, 0.155109));

            RobotHead::getInstance()->lookAtThreaded("/map",.29, -1.38, 0.899);

            //RobotArm::moveBothArms(loffer,roffer);

            boost::thread t0(&OperateHandleController::plateAttackPose);

            RobotDriver::moveBase4(-0.135284, -0.136694, -0.676531, 0.736413);

            t0.join();

            //RobotHead::getInstance()->lookAt("/map", 2.1, -5.1, 1.6, true);
            //RobotHead::getInstance()->lookAt("/map", -2.8, -6.3, 1.6, true);

            boost::thread t1(&OperateHandleController::plateTuckPose);

            RobotHead::getInstance()->lookAtThreaded("/map",.29, -1.38, 0.899);


        }
    }

    if (atoi(argv[1]) == -641)
    {
        double xshift = atof(argv[2]);
        double yshift = atof(argv[3]);
        //pourReadyPopcorn_(0.279824 + 0.025, -0.691109 -.03, -0.659599, 0.751613, 0.12);
        RobotHead::getInstance()->lookAtThreaded("/map",.27, -1.33, 0.899);
        //pourReadyPopcorn_(0.279824 + xshift, -0.691109 + yshift, -0.659599, 0.751613, 0.12);
        tf::Stamped<tf::Pose> bowlPose = getBigBowlPose(btVector3(.27,-1.35, .75));

        btVector3 diff = bowlPose.getOrigin() - btVector3(.271, -1.33, 1.00);
        ROS_ERROR("BOWL DIFF %f %f", diff.x(), diff.y());
        //for bowl .271 -1.33 1.00
        pourReadyPopcorn_(.3098 + diff.x() + xshift, -0.716 + diff.y() + yshift, -0.659599, 0.751613, 0.12);
    }

    if (atoi(argv[1]) == -642)
    {
        ROS_INFO("RECORD TRAJECTORY WHILE GRIPPER IS CLOSED.");
        ROS_INFO("bin/ias_drawer_executive -642 amountOpenThreshold rate side");
        double gripperOpeningThreshold = atof(argv[2]);
        double rate = atof(argv[3]);
        ros::Rate rt(rate);

        while (ros::ok())
        {
            if (Gripper::getInstance(atoi(argv[4]))->getAmountOpen() < gripperOpeningThreshold)
                printPoseSimple(RobotArm::getInstance(atoi(argv[4]))->getToolPose("/map"));
            rt.sleep();
        }
    }

    if (atoi(argv[1]) == -643)
    {
        RobotDriver::moveBase4(-1.281009, -3.185903, 0.830314, 0.557294);


        RobotHead::getInstance()->lookAtThreaded("r_gripper_tool_frame",0,0,0);

        double pts[][7] ={{-1.351540,-2.296082,0.975278,0.659204,0.751421,-0.023565,-0.016137},
            {-1.351977,-2.296833,0.975388,0.658666,0.751905,-0.022882,-0.016551},
            {-1.351919,-2.297108,0.975577,0.660577,0.750314,-0.020253,-0.016063},
            {-1.352188,-2.296892,0.975475,0.664234,0.747228,-0.015198,-0.014589},
            {-1.351736,-2.301302,0.975433,0.659887,0.751031,-0.014568,-0.017011},
            {-1.343917,-2.335844,0.976201,0.619519,0.784706,-0.018606,-0.009272},
            {-1.329244,-2.372110,0.976194,0.574161,0.818253,-0.025994,-0.011172},
            {-1.312993,-2.402995,0.975653,0.524666,0.850551,-0.033886,-0.011848},
            {-1.289472,-2.436672,0.975557,0.476669,0.878117,-0.039786,-0.010718},
            {-1.248192,-2.478786,0.975374,0.407516,0.912462,-0.036654,-0.000978},
            {-1.196179,-2.513152,0.975324,0.316567,0.948113,-0.029311,0.002950},
            {-1.134346,-2.533039,0.975364,0.167569,0.985736,-0.012399,0.009524},
            {-1.083221,-2.539201,0.976012,0.087312,0.996137,-0.007728,-0.005284},
            {-1.045371,-2.537415,0.976197,0.028048,0.999574,-0.004110,-0.006976},
            {-1.010388,-2.531838,0.976724,-0.027442,0.999588,0.002041,-0.008125},
            {-0.990392,-2.526423,0.976715,-0.056290,0.998398,0.001162,-0.005625},
            {-0.989306,-2.526726,0.976869,-0.059351,0.998221,0.000320,-0.005661},
            {-0.988068,-2.526833,0.981427,-0.063703,0.997895,0.002038,-0.012009},
            {-0.987489,-2.526563,0.985804,-0.073449,0.997171,-0.003679,-0.015570},
            {-0.984144,-2.524953,0.984858,-0.071556,0.997236,-0.010002,-0.017344},
            {-0.986933,-2.526707,0.988158,-0.075933,0.997079,-0.004444,-0.006946},
            {-0.982650,-2.524833,0.988676,-0.077677,0.996949,-0.001231,-0.007560},
            {-0.981975,-2.524955,0.990235,-0.077920,0.996923,-0.004331,-0.007301},
            {-0.980884,-2.521257,0.991953,-0.079351,0.996689,-0.016710,-0.005972},
            {-0.979558,-2.521293,0.992222,-0.079630,0.996626,-0.019001,-0.005824}
        };

        std::vector<tf::Stamped<tf::Pose> > poses;

        poses.resize(24);

        for (size_t k = 0; k < poses.size(); k ++)
        {
            tf::Stamped<tf::Pose> act;
            act.frame_id_ = "map";
            act.setOrigin(btVector3(pts[k][0],pts[k][1],pts[k][2]));
            act.setRotation(btQuaternion(pts[k][3],pts[k][4],pts[k][5],pts[k][6]));
            poses[k] = act;
        }

        //we have our pose vector now:

        //check for a base pose
        {
            tf::Stamped<tf::Pose> result;
            std::vector<int> arm;
            std::vector<tf::Stamped<tf::Pose> > goal;

            for (size_t k = 0; k < poses.size(); k ++)
            {
                arm.push_back(0);
                goal.push_back(poses[k]);
            }

            RobotArm::findBaseMovement(result, arm, goal, true, false);
        }



        Gripper::getInstance(0)->open();

        for (int k = 10; k > 0 ; k --)
        {
            ROS_INFO("K %i", k);
            RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(poses[k]);
        }

        Gripper::getInstance(0)->close();

        for (size_t k = 0; k < poses.size() ; k++)
        {
            ROS_INFO("K %zu", k);
            RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(poses[k]);
        }

        Gripper::getInstance(0)->open();

        for (size_t k = poses.size() -1 ; k > 0 ; k --)
        {
            ROS_INFO("K %zu", k);
            RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(poses[k]);
        }

    }


    if (atoi(argv[1]) == -644)
    {
        ROS_INFO("RECORD TRAJECTORY WHILE GRIPPER IS CLOSED.");
        ROS_INFO("bin/ias_drawer_executive -642 amountOpenThreshold rate side");
        double gripperOpeningThreshold = atof(argv[2]);
        double rate = atof(argv[3]);
        ros::Rate rt(rate);


        ros::Time start = ros::Time::now();
        bool hit = false;

        tf::Stamped<tf::Pose> act = RobotArm::getInstance(atoi(argv[4]))->getToolPose("/map");

        while (ros::ok())
        {
            if (Gripper::getInstance(atoi(argv[4]))->getAmountOpen() < gripperOpeningThreshold)
            {
                if (!hit)
                {
                    hit = true;
                    start = ros::Time::now();
                    printf("std::vector<tf::Stamped<tf::Pose> > poses;\n");
                    printf("std::vector<double > timestamps;\n");
                    printf("tf::Stamped<tf::Pose> curr; curr.frame_id_ = \"/map\";  \n");
                }
                tf::Stamped<tf::Pose> act = RobotArm::getInstance(atoi(argv[4]))->getToolPose("/map");
                printf("curr.setOrigin(btVector3(%f,%f,%f));", act.getOrigin().x(), act.getOrigin().y(), act.getOrigin().z());
                printf("curr.setRotation(btQuaternion(%f,%f,%f,%f));", act.getRotation().x(), act.getRotation().y(), act.getRotation().z(),act.getRotation().w());
                printf("poses.push_back(curr);");
                ros::Duration actTime = ros::Time::now() - start;
                printf("timestamps.push_back(%f);\n",actTime.toSec());
                //printPoseSimple(RobotArm::getInstance(atoi(argv[4]))->getToolPose("/map"));
            }
            rt.sleep();
        }
    }

    if (atoi(argv[1]) == -645)
    {

        std::vector<tf::Stamped<tf::Pose> > poses;
        std::vector<double > timestamps;
        tf::Stamped<tf::Pose> curr;
        curr.frame_id_ = "/map";
        curr.setOrigin(btVector3(-0.858909,2.662146,1.069023));
        curr.setRotation(btQuaternion(-0.063184,0.705228,0.700371,-0.090230));
        poses.push_back(curr);
        timestamps.push_back(0.010172);
        curr.setOrigin(btVector3(-0.858132,2.662023,1.068230));
        curr.setRotation(btQuaternion(-0.061947,0.706447,0.699131,-0.091168));
        poses.push_back(curr);
        timestamps.push_back(0.510091);
        curr.setOrigin(btVector3(-0.856363,2.663267,1.066434));
        curr.setRotation(btQuaternion(-0.059214,0.709004,0.696730,-0.091511));
        poses.push_back(curr);
        timestamps.push_back(1.010011);
        curr.setOrigin(btVector3(-0.830527,2.676649,1.066342));
        curr.setRotation(btQuaternion(-0.043343,0.708105,0.701059,-0.072286));
        poses.push_back(curr);
        timestamps.push_back(1.510092);
        curr.setOrigin(btVector3(-0.727655,2.686956,1.064891));
        curr.setRotation(btQuaternion(-0.053571,0.706820,0.704510,-0.034657));
        poses.push_back(curr);
        timestamps.push_back(2.010104);
        curr.setOrigin(btVector3(-0.607844,2.665836,1.075091));
        curr.setRotation(btQuaternion(-0.053020,0.709189,0.699932,-0.065840));
        poses.push_back(curr);
        timestamps.push_back(2.510075);
        curr.setOrigin(btVector3(-0.568007,2.651925,1.071384));
        curr.setRotation(btQuaternion(-0.054405,0.709798,0.699263,-0.065253));
        poses.push_back(curr);
        timestamps.push_back(3.010017);
        curr.setOrigin(btVector3(-0.568066,2.650177,1.071403));
        curr.setRotation(btQuaternion(-0.091834,0.724687,0.679900,-0.064276));
        poses.push_back(curr);
        timestamps.push_back(3.510078);
        curr.setOrigin(btVector3(-0.562386,2.652335,1.072278));
        curr.setRotation(btQuaternion(-0.096352,0.716489,0.687963,-0.063779));
        poses.push_back(curr);
        timestamps.push_back(4.010068);
        curr.setOrigin(btVector3(-0.550039,2.660620,1.069650));
        curr.setRotation(btQuaternion(-0.087717,0.715498,0.691267,-0.050182));
        poses.push_back(curr);
        timestamps.push_back(4.510390);

        std::vector<int> arms;
        for (int k =0; k < poses.size(); k++)
            arms.push_back(0);

        tf::Stamped<tf::Pose> result;
        RobotArm::findBaseMovement(result, arms, poses, true, false);

        RobotArm::getInstance(0)->universal_move_toolframe_ik_pose(poses[0]);
        RobotArm::getInstance(0)->startTrajectory(RobotArm::getInstance(0)->multiPointTrajectory(poses,timestamps));

    }


    if (atoi(argv[1]) == -677)
    {
        fotopourReadyPopcorn_(atof(argv[2]), 1.6,  0.997743, -0.067087, atof(argv[3]), atof(argv[4]),atof(argv[5]) > 0);
    }

    if (atoi(argv[1]) == -677)
    {
        salt_it(btVector3(.43, -1.46, .742 + 0.022 + 0.006), 0.129038 + 0.09, -0.702062 - 0.04, -0.655191, 0.75545);
    }



    if (atoi(argv[1]) == -704)
    {

        int jump = 0;

        switch (atoi(argv[2]))
        {

        case 0 : //0:15
            Current::getPotOut_openDrawer(jump);

        case 1 : //0:54 pot on // 1:30
            Current::getPotOut_pickPlacePot(jump);

        case 2 :
            if ((argc > 3) && (atoi(argv[3])==1))
                RobotDriver::moveBase4(-0.346670, 2.307890, 0.757347, 0.653002);

        case 3 :
            Current::manipulateKnob(1); // 1:54
            //Current::manipulateKnob(-1);

        case 4 :
            Current::openDrawerUnderOven(jump); // 2:23

        case 5 :
            Current::getLidOut(jump); // 2:45

        case 6 :
            Current::getBowlOut(jump); // 3:15

        case 7 :
            Current::closeDrawerUnderOven(jump); // 3:30

        case 8 :
            if ((argc > 3) && (atoi(argv[3])==1))
                RobotDriver::moveBase4(-0.346670, 2.307890, 0.757347, 0.653002);
        case 9 :
            Current::pourPopCorn(jump); //3:54

        case 10 :
            Current::putLidOn(jump); //4:10
            //RobotDriver::moveBase4(-0.904410, 2.493494, 0.998334, -0.057692); // for waiting
        case 11:

            Current::takePotFromIsland(jump); // 6:06

            shake_it();

            ROS_ERROR(" SHAKE DONE ");
            //putBigBowlOntoIsland_heater();

        case 12:
            openFavouriteDrawer(0);
        case 13:
            getBowlFromDrawer(0);

        case 14:
            putBigBowlOntoIsland();

            //putBigBowlOntoTable(btVector3(.27, -1.33, .75), 0.129038, -0.702062, -0.655191, 0.755459);

        case 15:

            closeDrawer(0);

        case 16 :
            Current::pushPot(jump); // 4:31

        case 17 :
            if ((argc > 3) && (atoi(argv[3])==1))
                RobotDriver::moveBase4(-0.346670, 2.307890, 0.757347, 0.653002);
            Current::manipulateKnob(-1); // 5:13

        case 18 :
            Current::removeLid(jump); //5:51

        case 19 :
            Current::takePotFromIsland(jump); // 6:06

        case 20 :
            //pourReadyPopcornTable(jump); // 6:27 pour 6:50 pot down
            Current::pourReadyPopcorn(0);
            //
            //RobotHead::getInstance()->lookAtThreaded("/map", -1.7, 1.5, 0.899);

            //fotopourReadyPopcorn_(-1, 1.6,  0.997743, -0.067087, 0,2.4,false);

        case 21 :

            returnPotToHeater();
            //returnPotToSink();

        case 22 :
            Current::salt(jump); // 6:27 pour 6:50 pot down

            //salt_it(btVector3(.43, -1.46, .742 + 0.022 + 0.006), 0.129038 + 0.09, -0.702062 - 0.04, -0.655191, 0.75545);
            exit(0);

        }
    }





    return 0;
}


// deviation between odom_combined and map relative to base_link
