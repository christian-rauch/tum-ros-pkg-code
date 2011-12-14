
#include <ias_drawer_executive/Kinect.h>
#include <ias_drawer_executive/Geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>

#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>


Kinect *Kinect::instance_ = 0;


void Kinect::getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string frame_id, ros::Time after)
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
    tf::Stamped<tf::Pose> net_stamped = Geometry::getPose(frame_id.c_str(),"/openni_rgb_optical_frame");
    tf::Transform net_transform;
    net_transform.setOrigin(net_stamped.getOrigin());
    net_transform.setRotation(net_stamped.getRotation());

    sensor_msgs::PointCloud2 pct; //in map frame

    pcl_ros::transformPointCloud(frame_id.c_str(),net_transform,pc,pct);
    pct.header.frame_id = frame_id.c_str();

    pcl::fromROSMsg(pct, *cloud);
}

void Kinect::getCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string frame_id, ros::Time after)
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
    tf::Stamped<tf::Pose> net_stamped = Geometry::getPose(frame_id.c_str(),"/openni_rgb_optical_frame");
    tf::Transform net_transform;
    net_transform.setOrigin(net_stamped.getOrigin());
    net_transform.setRotation(net_stamped.getRotation());

    sensor_msgs::PointCloud2 pct; //in map frame

    pcl_ros::transformPointCloud(frame_id.c_str(),net_transform,pc,pct);
    pct.header.frame_id = frame_id.c_str();

    pcl::fromROSMsg(pct, *cloud);
}
