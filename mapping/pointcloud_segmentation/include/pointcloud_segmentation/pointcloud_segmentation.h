#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

//Segmentation includes
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/ModelCoefficients.h"

//Point type
#include "pointcloud_segmentation/pointcloud_segmentation_point_types.h"

//Other includes
#include "visualization_msgs/Marker.h"
#include "tf/tf.h"
#include <boost/thread.hpp>

class PointCloudSegmentation
{
  public:
    PointCloudSegmentation();
    ~PointCloudSegmentation();
    void pointcloudSegmentationCallBack(const sensor_msgs::PointCloud2& pointcloud);
    void boxfitCallBack(const visualization_msgs::Marker& marker);
    void run();

    pcl::PointCloud<pcl::PointXYZINormal> updatePointCloud( pcl::PointIndices inliers);
    pcl::PointCloud<pcl::PointSegmentation> createSegmentedPointCloud( pcl::PointIndices inliers, double rgb = 0.1);
    pcl::PointCloud<pcl::PointSegmentation> createSegmentedPointCloud( pcl::PointCloud<pcl::PointXYZINormal> cloud, pcl::PointIndices inliers, double rgb = 0.1);
    void publishPointCloud(pcl::PointCloud<pcl::PointSegmentation> pointcloud);
    void publishPointCloud(pcl::PointCloud<pcl::PointXYZINormal> pointcloud);
    void segmentPointCloud();
    void segmentFloor();
    void segmentCeiling();
    void segmentVerticalPlanes();
    void spin();

  private:
    ros::NodeHandle nh_;
    std::string subscribe_pointcloud_topic_, subscribe_box_fit_topic_, frame_id_;
    ros::Subscriber pointcloud_subscriber_;
    ros::Subscriber box_fit_subscriber_;
    ros::Publisher pointcloud_publisher_;
    ros::Publisher segment_publisher_;
    boost::thread spin_thread_;
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZINormal> seg_;
    Eigen3::Vector3f axis_x_, axis_y_, axis_z_;
    int label_, model_type_, method_type_;
    bool set_axis_, marker_published_, point_cloud_received_;
    pcl::PointCloud<pcl::PointXYZINormal> input_cloud_;

};


