#include <pointcloud_segmentation/pointcloud_segmentation.h>
#include <iostream>
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double getRGB (float r, float g, float b)
{
  int res = (int (r * 255) << 16) | (int (g*255) << 8) | int (b*255);
  double rgb = *(float*)(&res);
  return (rgb);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PointCloudSegmentation::PointCloudSegmentation(): nh_("~")
{
  nh_.param("subscribe_pointcloud_topic", subscribe_pointcloud_topic_, std::string("/point_cloud"));
  nh_.param("subscribe_box_fit_topic", subscribe_box_fit_topic_, std::string("/box_estimation_node/box_marker"));
  nh_.param("model_type", model_type_, pcl::SACMODEL_PLANE);
  nh_.param("method_type", method_type_, pcl::SAC_RANSAC);
  nh_.param("set_axis", set_axis_, true);
  marker_published_ = false;
  point_cloud_received_ = false;
  ROS_INFO("pointcloud_segmentation node is up and running.");
  run();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PointCloudSegmentation::~PointCloudSegmentation()
{
  ROS_INFO("Shutting down pointcloud_segmentation node!");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PointCloudSegmentation::run()
{
  pointcloud_subscriber_ = nh_.subscribe(subscribe_pointcloud_topic_, 100, &PointCloudSegmentation::pointcloudSegmentationCallBack, this);
  box_fit_subscriber_ = nh_.subscribe(subscribe_box_fit_topic_, 100, &PointCloudSegmentation::boxfitCallBack, this);
  segment_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("segmented_plane", 100);
  pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("unsegmented_cloud", 100);

  spin_thread_ = boost::thread (boost::bind (&ros::spin));
  //ros::spin();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudSegmentation::spin()
{
  ros::Rate loop_rate(1);
  while( point_cloud_received_ == false)
    loop_rate.sleep();

    //while( nh_.ok() && input_cloud_.points.size() > 100 )
    //{
    loop_rate.sleep();
    segmentFloor();
    segmentCeiling();
    segmentPointCloud();
    loop_rate.sleep();
    //}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

pcl::PointCloud<pcl::PointSegmentation> PointCloudSegmentation::createSegmentedPointCloud(pcl::PointIndices inliers, double rgb )
{
  pcl::PointCloud<pcl::PointSegmentation> segmented_point_cloud;
  segmented_point_cloud.points.resize(inliers.indices.size());
  for(u_int i = 0 ; i < inliers.indices.size(); i++)
  {
    segmented_point_cloud.points[i].x = input_cloud_.points[inliers.indices[i]].x;
    segmented_point_cloud.points[i].y = input_cloud_.points[inliers.indices[i]].y;
    segmented_point_cloud.points[i].z = input_cloud_.points[inliers.indices[i]].z;
    segmented_point_cloud.points[i].intensity = input_cloud_.points[inliers.indices[i]].intensity;
    segmented_point_cloud.points[i].rgb = rgb;
    segmented_point_cloud.points[i].normal[0] = input_cloud_.points[inliers.indices[i]].normal[0];
    segmented_point_cloud.points[i].normal[1] = input_cloud_.points[inliers.indices[i]].normal[1];
    segmented_point_cloud.points[i].normal[2] = input_cloud_.points[inliers.indices[i]].normal[2];
    segmented_point_cloud.points[i].curvature = input_cloud_.points[inliers.indices[i]].curvature;
  }

  return (segmented_point_cloud);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

pcl::PointCloud<pcl::PointSegmentation> PointCloudSegmentation::createSegmentedPointCloud(pcl::PointCloud<pcl::PointXYZINormal> cloud, pcl::PointIndices inliers, double rgb )
{
  pcl::PointCloud<pcl::PointSegmentation> segmented_point_cloud;
  segmented_point_cloud.points.resize(inliers.indices.size());
  for(u_int i = 0 ; i < inliers.indices.size(); i++)
  {
    segmented_point_cloud.points[i].x = cloud.points[inliers.indices[i]].x;
    segmented_point_cloud.points[i].y = cloud.points[inliers.indices[i]].y;
    segmented_point_cloud.points[i].z = cloud.points[inliers.indices[i]].z;
    segmented_point_cloud.points[i].intensity = cloud.points[inliers.indices[i]].intensity;
    segmented_point_cloud.points[i].rgb = rgb;
    segmented_point_cloud.points[i].normal[0] = cloud.points[inliers.indices[i]].normal[0];
    segmented_point_cloud.points[i].normal[1] = cloud.points[inliers.indices[i]].normal[1];
    segmented_point_cloud.points[i].normal[2] = cloud.points[inliers.indices[i]].normal[2];
    segmented_point_cloud.points[i].curvature = cloud.points[inliers.indices[i]].curvature;
  }

  return (segmented_point_cloud);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

pcl::PointCloud<pcl::PointXYZINormal> PointCloudSegmentation::updatePointCloud( pcl::PointIndices inliers)
{
  int check = 0;
  pcl::PointCloud<pcl::PointXYZINormal> ret_cloud;

  for(u_int i = 0; i < input_cloud_.points.size(); i++)
  {
    for(u_int j = 0 ; j < inliers.indices.size(); j++)
    {
      if( (int)i == inliers.indices[j])
      {
        check = 1;
        break;
      }
    }
    if(check == 0)
    {
      ret_cloud.points.push_back(input_cloud_.points[i]);
    }
    else
      check = 0;
  }

  return (ret_cloud);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PointCloudSegmentation::publishPointCloud(pcl::PointCloud<pcl::PointXYZINormal> pointcloud)
{
  sensor_msgs::PointCloud2 pointcloud2_msg;
  pointcloud.width = 1;
  pointcloud.height = pointcloud.points.size();
  pcl::toROSMsg(pointcloud, pointcloud2_msg);
  pointcloud2_msg.header.frame_id = frame_id_;
  pointcloud2_msg.header.stamp = ros::Time::now();
  pointcloud_publisher_.publish(pointcloud2_msg);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PointCloudSegmentation::publishPointCloud(pcl::PointCloud<pcl::PointSegmentation> pointcloud)
{
  sensor_msgs::PointCloud2 pointcloud2_msg;
  pcl::toROSMsg(pointcloud, pointcloud2_msg);
  pointcloud2_msg.header.frame_id = frame_id_;
  pointcloud2_msg.header.stamp = ros::Time::now();
  segment_publisher_.publish(pointcloud2_msg);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudSegmentation::segmentVerticalPlanes()
{
  ROS_INFO("Looking for vertical planes perpendicular to the x-axis");
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  pcl::PointCloud<pcl::PointSegmentation> segmented_vertical_planes_x;

  seg_.setOptimizeCoefficients (true);
  seg_.setModelType (pcl::SACMODEL_PLANE);
  seg_.setMethodType (pcl::SAC_RANSAC);
  seg_.setDistanceThreshold (0.01);

  seg_.setAxis(axis_x_);
  seg_.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZINormal> > (input_cloud_));
  seg_.segment(inliers, coefficients);

  segmented_vertical_planes_x = createSegmentedPointCloud(inliers);
  publishPointCloud(segmented_vertical_planes_x);
  input_cloud_ = updatePointCloud(inliers);
  publishPointCloud(input_cloud_);

  if(inliers.indices.size() == 0 )
  {
    ROS_WARN("No planes found perpendicular to the x-axis");
  }
  ROS_INFO("Looking for vertical planes perpendicular to the y-axis");
  pcl::PointCloud<pcl::PointSegmentation> segmented_vertical_planes_y;

  seg_.setOptimizeCoefficients (true);
  seg_.setModelType (pcl::SACMODEL_PLANE);
  seg_.setMethodType (pcl::SAC_RANSAC);
  seg_.setDistanceThreshold (0.01);

  seg_.setAxis(axis_y_);
  seg_.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZINormal> > (input_cloud_));
  seg_.segment(inliers, coefficients);

  segmented_vertical_planes_y = createSegmentedPointCloud(inliers);
  publishPointCloud(segmented_vertical_planes_y);
  input_cloud_ = updatePointCloud(inliers);

  publishPointCloud(input_cloud_);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PointCloudSegmentation::segmentCeiling()
{
  ROS_INFO("Segmenting ceiling from point cloud of size %d", (int)input_cloud_.points.size());

  //first we remove the ceiling candidate points from the input cloud i.e. the points that lie between above 2.7 m
  pcl::PointCloud<pcl::PointXYZINormal> ceiling_candidate;
  pcl::PointIndices ceiling_points;

  for(u_int i = 0 ; i  < input_cloud_.points.size(); i++)
  {
    if(input_cloud_.points[i].z > 2.7)
    {
      ceiling_candidate.points.push_back(input_cloud_.points[i]);
      ceiling_points.indices.push_back(i);
    }
  }
  ceiling_candidate.width = 1;
  ceiling_candidate.height = ceiling_candidate.points.size();

  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  pcl::PointCloud<pcl::PointSegmentation> segmented_ceiling;

  seg_.setOptimizeCoefficients (true);
  seg_.setModelType (pcl::SACMODEL_ORIENTED_PLANE);
  seg_.setMethodType (pcl::SAC_RANSAC);
  seg_.setDistanceThreshold (0.01);

  seg_.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZINormal> > (ceiling_candidate));
  seg_.segment(inliers, coefficients);

  if(inliers.indices.size() > 0 )
  {
    ROS_INFO("Ceiling detected successfully");
    segmented_ceiling = createSegmentedPointCloud(ceiling_candidate, inliers);
    publishPointCloud(segmented_ceiling);
    ROS_INFO("Segmented floor published");
    input_cloud_ = updatePointCloud( ceiling_points);
    publishPointCloud(input_cloud_);
    ROS_INFO("Current unsegmented point cloud of size %d published", (int)input_cloud_.points.size());
  }
  else
  {
    ROS_WARN("No floor detected");
  }

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PointCloudSegmentation::segmentFloor()
{
  ROS_INFO("Segmenting floor from point cloud of size %d", (int)input_cloud_.points.size());

  //first we remove the floor candidate points from the input cloud i.e. the points that lie between 0 - 10 cm
  pcl::PointCloud<pcl::PointXYZINormal> floor_candidate;
  pcl::PointIndices floor_points;

  for(u_int i = 0 ; i  < input_cloud_.points.size(); i++)
  {
    if(input_cloud_.points[i].z < 0.1)
    {
      floor_candidate.points.push_back(input_cloud_.points[i]);
      floor_points.indices.push_back(i);
    }
  }
  floor_candidate.width = 1;
  floor_candidate.height = floor_candidate.points.size();

  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  pcl::PointCloud<pcl::PointSegmentation> segmented_floor;

  seg_.setOptimizeCoefficients (true);
  seg_.setModelType (pcl::SACMODEL_ORIENTED_PLANE);
  seg_.setMethodType (pcl::SAC_RANSAC);
  seg_.setDistanceThreshold (0.01);

  seg_.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZINormal> > (floor_candidate));
  seg_.segment(inliers, coefficients);

  if(inliers.indices.size() > 0 )
  {
    ROS_INFO("Floor detected successfully");
    segmented_floor = createSegmentedPointCloud(floor_candidate, inliers);
    publishPointCloud(segmented_floor);
    ROS_INFO("Segmented floor published");
    input_cloud_ = updatePointCloud( floor_points);
    publishPointCloud(input_cloud_);
    ROS_INFO("Current unsegmented point cloud of size %d published", (int)input_cloud_.points.size());
  }
  else
  {
    ROS_WARN("No ceiling detected");
  }

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PointCloudSegmentation::segmentPointCloud()
{
  ROS_INFO("Segmenting Point Cloud of size %d", (int) input_cloud_.points.size());
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  pcl::PointCloud<pcl::PointSegmentation> segmented_plane;

  seg_.setOptimizeCoefficients (true);
  seg_.setModelType (pcl::SACMODEL_PLANE);
  seg_.setMethodType (pcl::SAC_RANSAC);
  seg_.setDistanceThreshold (0.01);

  axis_x_[0] = 1; axis_x_[1] = 0 ; axis_x_[2] = 0;

  //seg_.setAxis(axis_x_);

  seg_.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZINormal> > (input_cloud_));
  seg_.segment(inliers, coefficients);

  if (inliers.indices.size () == 0)
  {
    ROS_ERROR ("Could not estimate a planar model for the given dataset.");
    exit(1);
  }

  segmented_plane = createSegmentedPointCloud(inliers);
  publishPointCloud(segmented_plane);
  ROS_INFO("Segmented Plane published");
  input_cloud_ = updatePointCloud(inliers);
  publishPointCloud(input_cloud_);
  ROS_INFO("Current unsegmented point cloud published");
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PointCloudSegmentation::boxfitCallBack(const visualization_msgs::Marker& marker)
{
  ROS_WARN("Marker msg received from box_fit2_node");
  btQuaternion qt(marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w);
  btMatrix3x3 box(qt);

  axis_x_[0] = box.getRow(0).x();
  axis_x_[1] = box.getRow(0).y();
  axis_x_[2] = box.getRow(0).z();

  axis_y_[0] = box.getRow(1).x();
  axis_y_[1] = box.getRow(1).y();
  axis_y_[2] = box.getRow(1).z();

  axis_z_[0] = box.getRow(2).x();
  axis_z_[1] = box.getRow(2).y();
  axis_z_[2] = box.getRow(2).z();

  ROS_INFO( "%f, %f, %f, %f, %f, %f, %f, %f, %f",
            box.getRow(0).x(), box.getRow(0).y(), box.getRow(0).z(),
            box.getRow(1).x(), box.getRow(1).y(), box.getRow(1).z(),
            box.getRow(2).x(), box.getRow(2).y(), box.getRow(2).z());

  marker_published_ = true;


}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PointCloudSegmentation::pointcloudSegmentationCallBack(const sensor_msgs::PointCloud2& pointcloud2_msg)
{
  //Local variables
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;

  ROS_INFO("Received a point cloud.");

  frame_id_ = pointcloud2_msg.header.frame_id;

  pcl::fromROSMsg(pointcloud2_msg, input_cloud_);

  point_cloud_received_ = true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_segmentation");
    PointCloudSegmentation pointcloud_segmentation;
    pointcloud_segmentation.spin();
    return(0);
}


