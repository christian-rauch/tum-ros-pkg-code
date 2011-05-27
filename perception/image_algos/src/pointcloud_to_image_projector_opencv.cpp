#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
//#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
//for pcl::transformPointCloud
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>
#include <sensor_msgs/PointCloud.h>
//boost
#include <boost/thread/mutex.hpp>

//for point_cloud::fromROSMsg
#include <pcl/ros/conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "pcl/filters/extract_indices.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/kdtree/kdtree_flann.h"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include "pcl/common/common.h"

#include <geometry_msgs/PolygonStamped.h>
typedef pcl::PointXYZ Point;
typedef pcl::KdTree<Point>::Ptr KdTreePtr;

class PointCloudToImageProjector
{
  //subscribers/publishers
  ros::NodeHandle nh_;
  image_transport::Publisher image_pub_;
  image_transport::CameraSubscriber image_sub_;
  ros::Subscriber cloud_sub_;
  ros::Publisher polygon_pub_;

  tf::TransformListener tf_listener_;
  //opencv bridge
  image_transport::ImageTransport it_;
  sensor_msgs::CvBridge bridge_;
  std::string input_image_topic_, input_cloud_topic_, output_cluster_topic_, output_image_topic_, output_polygon_points_topic_;
  //pcl clouds
  pcl::PointCloud<pcl::PointXYZ> cloud_in_, cloud_in_tranformed_;
  IplImage* image_;
  boost::mutex  cloud_lock_;
  std::vector <pcl::PointCloud<pcl::PointXYZ> > cloud_queue_, cluster_queue_;
  //pinhole cam model
  image_geometry::PinholeCameraModel cam_model_;
  int offset_, nr_clusters_;
  pcl::EuclideanClusterExtraction<Point> cluster_;
  int object_cluster_min_size_, counter_, count_images_;
  double object_cluster_tolerance_;
  KdTreePtr clusters_tree_;
  bool cluster_cloud_, save_data_;
  std::string image_cluster_name_, location_;
  pcl::PCDWriter pcd_writer_;
  pcl::PointXYZ point_center_, point_min_, point_max_;
  geometry_msgs::PolygonStamped polygon_points_;

public:
  PointCloudToImageProjector(ros::NodeHandle &n)
    :  nh_(n), it_(nh_)
  {
    nh_.param("input_image_topic", input_image_topic_, std::string("/wide_stereo/right/image_rect_color"));
    nh_.param("input_cloud_topic", input_cloud_topic_, std::string("/laser_table_detector/extract_object_clusters/output"));
    nh_.param("output_cluster_topic", output_cluster_topic_, std::string("output_cluster"));
    nh_.param("output_image_topic", output_image_topic_, std::string("image_with_projected_cluster"));
    nh_.param("output_polygon_points_topic", output_polygon_points_topic_, std::string("polygon_points"));
    nh_.param("offset", offset_, 0);
    nh_.param("object_cluster_min_size", object_cluster_min_size_, 100);
    nh_.param("object_cluster_tolerance", object_cluster_tolerance_, 0.03);
    nh_.param("cluster_cloud", cluster_cloud_, false);
    nh_.param("image_cluster_name", image_cluster_name_, std::string(""));
    nh_.param("location", location_, std::string("island"));
    nh_.param("save_data", save_data_, false);
    nh_.param("nr_clusters_", nr_clusters_, 1);
    counter_ = count_images_ = 0;
    image_sub_ = it_.subscribeCamera(input_image_topic_, 1, &PointCloudToImageProjector::image_cb, this);
    cloud_sub_= nh_.subscribe (input_cloud_topic_, 10, &PointCloudToImageProjector::cloud_cb, this);
    image_pub_ = it_.advertise(output_image_topic_, 100);
    polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>(output_polygon_points_topic_, 1);
    clusters_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
    clusters_tree_->setEpsilon (1);
  }

  void cluster_cloud (pcl::PointCloud<pcl::PointXYZ> &cloud_in, std::vector<pcl::PointCloud<pcl::PointXYZ> > &cluster_queue)
  {
    std::vector<pcl::PointIndices> clusters;
    cluster_.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud_in));
    cluster_.setClusterTolerance (object_cluster_tolerance_);
    cluster_.setMinClusterSize (object_cluster_min_size_);
    //    cluster_.setMaxClusterSize (object_cluster_max_size_);
    cluster_.setSearchMethod (clusters_tree_);
    cluster_.extract (clusters);
    
    ROS_INFO("[PointCloudToImageProjector: ] Found clusters: %ld", clusters.size());
    
    cluster_queue.resize(clusters.size());
    for (unsigned int i = 0; i < clusters.size(); i++)
      {
	pcl::copyPointCloud (cloud_in, clusters[i], cluster_queue[i]);
      }
  }

  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& pc)
  {
    unsigned long size = pc->height*pc->width;
    ROS_INFO("[PointCloudToImageProjector: ] cloud received with points: %ld in frame %s", 
             size, pc->header.frame_id.c_str());
    pcl::fromROSMsg(*pc, cloud_in_);
    boost::mutex::scoped_lock lock(cloud_lock_);
    cloud_queue_.push_back(cloud_in_);
    counter_++;
  }


  void image_cb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    //ROS_INFO("[PointCloudToImageProjector: ] Image received in frame %s", 
    //       image_msg->header.frame_id.c_str());
    if (!cloud_queue_.empty())
    {
      image_ = NULL;
      try 
      {
        image_ = bridge_.imgMsgToCv(image_msg, "rgb8");
      }
      catch (sensor_msgs::CvBridgeException& ex) {
        ROS_ERROR("[PointCloudToImageProjector:] Failed to convert image");
        return;
      }
      cam_model_.fromCameraInfo(info_msg);
      for (unsigned long i = 0; i < cloud_queue_.size(); i++)
      {
        if (cluster_cloud_)
        {
          cluster_cloud (cloud_queue_[i], cluster_queue_);
          for (unsigned long j = 0; j < cluster_queue_.size(); j++)
          {
            process(cluster_queue_[j], image_);
          }
          cluster_queue_.clear();
        }
        else
        {
          process(cloud_queue_[i], image_);
        }
      }
      cloud_queue_.clear();
    }
  }

  // bool get_projected_points (image_algos::GetProjectedPoints &req, image_algos::GetProjectedPoints &res)
  //   {
  //     boost::mutex::scoped_lock lock(polygon_points_lock_);
  //     res.points = ;
  //     return true;
  //   }

  void process (pcl::PointCloud<pcl::PointXYZ> &cloud_in, IplImage *image)
  {
    //sleep(2.0);
    if (cloud_in.width == 1 && cloud_in.height == 1)
      {
	// cv::Mat token(480, 1, CV_8U);
	// cv_bridge::CvImagePtr cv_ptr;
	// cv_ptr->image = token;
	// image_pub_.publish(cv_ptr->toImageMsg());
	// //	image_pub_.publish(bridge_.toImageMsg(token, "rgb8"));
	IplImage * token = cvCreateImage(cvSize(2, 480), image->depth, image->nChannels);
	cvZero(token);
	image_pub_.publish(bridge_.cvToImgMsg(token));
	ROS_INFO("[PointCloudToImageProjector:] Published TOKEN Image");
	cvReleaseImage(&token);
	return;
      }
    pcl_ros::transformPointCloud(cam_model_.tfFrame(), cloud_in, cloud_in_tranformed_, tf_listener_);
    ROS_INFO("[PointCloudToImageProjector:] cloud transformed to %s", cam_model_.tfFrame().c_str());
    //create a sequence storage for projected points 
    CvMemStorage* stor = cvCreateMemStorage (0);
    CvSeq* seq = cvCreateSeq (CV_SEQ_ELTYPE_POINT, sizeof (CvSeq), sizeof (CvPoint), stor);
    if (cloud_in_tranformed_.points.size() == 0)
    {
      ROS_WARN("[PointCloudToImageProjector:] Point cloud points with size 0!!! Returning.");
      return;
    }
    
    polygon_points_.polygon.points.clear();
    geometry_msgs::Point32 tmp_pt;
    for (unsigned long i = 0; i < cloud_in_tranformed_.points.size(); i++)
    {
      cv::Point3d pt_cv(cloud_in_tranformed_.points[i].x, cloud_in_tranformed_.points[i].y, 
                        cloud_in_tranformed_.points[i].z);
      cv::Point2d uv;
      cam_model_.project3dToPixel(pt_cv, uv);
      ROS_DEBUG_STREAM("points projected: " << uv.x << " " << uv.y);
      //     Project3DPoint(pt_cv.x, pt_cv.y, pt_cv.z, uv.y, uv.x);
      if (uv.x < 0 || uv.x > image->width)
      {
        //ROS_WARN("[PointCloudToImageProjector:] width out of bounds");
        continue;
      }
      if (uv.y < 0 || uv.y > image->height)
      {
        //ROS_WARN("[PointCloudToImageProjector:] height out of bounds");
        continue;
      }
      //static const int RADIUS = 3;
      //cvCircle(image, uv, RADIUS, CV_RGB(255,0,0), -1);
      CvPoint pt;
      pt.x = uv.x;
      pt.y = uv.y;
      tmp_pt.x = uv.x;
      tmp_pt.y = uv.y;
      tmp_pt.z = 0.0;
      polygon_points_.polygon.points.push_back(tmp_pt);
      cvSeqPush( seq, &pt );
    }
    polygon_points_.header.stamp = ros::Time::now();
    polygon_points_.header.seq = 0;
    polygon_pub_.publish(polygon_points_);
    ROS_INFO("Polygon with %ld points published", polygon_points_.polygon.points.size());
    
    CvRect rect = cvBoundingRect(seq);

    ROS_DEBUG_STREAM("rect: " << rect.x << " " << rect.y << " " << rect.width << " " << rect.height);

    if (rect.width == 0 || rect.height == 0)
    {
      ROS_WARN("[PointCloudToImageProjector:] Rectangle's width = height = 0. This is impossible. Skipping ROI extraction.");
      return;
    }
    CvPoint pt1, pt2;
    pt1.x = rect.x;
    //check boundary, increase ROI for 10%
    if ((rect.x + (rect.width + 0.1*rect.width)) > image->width)
    {
      rect.width =  image->width + - rect.x;
    }
    else 
      rect.width = rect.width + 0.1 * rect.width;
    
    pt2.x = rect.x + rect.width;
    pt1.y = rect.y;
    
    //check boundary, increase ROI for 10%
    if ((rect.y + (rect.height + 0.1*rect.height)) > image->height)
    {
      rect.height =  image->height +  - rect.y;
    }
    else
      rect.height = rect.height + 0.1 * rect.height;
    pt2.y = rect.y+rect.height;
    
    //draw rectangle around the points
    //cvRectangle(image, pt1, pt2, CV_RGB(255,0,0), 3, 8, 0 );
    //cvClearMemStorage( stor );
    
    //get subimage, aka region of interest
    rect.x = rect.x - offset_;
    rect.y = rect.y - offset_;
    rect.height = rect.height + 2*offset_;
    rect.width = rect.width + 2*offset_;
    IplImage *subimage;
    cvSetImageROI(image,rect);
    // sub-image
    subimage = cvCreateImage( cvSize(rect.width, rect.height), image->depth, image->nChannels );
    cvCopy(image, subimage);
    cvResetImageROI(image); // release image ROI
    image_pub_.publish(bridge_.cvToImgMsg(subimage, "rgb8"));

    if (save_data_)
    {
      char counter_char[100];
      sprintf (counter_char, "%04d",  counter_);
      std::stringstream ss;
      std::stringstream ss_position;
      getMinMax3D (cloud_in, point_min_, point_max_);
      //Calculate the centroid of the cluster
      point_center_.x = (point_max_.x + point_min_.x)/2;
      point_center_.y = (point_max_.y + point_min_.y)/2;
      point_center_.z = (point_max_.z + point_min_.z)/2;
      ss_position << point_center_.x << "_" << point_center_.y << "_" << point_center_.z;

      ss << ros::Time::now();
      image_cluster_name_ = location_ + "/" + std::string(counter_char) + "/" +  "NAME" + "_" + ss.str() + "_" + ss_position.str();
     
      boost::filesystem::path outpath (location_ + "/" + std::string(counter_char));
      if (!boost::filesystem::exists (outpath))
      {
        if (!boost::filesystem::create_directories (outpath))
        {
          ROS_ERROR ("Error creating directory %s.", (location_ + "/" + std::string(counter_char)).c_str ());
          //return (-1);
        }
        ROS_INFO ("Creating directory %s", (location_ + "/" + std::string(counter_char)).c_str ());
      }
      
      pcd_writer_.write (image_cluster_name_ + ".pcd", cloud_in_tranformed_, true);
      cv::imwrite(image_cluster_name_ + ".png", subimage);
    }
    ROS_INFO("[PointCloudToImageProjector:] image with size %d %d published to %s", subimage->width, subimage->height, output_image_topic_.c_str());
    cvReleaseImage(&subimage);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_to_image_projector_opencv_node");
  ros::NodeHandle n("~");
  PointCloudToImageProjector pp(n);
  ros::spin();
  //pp.spin();
}
