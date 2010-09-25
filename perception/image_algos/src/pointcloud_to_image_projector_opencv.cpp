#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <image_geometry/pinhole_camera_model.h>
//for pcl::transformPointCloud
#include <pcl_tf/transforms.h>
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
//#include <pcl/io/pcd_io.h>

class PointCloudToImageProjector
{
  //subscribers/publishers
  ros::NodeHandle nh_;
  image_transport::Publisher image_pub_;
  image_transport::CameraSubscriber image_sub_;
  ros::Subscriber cloud_sub_;

  tf::TransformListener tf_listener_;
  //opencv bridge
  image_transport::ImageTransport it_;
  sensor_msgs::CvBridge bridge_;
  std::string input_image_topic_, input_cloud_topic_, output_cluster_topic_, output_image_topic_;
  //pcl clouds
  pcl::PointCloud<pcl::PointXYZ> cloud_in_, cloud_in_tranformed_;
  IplImage* image_;
  boost::mutex  cloud_lock_;
  std::vector <pcl::PointCloud<pcl::PointXYZ> > cloud_queue_;
  //pinhole cam model
  image_geometry::PinholeCameraModel cam_model_;

public:
  PointCloudToImageProjector(ros::NodeHandle &n)
    :  nh_(n), it_(nh_)
  {
    nh_.param("input_image_topic", input_image_topic_, std::string("/wide_stereo/right/image_rect_color"));
    nh_.param("input_cloud_topic", input_cloud_topic_, std::string("/laser_table_detector/extract_object_clusters/output"));
    nh_.param("output_cluster_topic", output_cluster_topic_, std::string("output_cluster"));
    nh_.param("output_image_topic", output_image_topic_, std::string("image_with_projected_cluster"));
    image_sub_ = it_.subscribeCamera(input_image_topic_, 1, &PointCloudToImageProjector::image_cb, this);
    cloud_sub_= nh_.subscribe (input_cloud_topic_, 10, &PointCloudToImageProjector::cloud_cb, this);
    image_pub_ = it_.advertise(output_image_topic_, 1);
  }

  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& pc)
  {
    unsigned long size = pc->height*pc->width;
    ROS_INFO("[PointCloudToImageProjector: ] cloud received with points: %ld in frame %s", 
             size, pc->header.frame_id.c_str());
    pcl::fromROSMsg(*pc, cloud_in_);
    boost::mutex::scoped_lock lock(cloud_lock_);
    cloud_queue_.push_back(cloud_in_);
  }


  void image_cb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    ROS_INFO("[PointCloudToImageProjector: ] Image received in frame %s", 
             image_msg->header.frame_id.c_str());
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
        process(cloud_queue_[i], image_);
      cloud_queue_.clear();
    }
  }

  void process (pcl::PointCloud<pcl::PointXYZ> &cloud_in, IplImage *image)
  {
    pcl::transformPointCloud(cam_model_.tfFrame(), cloud_in, cloud_in_tranformed_, tf_listener_);
    ROS_INFO("[PointCloudToImageProjector:] cloud transformed to %s", cam_model_.tfFrame().c_str());
    //create a sequence storage for projected points 
    CvMemStorage* stor = cvCreateMemStorage (0);
    CvSeq* seq = cvCreateSeq (CV_SEQ_ELTYPE_POINT, sizeof (CvSeq), sizeof (CvPoint), stor);
    if (cloud_in_tranformed_.points.size() == 0)
    {
      ROS_WARN("[PointCloudToImageProjector:] Point cloud points with size 0!!! Returning.");
      return;
    }
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
        ROS_WARN("[PointCloudToImageProjector:] width out of bounds");
        continue;
      }
      if (uv.y < 0 || uv.y > image->height)
      {
        ROS_WARN("[PointCloudToImageProjector:] height out of bounds");
        continue;
      }
      //static const int RADIUS = 3;
      //cvCircle(image, uv, RADIUS, CV_RGB(255,0,0), -1);
      CvPoint pt;
      pt.x = uv.x;
      pt.y = uv.y;
      cvSeqPush( seq, &pt );
    }
    
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
    IplImage *subimage;
    cvSetImageROI(image,rect);
    // sub-image
    subimage = cvCreateImage( cvSize(rect.width, rect.height), image->depth, image->nChannels );
    cvCopy(image, subimage);
    cvResetImageROI(image); // release image ROI
    image_pub_.publish(bridge_.cvToImgMsg(subimage, "rgb8"));
    ROS_INFO("[PointCloudToImageProjector:] image published to %s", output_image_topic_.c_str());
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
