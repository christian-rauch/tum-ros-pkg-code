#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>
#include <sensor_msgs/PointCloud.h>
//boost
#include <boost/thread/mutex.hpp>

class PointCloudToImageProjector
{
  //subscribers/publishers
  ros::NodeHandle nh_;
  image_transport::Publisher image_pub_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber cloud_sub_;

  tf::TransformListener tf_listener_;
  //opencv bridge
  image_transport::ImageTransport it_;
  sensor_msgs::CvBridge bridge_;
  std::string input_image_topic_, input_cloud_topic_, output_cluster_topic_, output_image_topic_;
  //calibration parameters for svistec cameras
  double focal_length_, proj_center_x_, proj_center_y_, pix_size_x_, pix_size_y_;
  //ROS msgs
  sensor_msgs::PointCloud cloud_in_;
  IplImage* image_;
  std::string origin_, interim_, child_;
  boost::mutex  cloud_lock_;
  std::vector <sensor_msgs::PointCloud> cloud_queue_;

public:
  PointCloudToImageProjector(ros::NodeHandle &n)
    :  nh_(n), it_(nh_)
  {
    nh_.param("origin", origin_, std::string("/LeftEyeCalc"));
    nh_.param("child", child_, std::string("/sr4"));
    nh_.param("input_image_topic", input_image_topic_, std::string("/cop/left/camera"));
    nh_.param("input_cloud_topic", input_cloud_topic_, std::string("/swissranger_test/cloud_sr"));
    nh_.param("output_cluster_topic", output_cluster_topic_, std::string("output_cluster"));
    nh_.param("output_image_topic", output_image_topic_, std::string("image_with_projected_cluster"));
    nh_.param("focal_length", focal_length_, 0.00641331974023884);
    nh_.param("proj_center_x", proj_center_x_, 833.248646581603);
    nh_.param("proj_center_y", proj_center_y_, 661.107370424523);
    nh_.param("pix_size_x", pix_size_x_, 7.43100103980579e-06);
    nh_.param("pix_size_y", pix_size_y_, 7.4e-06);
    image_sub_ = it_.subscribe(input_image_topic_, 1, &PointCloudToImageProjector::image_cb, this);
    cloud_sub_= nh_.subscribe (input_cloud_topic_, 10, &PointCloudToImageProjector::cloud_cb, this);
    image_pub_ = it_.advertise(output_image_topic_, 1);
  }

  void Project3DPoint(const double &x, const double &y, const double &z, int &row, int &column)
  {
    double temp1,temp2;
    if (focal_length_ > 0.0)
    {
      temp1 = focal_length_ * (x / z );
      temp2 = focal_length_ * (y / z );
    }
    else
    {
      temp1 = x;
      temp2 = y;
    }
    column = ( temp1 / pix_size_x_ ) + proj_center_x_;
    row = ( temp2 / pix_size_y_ ) + proj_center_y_;
  }

   void cloud_cb (const sensor_msgs::PointCloudConstPtr& pc)
    {
      ROS_INFO("[PointCloudToImageProjector: ] cloud received with points: %ld", pc->points.size());
      try
      {
        tf_listener_.transformPointCloud(origin_, *pc, cloud_in_);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("[PointCloudToImageProjector:] %s",ex.what());
      }
      ROS_INFO("[PointCloudToImageProjector:] cloud transformed to %s", origin_.c_str());
      boost::mutex::scoped_lock lock(cloud_lock_);
      cloud_queue_.push_back(cloud_in_);
    }


  void image_cb(const sensor_msgs::ImageConstPtr& image_msg)
  {
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
      for (unsigned long i = 0; i < cloud_queue_.size(); i++)
        process(cloud_queue_[i], image_);
      cloud_queue_.clear();
    }
  }

  void process (sensor_msgs::PointCloud &cloud_in, IplImage *image)
  {
    //create a sequence storage for projected points 
    CvMemStorage* stor = cvCreateMemStorage (0);
    CvSeq* seq = cvCreateSeq (CV_SEQ_ELTYPE_POINT, sizeof (CvSeq), sizeof (CvPoint), stor);
    if (cloud_in.points.size() == 0)
    {
      ROS_WARN("[PointCloudToImageProjector:] Point cloud points with size 0!!! Returning.");
      return;
    }
    for (unsigned long i = 0; i < cloud_in.points.size(); i++)
    {
      cv::Point3d pt_cv(cloud_in.points[i].x, cloud_in.points[i].y, cloud_in.points[i].z);
      CvPoint uv;
      Project3DPoint(pt_cv.x, pt_cv.y, pt_cv.z, uv.y, uv.x);
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
      cvSeqPush( seq, &uv );
    }
    
    CvRect rect = cvBoundingRect(seq);
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
    cvClearMemStorage( stor );
    
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
  ros::init(argc, argv, "pointcloud_to_image_projector_node");
  ros::NodeHandle n("~");
  PointCloudToImageProjector pp(n);
  ros::spin();
  //pp.spin();
}
