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
#include <sensor_msgs/PointCloud2.h>
//boost
#include <boost/thread/mutex.hpp>

//for point_cloud::fromROSMsg
#include <pcl/ros/conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

//for moving of the head
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

class PointCloudColorizer
{
  //subscribers/publishers
  ros::NodeHandle nh_;
  image_transport::CameraSubscriber image_sub_;
  ros::Subscriber cloud_sub_;

  tf::TransformListener tf_listener_;
  //opencv bridge
  image_transport::ImageTransport it_;
  sensor_msgs::CvBridge bridge_;
  std::string input_image_topic_, input_cloud_topic_, cloud_rgb_topic_;
  bool monochrome_, save_image_, cloud_received_;
  //pcl clouds
  pcl::PointCloud<pcl::PointXYZ> cloud_in_, cloud_in_tranformed_;
  cv::Mat image_;
  boost::mutex  cloud_lock_;
  std::vector <pcl::PointCloud<pcl::PointXYZ> > cloud_queue_;
  //pinhole cam model
  image_geometry::PinholeCameraModel cam_model_;
  sensor_msgs::PointCloud2::Ptr output_cloud_ptr_;
  ros::Publisher pub_output_;

  //point head client
  PointHeadClient* point_head_client_;
  //move offset
  double move_offset_y_min_, move_offset_y_max_, step_y_; 
  double move_offset_z_min_, move_offset_z_max_, step_z_;
  double move_offset_x_;
  ros::Time stamp_;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud_xyzrgb_;
public:
  PointCloudColorizer(ros::NodeHandle &n)
    :  nh_(n), it_(nh_)
  {
    //    nh_.param("input_image_topic", input_image_topic_, std::string("/wide_stereo/right/image_rect_color"));
    nh_.param("input_image_topic", input_image_topic_, std::string("/wide_stereo/right/image_rect_color_throttle"));
    nh_.param("input_cloud_topic", input_cloud_topic_, std::string("/points2_out"));
    nh_.param("monochrome", monochrome_, false);
    nh_.param("save_image", save_image_, false);
    nh_.param("cloud_rgb_topic", cloud_rgb_topic_, std::string("cloud_rgb"));
    image_sub_ = it_.subscribeCamera(input_image_topic_, 1, &PointCloudColorizer::image_cb, this);
    cloud_sub_= nh_.subscribe (input_cloud_topic_, 10, &PointCloudColorizer::cloud_cb, this);
    pub_output_ = nh_.advertise<sensor_msgs::PointCloud2> (cloud_rgb_topic_, 1);
    ROS_INFO("[PointCloudColorizer:] Subscribing to image topic %s", input_image_topic_.c_str());
    ROS_INFO("[PointCloudColorizer:] Subscribing to cloud topic %s", input_cloud_topic_.c_str());

    point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);
    //wait for head controller action server to come up 
    while(!point_head_client_->waitForServer(ros::Duration(5.0)) && ros::ok())
    {
      ROS_INFO("Waiting for the point_head_action server to come up");
    }
    cloud_received_ = false;

    //TODO: parametrize
    move_offset_y_min_ = -0.5;
    move_offset_y_max_ = 0.5;
    step_y_ = 0.1;
    move_offset_z_max_ = 1.2;
    move_offset_z_min_ = 0.0;
    step_z_ = 0.3;
    move_offset_x_ = 1.0;
    move_head("base_link", move_offset_x_, move_offset_y_max_, move_offset_z_max_);
  }

  ~PointCloudColorizer()
  {
    delete point_head_client_;
  }

  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& pc)
  {
    if (!cloud_received_)
    {
      pcl::PointXYZRGB point_xyzrgb;
      unsigned long size = pc->height*pc->width;
      ROS_INFO("[PointCloudColorizer: ] cloud received with points: %ld in frame %s", 
               size, pc->header.frame_id.c_str());
      pcl::fromROSMsg(*pc, cloud_in_);
      //TODO: check that the frame is valid
      
      cloud_xyzrgb_ = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>());
      cloud_xyzrgb_->width    = cloud_in_.points.size();
      cloud_xyzrgb_->height   = 1;
      cloud_xyzrgb_->is_dense = true;
      cloud_xyzrgb_->header = cloud_in_.header;
      for (unsigned long i = 0; i < cloud_in_.points.size(); i++)
      {
        point_xyzrgb.x = cloud_in_.points[i].x;
        point_xyzrgb.y = cloud_in_.points[i].y;
        point_xyzrgb.z = cloud_in_.points[i].z;
        //initialize RGB channel to red
        point_xyzrgb.rgb == getRGB(0.9, 0.0, 0.0);
        cloud_xyzrgb_->points.push_back(point_xyzrgb);
      }
      cloud_xyzrgb_->width =  cloud_xyzrgb_->points.size();
      ROS_INFO_STREAM("width, height: " << cloud_xyzrgb_->width << ", " << cloud_xyzrgb_->width 
                      << " is dense: " << cloud_xyzrgb_->is_dense);
      cloud_received_ = true;
    }
  }

  float
  getRGB (float r, float g, float b)
  {
    int32_t res = (int (r * 255) << 16) | (int (g*255) << 8) | int (b*255);
    float rgb = *reinterpret_cast<float*>(&res);
    return (rgb);
  }

  void image_cb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    //    ROS_INFO("[PointCloudColorizer: ] Image received in frame %s", 
    //        image_msg->header.frame_id.c_str());
      ROS_INFO("[PointCloudColorizer:] Image encoding %s", image_msg->encoding.c_str());
      //image_ = NULL;
      try 
      {
        image_ = bridge_.imgMsgToCv(image_msg, "passthrough");
      }
      catch (sensor_msgs::CvBridgeException& ex) {
        ROS_ERROR("[PointCloudColorizer:] Failed to convert image");
        return;
      }
      cam_model_.fromCameraInfo(info_msg);

      //HEAD moving part
      if (cloud_received_ && move_offset_y_max_ >= move_offset_y_min_ && move_offset_z_max_ >= move_offset_z_min_)
      {
        stamp_ = info_msg->header.stamp;
        process(image_);
        move_offset_y_max_ = move_offset_y_max_ - step_y_;
        move_head("base_link", 1.0, move_offset_y_max_, move_offset_z_max_);
        if (move_offset_y_max_ == move_offset_y_min_ && move_offset_z_max_ != move_offset_z_min_)
        {
          move_offset_z_max_ = move_offset_z_max_ - step_z_;
          //TODO: only works if min/max offsets are symetrical
          move_offset_y_max_ = -move_offset_y_min_;
        }
        sleep(1.0);
      }
      else
      {
        //TODO: ros::shutdown()??
        ROS_INFO("Done!");
      }
  }

  void move_head (std::string frame_id, double x, double y, double z)
  {
    //the goal message we will be sending
    pr2_controllers_msgs::PointHeadGoal goal;
    
    //the target point, expressed in the requested frame
    geometry_msgs::PointStamped point;
    point.header.frame_id = frame_id;
    point.point.x = x; point.point.y = y; point.point.z = z;
    goal.target = point;
    
    //we are pointing the wide_stereo camera frame 
    //(pointing_axis defaults to X-axis)
    goal.pointing_frame = "narrow_stereo_optical_frame";
    
    //take at least 5 seconds to get there
    goal.min_duration = ros::Duration(5);
    
    //and go no faster than 0.1 rad/s
    goal.max_velocity = 0.1;
    
    //send the goal
    point_head_client_->sendGoal(goal);
    
    //wait for it to get there
    point_head_client_->waitForResult();
  }

  void process (cv::Mat image)
  {
    //create a sequence storage for projected points 
    CvMemStorage* stor = cvCreateMemStorage (0);
    CvSeq* seq = cvCreateSeq (CV_SEQ_ELTYPE_POINT, sizeof (CvSeq), sizeof (CvPoint), stor);
    // pcl::transformPointCloud(cam_model_.tfFrame(), cloud_in_, cloud_in_tranformed_, tf_listener_);
    tf::StampedTransform transform;
    tf_listener_.lookupTransform(cam_model_.tfFrame(), cloud_in_.header.frame_id, stamp_, transform);
    pcl::transformPointCloud(cloud_in_, cloud_in_tranformed_, transform);

    ROS_INFO("[PointCloudColorizer:] cloud transformed to %s", cam_model_.tfFrame().c_str());


    if (cloud_in_tranformed_.points.size() == 0)
    {
      ROS_WARN("[PointCloudColorizer:] Point cloud points with size 0!!! Returning.");
      return;
    }

    //pcl::PointXYZRGB point_xyzrgb;
    for (unsigned long i = 0; i < cloud_in_tranformed_.points.size(); i++)
    {
      cv::Point3d pt_cv(cloud_in_tranformed_.points[i].x, cloud_in_tranformed_.points[i].y, 
                        cloud_in_tranformed_.points[i].z);
      cv::Point2d uv;
      cam_model_.project3dToPixel(pt_cv, uv);
      ROS_DEBUG_STREAM("points projected: " << round(uv.x) << " " << round(uv.y));
      //ROS_INFO_STREAM("uv.x: " << round(uv.x) << "uv.y: " << round(uv.y));
   
      //if point projection outside of image assign rgb white value
      if (uv.x < 0 || uv.x > image.cols || uv.y < 0 || uv.y > image.rows)
      {
        //        ROS_WARN("[PointCloudColorizer:] width out of bounds");
        //point_xyzrgb.rgb = getRGB(0.9, 0.0, 0.0);
        //continue;
      }
      else
      {
        ROS_DEBUG_STREAM("uv.x: " << round(uv.x) << "uv.y: " << round(uv.y));

        //uint8_t g = image.at<uint8_t>(round(uv.x), round(uv.y)*image.channels() + 3);
        //int32_t rgb = g | (g << 8) |  (g << 16);
	
        //VERSION for MONO
        if (monochrome_)
        {
          uint8_t g = image.at<uint8_t>(round(uv.y), round(uv.x));
          //int32_t rgb = g | (g << 8) |  (g << 16);
          int32_t rgb = (g << 16) | (g << 8) | g;
          //          point_xyzrgb.rgb = *reinterpret_cast<float*>(&rgb);
          if (cloud_xyzrgb_->points[i].rgb != getRGB(0.9, 0.0, 0.0))
            cloud_xyzrgb_->points[i].rgb = *reinterpret_cast<float*>(&rgb);
        }
        //VERSION for BGR8
        else
        {
          const cv::Vec3b& bgr = image.at<cv::Vec3b>(round(uv.y), round(uv.x));
          int32_t rgb_packed = (bgr[0] << 16) | (bgr[1] << 8) | bgr[2];
          //point_xyzrgb.rgb = *reinterpret_cast<float*>(&rgb_packed);
          if (cloud_xyzrgb_->points[i].rgb != getRGB(0.9, 0.0, 0.0))
            cloud_xyzrgb_->points[i].rgb = *reinterpret_cast<float*>(&rgb_packed);

        }
      }
      //fill in points
      //point_xyzrgb.x = cloud_in_tranformed_.points[i].x;
      //point_xyzrgb.y = cloud_in_tranformed_.points[i].y;
      //point_xyzrgb.z = cloud_in_tranformed_.points[i].z;
      //TODO:fix me
      //if (point_xyzrgb.rgb == getRGB(0.9, 0.0, 0.0))
      //cloud_xyzrgb_->points.push_back(point_xyzrgb);
    }
    if (save_image_)
    {
      std::stringstream stamp;
      stamp << cloud_in_tranformed_.header.stamp;
      cv::imwrite(stamp.str() + ".png", image);
    }
    //cloud_xyzrgb_->width =  cloud_xyzrgb_->points.size();
    boost::shared_ptr< sensor_msgs::PointCloud2> output_cloud = boost::shared_ptr<sensor_msgs::PointCloud2>(new sensor_msgs::PointCloud2()); 
    pcl::toROSMsg (*cloud_xyzrgb_, *output_cloud);
    ROS_INFO("Publishing PointXZYRGB cloud with %ld points in frame %s", cloud_xyzrgb_->points.size(), 
             output_cloud->header.frame_id.c_str());
    pub_output_.publish (output_cloud);
    cvClearSeq(seq);
    cvClearMemStorage(stor);
  } 
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_colorizer_node");
  ros::NodeHandle n("~");
  PointCloudColorizer pp(n);
  ros::spin();
  //pp.spin();
}
