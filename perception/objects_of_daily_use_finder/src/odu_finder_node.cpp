#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/CvBridge.h>
#include <vision_msgs/cop_answer.h>
#include "odu_finder.h"

class ODUFinderNode : public ODUFinder
{
private:
	// NODE
	ros::NodeHandle node_handle;
	ros::Subscriber image_subscriber;
	ros::Publisher template_publisher;
	ros::Publisher image_pub_;

	// VISUALIZATION
	sensor_msgs::CvBridge bridge;
    std::string image_topic, template_topic;
    std::string output_image_topic_;

public:
	ODUFinderNode(ros::NodeHandle &anode)
		: node_handle(anode)
    {
      //load or build database, equivalent to detect or build
      node_handle.param ("command", command, std::string("/load"));
      //path to database
      node_handle.param ("database_location", database_location, std::string("database/germandeli"));
      //path to training images
      node_handle.param ("images_folder", images_directory, std::string("data/germandeli"));
      //path to visualization images
      node_handle.param ("images_for_visualization_directory", images_for_visualization_directory, std::string(""));
      //input image topic
      node_handle.param ("image_topic", image_topic, std::string("/narrow_stereo/left/image_rect"));
      //template publisher for http://www.ros.org/wiki/cop
      node_handle.param ("template_topic", template_topic, std::string("TemplateName"));
      //how many best votes do we consider
      node_handle.param ("votes_count", votes_count, 10);
      //vocabulary trees parameters
      node_handle.param ("tree_k", tree_k, 5);
      node_handle.param ("tree_levels", tree_levels, 5);
      //min size of features when clustering
      node_handle.param ("min_cluster_size", min_cluster_size, 30);
      //when is object uknown (score goes from 0 (best) - 2 (worst)
      node_handle.param ("unknown_object_threshold", unknown_object_threshold, 0.3);
      //wait for #sliding_window_size frames before deciding on the classification result
      node_handle.param ("sliding_window_size", sliding_window_size, 10);
      //clustering yes or no
      node_handle.param ("enable_clustering", enable_clustering, 1);
      //acquire complete appearance models (suitable for e.g. images from germandeli)
      node_handle.param ("enable_incremental_learning", enable_incremental_learning, 0);
      //enable visualization
      node_handle.param ("enable_visualization", enable_visualization, 1);
      set_visualization(enable_visualization);
      //object id for http://www.ros.org/wiki/cop
      node_handle.param ("object_id", object_id, 700000);
      //radius adaptation parameters for clustering
      node_handle.param ("radius_adaptation_r_min", radius_adaptation_r_min, 200.0);
      node_handle.param ("radius_adaptation_r_max", radius_adaptation_r_max, 600.9);
      node_handle.param ("radius_adaptation_A", radius_adaptation_A, 800.0);
      node_handle.param ("radius_adaptation_K", radius_adaptation_K, 0.02);
      image_subscriber = node_handle.subscribe(image_topic, 1, &ODUFinderNode::image_callback, this);

      //template_publisher = node_handle.advertise<std_msgs::String>(template_topic, 1);
      //publish data for http://www.ros.org/wiki/cop
      template_publisher = node_handle.advertise<vision_msgs::cop_answer>(template_topic, 1);
      node_handle.param ("output_image_topic" , output_image_topic_ , std::string("object_found"));
  	  image_pub_ = node_handle.advertise<sensor_msgs::Image>(output_image_topic_,10);
    
      //shall we extract roi around the keypoints??
      node_handle.param ("extract_roi" , extract_roi_, false);

      if (enable_visualization)
      {
        cvNamedWindow("visualization", CV_WINDOW_AUTOSIZE);
        cvStartWindowThread();
      }

//		SiftParameters params = GetSiftParameters();
//		params.DoubleImSize = 0;
//		SetSiftParameters(params);

      //for visualization of feature points
      color_table[0] = cvScalar(255,		0,		0);
      color_table[1] = cvScalar(0,		255,	0);
      color_table[2] = cvScalar(0,		0,		255);
      color_table[3] = cvScalar(255,		255,	0);
      color_table[4] = cvScalar(255,		0,		255);
      color_table[5] = cvScalar(0,		255,	255);
      color_table[6] = cvScalar(255,		255,	255);
      color_table[7] = cvScalar(125,		255,	255);
      color_table[8] = cvScalar(255,		125,	255);
      color_table[9] = cvScalar(255,		255,	125);
      color_table[10] = cvScalar(125,		125,	255);
      color_table[11] = cvScalar(255,		125,	125);
      color_table[12] = cvScalar(125,		255,	125);
    }

private:
	void image_callback (const sensor_msgs::ImageConstPtr& received_image)
    {
      //if image is not de-bayered yet do it (needed for Prosilica image on PR2 robot)
      if (received_image->encoding.find("bayer") != std::string::npos)
      {
        cv::Mat tmpImage;
        const std::string& raw_encoding = received_image->encoding;
        int raw_type = CV_8UC1;
        if (raw_encoding == sensor_msgs::image_encodings::BGR8 || raw_encoding == sensor_msgs::image_encodings::RGB8)
          raw_type = CV_8UC3;
        const cv::Mat raw(received_image->height, received_image->width, raw_type,
                          const_cast<uint8_t*>(&received_image->data[0]), received_image->step);

        // Convert to color BGR
        int code = 0;
        if (received_image->encoding == sensor_msgs::image_encodings::BAYER_RGGB8)
          code = CV_BayerBG2BGR;
        else if (received_image->encoding == sensor_msgs::image_encodings::BAYER_BGGR8)
          code = CV_BayerRG2BGR;
        else if (received_image->encoding == sensor_msgs::image_encodings::BAYER_GBRG8)
          code = CV_BayerGR2BGR;
        else if (received_image->encoding == sensor_msgs::image_encodings::BAYER_GRBG8)
          code = CV_BayerGB2BGR;
        else
        {
          ROS_ERROR("[image_proc] Unsupported encoding '%s'", received_image->encoding.c_str());
          return;
        }
        cv::cvtColor(raw, tmpImage, code);

        cv::Mat tmp2;
        cv::cvtColor(tmpImage, tmp2, CV_BGR2GRAY);
        camera_image = cvCreateImage(cvSize(800, 600), IPL_DEPTH_8U, 1);
        IplImage ti;
        ti = tmp2;
        cvSetImageROI(&ti, cvRect(300, 300, 1848, 1450));
        cvResize(&ti, camera_image);
      }
      else
      {
        camera_image = bridge.imgMsgToCv(received_image, "mono8");
      }
      printf("\n\n");
      ROS_INFO("Image received!");
      //call main processing function
      std::string result = process_image(camera_image);

      //image_pub_.publish(bridge.cvToImgMsg(image_roi));
      image_pub_.publish(bridge.cvToImgMsg(camera_image));
      ROS_INFO("[ODUFinder:] image published to %s", output_image_topic_.c_str());
      cvReleaseImage(&image_roi);



      //msg.data = name.substr(0, name.find_last_of('.')).c_str();
      //publish the result to http://www.ros.org/wiki/cop
      if (result.length() > 0)
      {
        vision_msgs::cop_answer msg;
        vision_msgs::cop_descriptor cop_descriptor;
        vision_msgs::aposteriori_position aposteriori_position;
        msg.found_poses.push_back(aposteriori_position);
        msg.found_poses[0].models.push_back(cop_descriptor);
        //TODO: only one needed probably, ask Moritz
        msg.found_poses[0].models[0].type = "ODUFinder";
        msg.found_poses[0].models[0].sem_class = result;
        msg.found_poses[0].models[0].object_id = ++object_id;
        msg.found_poses[0].position = 0;
        template_publisher.publish(msg);
      }
    }
};

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "odu_finder");
	ros::NodeHandle node_handle("~");
	ODUFinderNode odu_finder(node_handle);
	int result = odu_finder.start();
	if(result != -1)
	  ros::spin();
	//odu_finder.write_stat_summary();
}
