#ifndef IMAGE_ALGOS_H
#define IMAGE_ALGOS_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>

namespace image_algos
{


class ImageAlgo
{
 public:
  ImageAlgo () {};
  
  typedef void OutputType;
  typedef sensor_msgs::Image InputType;
  static std::string default_output_topic () {return std::string("");};
  static std::string default_input_topic () {return std::string("");};
  static std::string default_node_name () {return std::string("");};
  
  virtual void init (ros::NodeHandle&) = 0;

  virtual void pre () = 0;
  virtual void post () = 0;
  virtual std::vector<std::string> requires () = 0;
  virtual std::vector<std::string> provides () = 0;
//  virtual std::string process (const boost::shared_ptr<const InputType>&) = ;
  OutputType output ();
};

template <class algo>
  class ImageAlgoNode
{
 public:
  ImageAlgoNode (ros::NodeHandle& nh, algo &alg)
  : nh_ (nh), a (alg)
  {
    //check if input topic is advertised
    std::vector<ros::master::TopicInfo> t_list;
    bool topic_found = false;
    ros::master::getTopics (t_list);
    for (std::vector<ros::master::TopicInfo>::iterator it = t_list.begin (); it != t_list.end (); it++)
    {
      if (it->name == a.default_input_topic())
      {
        topic_found = true;
        break;
      }
    }
    if (!topic_found)
      ROS_WARN ("Trying to subscribe to %s, but the topic doesn't exist!", a.default_input_topic().c_str());

    pub_ = nh_.advertise <typename algo::OutputType> (a.default_output_topic (), 5);
    sub_ = nh_.subscribe (a.default_input_topic (), 1, &ImageAlgoNode<algo>::input_cb, this);
   
    ROS_INFO("ImageAlgoNode (%s) created. SUB [%s], PUB[%s]",
             a.default_node_name().c_str(),
             a.default_input_topic().c_str(), 
	     a.default_output_topic().c_str());
    a.init (nh_);
  }

  void input_cb (const boost::shared_ptr<const typename algo::InputType> &input)
  {
    ROS_INFO("Received message.");
    a.pre();
    ROS_INFO("Algo.pre () returned.");
    ROS_INFO("");
    a.process (input);
    ROS_INFO("");
    ROS_INFO("Processed message.");
    a.post();
    ROS_INFO("Algo.post () returned.");
    pub_.publish (a.output());
    ROS_INFO("Published result message.\n");
  }
  
  ros::NodeHandle& nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

  algo& a;
};

template <class algo>
  int standalone_node (int argc, char* argv[])
{
  ros::init (argc, argv, algo::default_node_name());
  algo a;
  ros::NodeHandle nh ("~");
  ImageAlgoNode<algo> c(nh, a);

  ros::spin ();

  return (0);
}

}
#endif

