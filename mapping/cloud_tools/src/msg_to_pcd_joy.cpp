// #include <unistd.h>

#include <ctime>
#include <ros/node.h>
#include <joy/Joy.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud.h>
#include <point_cloud_mapping/cloud_io.h>

class MsgToPCDJoy
{
  protected:
    ros::NodeHandle nh_;
    std::string input_cloud_topic_;
    std::string input_joy_topic_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber joy_sub_;
    int counter_;
    boost::mutex lock;
    sensor_msgs::PointCloudConstPtr cloud_in_;

  public:
    MsgToPCDJoy () : counter_(0)
    {
      nh_.param ("input_cloud_topic", input_cloud_topic_, std::string("shoulder_cloud"));
      nh_.param ("input_joy_topic", input_joy_topic_, std::string("/wii/wiimote"));
      cloud_sub_ = nh_.subscribe (input_cloud_topic_, 1, &MsgToPCDJoy::cloud_cb, this);
      joy_sub_ = nh_.subscribe (input_joy_topic_, 1, &MsgToPCDJoy::joy_cb, this);
      ROS_INFO ("subscribed to both topics.");
    }
    
    void
      joy_cb (const joy::Joy::ConstPtr& j)
    {
      if (j->buttons[3] == 1 && counter_ > 0)
      {
        lock.lock ();
        std::ostringstream filename;
        filename << "cloud_" << time (NULL) << "_" << getpid () << ".pcd";
        ROS_INFO ("Joystick message received on %s. Saving to %s", input_joy_topic_.c_str (), filename.str ().c_str ());
        cloud_io::savePCDFile (filename.str ().c_str (), *cloud_in_, true);
        lock.unlock ();
      }
      else
      {
        for (unsigned int i = 0; i < j->buttons.size(); i++)
          if (j->buttons[i] == 1)
            ROS_INFO ("Joystick button Nr. %i pressed.", i);
      }
    }
    
    void
      cloud_cb (const sensor_msgs::PointCloudConstPtr& cloud)
    {
      ROS_INFO ("PointCloud message received on %s with %d points", input_cloud_topic_.c_str (), (int)cloud->points.size());
      lock.lock ();
      cloud_in_ = cloud;
      counter_ ++;
      lock.unlock ();
    }

    bool 
      spin ()
    {
      while (ros::ok())
      {
        ros::spinOnce ();
//         if (counter_ > 0)
//           return true;
      }
      return true;
    }
};

int main (int argc, char* argv[])
{
  ros::init (argc, argv, "msg_to_pcd");

  MsgToPCDJoy n;
  n.spin ();

  return (0);
}

