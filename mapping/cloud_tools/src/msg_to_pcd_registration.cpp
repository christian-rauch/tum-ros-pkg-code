// #include <unistd.h>

#include <ctime>
#include <ros/node_handle.h>
#include <joy/Joy.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud.h>
#include <point_cloud_mapping/cloud_io.h>

class MsgToPCD
{
  protected:
    ros::NodeHandle nh_;
    std::string input_cloud_topic_;
    std::string input_joy_topic_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber joy_sub_;
    int counter_;
    boost::mutex lock;
//     std::vector<sensor_msgs::PointCloudConstPtr> all_clouds_;
    sensor_msgs::PointCloud all_points_;
    sensor_msgs::PointCloudConstPtr cloud_in_;

  public:
    MsgToPCD () : counter_(0)
    {
      nh_.param ("input_cloud_topic", input_cloud_topic_, std::string("cloud_pcd"));
      nh_.param ("input_joy_topic", input_joy_topic_, std::string("wii"));
      cloud_sub_ = nh_.subscribe (input_cloud_topic_, 1, &MsgToPCD::cloud_cb, this);
      joy_sub_ = nh_.subscribe (input_joy_topic_, 1, &MsgToPCD::joy_cb, this);
    }
   
    bool append_cur_cloud ()
    {
      if (cloud_in_->channels.size () != all_points_.channels.size ())
        return false;

      for (unsigned int i = 0; i < cloud_in_->channels.size (); i++)
        if (cloud_in_->channels[i].name != all_points_.channels[i].name)
          return false;
      
      for (unsigned int i = 0; i < cloud_in_->channels.size (); i++)
        all_points_.channels[i].values.insert (all_points_.channels[i].values.end (), 
                    cloud_in_->channels[i].values.begin (), cloud_in_->channels[i].values.end ());

      all_points_.points.insert (all_points_.points.end (), cloud_in_->points.begin (), cloud_in_->points.end ());
      return true;
    }

    void registration ()
    {
      lock.lock ();
      if (all_points_.points.size() == 0)
      {
        all_points_.channels = cloud_in_->channels;
        all_points_.points = cloud_in_->points;
        all_points_.header.frame_id = cloud_in_->header.frame_id;
        return;
      }
      
      // minimize

      if (!append_cur_cloud ())
        ROS_WARN ("Could not append cloud. Check if channels are similar!");
      lock.unlock ();
    }
    
    void
      joy_cb (const joy::Joy::ConstPtr& j)
    {
      if (j->buttons[0] == 1)
      {
        lock.lock ();
        ROS_INFO ("Joystick message received on %s. Registering last point cloud", input_joy_topic_.c_str ());


        counter_ ++;
        lock.unlock ();
      }
    }
    
    void
      cloud_cb (const sensor_msgs::PointCloudConstPtr& cloud)
    {
      ROS_INFO ("PointCloud message received on %s with %d points", input_cloud_topic_.c_str (), (int)cloud->points.size());
      lock.lock ();
      cloud_in_ = cloud;
      lock.unlock ();
    }

    bool 
      spin ()
    {
      while (ros::ok())
      {
        ros::spinOnce ();
        if (counter_ > 0)
          return true;
      }
      return true;
    }
};

int main (int argc, char* argv[])
{
  ros::init (argc, argv, "msg_to_pcd_registration");

  MsgToPCD n;
  n.spin ();

  return (0);
}

