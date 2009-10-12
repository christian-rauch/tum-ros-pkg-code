// #include <unistd.h>

#include <ctime>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud.h>
#include <point_cloud_mapping/cloud_io.h>

class MsgToPCD
{
  protected:
    ros::NodeHandle nh_;
    std::string input_cloud_topic_;
    ros::Subscriber cloud_sub_;
    int counter_;

  public:
    MsgToPCD () : counter_(0)
    {
      nh_.param ("input_cloud_topic", input_cloud_topic_, std::string("cloud_pcd"));       // 15 degrees
      cloud_sub_ = nh_.subscribe (input_cloud_topic_, 1, &MsgToPCD::cloud_cb, this);
    }
    
    void
      cloud_cb (const sensor_msgs::PointCloudConstPtr& cloud)
    {
      std::ostringstream filename;
      filename << "cloud_" << time (NULL) << "_" << getpid () << ".pcd";
      ROS_INFO ("PointCloud message received on %s with %d points. Saving to %s", input_cloud_topic_.c_str (), (int)cloud->points.size (), filename.str ().c_str ());
      cloud_io::savePCDFile (filename.str ().c_str (), *cloud, true);
      counter_ ++;
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
  ros::init (argc, argv, "msg_to_pcd");

  MsgToPCD n;
  n.spin ();

  return (0);
}

