// #include <unistd.h>

#include <ctime>
#include <ros/node.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud.h>
#include <point_cloud_mapping/cloud_io.h>

class TableMsgSplit
{
  protected:
    ros::NodeHandle nh_;
    std::string input_table_topic_;
    std::string output_polygon_topic_;
    std::string output_pcds_topic_;
    ros::Subscriber table_sub_;

  public:
    TableMsgSplit ()
    {
      nh_.param ("input_table_topic", input_table_topic, std::string("table_with_objects"));
      nh_.param ("output_polygon_topic", output_pcds_topic_, std::string("table_polygon"));
      nh_.param ("output_pcds_topic", output_polygon_topic_, std::string("table_pcds"));
      cloud_sub_ = nh_.subscribe (input_cloud_topic_, 1, &MsgToPCD::table_cb, this);
    }
    
    void
      table_cb (const ias_table_msgs::TableWithObjects::ConstPtr& table)
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


