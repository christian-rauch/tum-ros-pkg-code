// #include <unistd.h>

#include <ctime>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud.h>
//later: cut polygon instead of AABB
#include <geometry_msgs/PolygonStamped.h>

class BoxFilter
{
  protected:
    ros::NodeHandle nh_;
    std::string input_cloud_topic_;
    std::string output_cloud_topic_;
    
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;

    double box_min_x_;
    double box_max_x_;
    double box_min_y_;
    double box_max_y_;

  public:
    BoxFilter (ros::NodeHandle &anode) : nh_(anode)
    {
      nh_.param ("input_cloud_topic", input_cloud_topic_, std::string("cloud_pcd"));
      nh_.param ("output_cloud_topic", output_cloud_topic_, std::string("cloud_box_clip"));
      
      nh_.param ("box_min_x", box_min_x_, -4.0);
      nh_.param ("box_max_x", box_max_x_,  3.0);
      nh_.param ("box_min_y", box_min_y_, -3.0);
      nh_.param ("box_max_y", box_max_y_,  2.5);
      
      cloud_sub_ = nh_.subscribe (input_cloud_topic_, 1, &BoxFilter::cloud_cb, this);
      cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud> (output_cloud_topic_, 1);
    }
    
    void
      cloud_cb (const sensor_msgs::PointCloudConstPtr& cloud)
    {
      // copy header over from input cloud
      sensor_msgs::PointCloud cloud_out;
      cloud_out.header = cloud->header;

      // resize channels and copy their names
      cloud_out.channels.resize (cloud->channels.size());
      for (unsigned int i = 0; i < cloud->channels.size(); i++)
        cloud_out.channels[i].name = cloud->channels[i].name;

      // go through all points 
      for (unsigned int i = 0; i < cloud->points.size(); i++)
      {
        // select all points that are within in the specified points
        if (cloud->points[i].x > box_min_x_ && cloud->points[i].x < box_max_x_
           && cloud->points[i].y > box_min_y_ && cloud->points[i].y < box_max_y_)
        {
          // copy point over
          cloud_out.points.push_back (cloud->points[i]);

          // copy channel values over
          for (unsigned int j = 0; j < cloud->channels.size(); j++)
            cloud_out.channels[j].values.push_back (cloud->channels[j].values[i]);
        }
      }
      cloud_pub_.publish (cloud_out); 
    }

//     bool 
//       spin ()
//     {
//       while (ros::ok())
//       {
//         ros::spinOnce ();
//       }
//       return true;
//     }
};

int main (int argc, char* argv[])
{
  ros::init (argc, argv, "box_filter");

  ros::NodeHandle nh("~");
  BoxFilter n (nh);
  ros::spin ();

  return (0);
}



