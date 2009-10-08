// #include <unistd.h>

#include <ctime>
#include <ros/node.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud.h>
#include <point_cloud_mapping/cloud_io.h>
#include <tabletop_msgs/Table.h>
#include <ias_table_msgs/TableWithObjects.h>
#include <geometry_msgs/PolygonStamped.h>

class TableMsgSplit
{
  protected:
    ros::NodeHandle nh_;
    std::string input_table_topic_;
    std::string output_polygon_topic_;
    std::string output_pcds_topic_;
    ros::Subscriber table_sub_;

    ros::Publisher polygon_pub_;
    ros::Publisher clusters_pub_;

  public:
    TableMsgSplit (ros::NodeHandle &anode) : nh_(anode)
    {
      nh_.param ("input_table_topic", input_table_topic_, std::string("table_with_objects"));
      nh_.param ("output_pcds_topic", output_pcds_topic_, std::string("table_pcds"));
      nh_.param ("output_polygon_topic", output_polygon_topic_, std::string("table_polygon"));
      
      table_sub_ = nh_.subscribe (input_table_topic_, 1, &TableMsgSplit::table_cb, this);
      polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped> (output_polygon_topic_, 1);
      clusters_pub_ = nh_.advertise<sensor_msgs::PointCloud> (output_pcds_topic_, 1);
    }
    
    void
      table_cb (const tabletop_msgs::Table::ConstPtr& table)
    {
      geometry_msgs::PolygonStamped p;
      p.header = table->header;
      p.polygon = table->table;
      polygon_pub_.publish (p);
      
      for (unsigned int i = 0; i < table->objects.size (); i++)
      {
        sensor_msgs::PointCloud pc;
        pc.header = table->header;
        pc.points = table->objects[i].points.points;
        clusters_pub_.publish (pc);
      }
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
  ros::init (argc, argv, "table_msg_split");

  ros::NodeHandle nh("/");
  TableMsgSplit n (nh);
  ros::spin ();

  return (0);
}


