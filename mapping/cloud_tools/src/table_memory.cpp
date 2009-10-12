// #include <unistd.h>

#include <ctime>
#include <ros/node_handle.h>
#include <ias_table_msgs/TableWithObjects.h>
#include <ias_table_srvs/ias_table_clusters_service.h>
#include <point_cloud_mapping/cloud_io.h>
#include <geometry_msgs/Polygon.h>
#include <tabletop_msgs/Table.h>

class TableObject
{
public:
  sensor_msgs::PointCloud point_cluster;
  // some other things like shape model etc.
};

class TableStateInstance
{
public:
  ros::Time time_instance;
  std::vector<TableObject> objects;
};

class Table
{
public:
  bool new_flag;
  geometry_msgs::Point32 center;
  geometry_msgs::Polygon polygon;

  std::vector<TableStateInstance> inst;
  void getInstance ();
};

class TableMemory
{
  protected:
    ros::NodeHandle nh_;
    std::string input_table_topic_;
    ros::Subscriber table_sub_;
    ros::ServiceServer table_memory_clusters_service_;
    int counter_;

    // THE structure... :D
    std::vector<Table> tables;

  public:
    TableMemory () : counter_(0)
    {
      nh_.param ("input_table_topic", input_table_topic_, std::string("table_with_objects"));       // 15 degrees
      table_sub_ = nh_.subscribe (input_table_topic_, 1, &TableMemory::table_cb, this);
      table_memory_clusters_service_ = nh_.advertiseService ("table_memory_clusters_service", &TableMemory::clusters_service, this);
    }
    
    bool compare_table (Table& old_table, const tabletop_msgs::Table::ConstPtr& new_table)
    {
      return true;
    }

    void
      update_table (Table& old_table, const tabletop_msgs::Table::ConstPtr& new_table)
    {
      ROS_INFO ("Table found. Updating table with new TableInstance.");
      TableStateInstance inst;
      for (unsigned int i = 0; i < new_table->objects.size(); i++)
      {
        TableObject to;
	//        to.point_cluster = new_table->objects[i].points;
        inst.time_instance = new_table->header.stamp;
        inst.objects.push_back (to);
      }
      old_table.new_flag = true;
    }
    
    bool
      clusters_service (ias_table_srvs::ias_table_clusters_service::Request &req, ias_table_srvs::ias_table_clusters_service::Response &resp)
    {
      for (unsigned int i = 0; i < tables.size(); i++)
      {
        if (tables[i].new_flag)
        {
          //tables[i].polygon.header.stamp
          //msg.id = i;
          tables[i].new_flag = false;
          for (unsigned int j = 0; j < tables[i].inst.back ().objects.size(); j++)
          {
            TableObject to = tables[i].inst.back ().objects[j];
            //msg.objectid = j;
          }

        }
      }

      return true;
    }

    // incoming data...
    void
      table_cb (const tabletop_msgs::Table::ConstPtr& table)
    {
      bool found = false;
      ROS_INFO ("Looking for table in list of known tables.");
      for (std::vector<Table>::iterator it = tables.begin (); it != tables.end (); it++)
      {
        if (compare_table (*it, table))
        {
          update_table (*it, table);
          found = true;
          break;
        }
      }
      if (! found)
      {
        ROS_INFO ("Not found. Creating new table.");
        Table t;
        t.center.x = table->table_max.x - table->table_min.x / 2.0;
        t.center.y = table->table_max.y - table->table_min.y / 2.0;
        t.center.z = table->table_max.z - table->table_min.z / 2.0;
        t.polygon  = table->table;
        t.new_flag = true;

        tables.push_back (t);
      }
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
  ros::init (argc, argv, "table_memory");

  TableMemory n;
  n.spin ();

  return (0);
}

