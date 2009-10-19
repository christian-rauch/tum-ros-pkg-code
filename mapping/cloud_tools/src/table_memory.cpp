// #include <unistd.h>

#include <ctime>
#include <ros/node_handle.h>
#include <ias_table_msgs/TableWithObjects.h>
#include <ias_table_srvs/ias_table_clusters_service.h>
#include <ias_table_srvs/ias_reconstruct_object.h>
#include <point_cloud_mapping/cloud_io.h>
#include <geometry_msgs/Polygon.h>
#include <tabletop_msgs/Table.h>

struct TableObject
{
  TableObject (): type (-1) { }
  geometry_msgs::Point32 center;
  sensor_msgs::PointCloud point_cluster;
  int type;
  std::vector <double> coeffs;
  double score;
  std::vector<int> triangles;
  std::string semantic_type;
};

struct TableStateInstance
{
  ros::Time time_instance;
  std::vector<TableObject*> objects;
};

struct Table
{
  bool new_flag;
  geometry_msgs::Point32 center;
  geometry_msgs::Polygon polygon;

  std::vector<TableStateInstance*> inst;
  
  TableStateInstance *getCurrentInstance ()
  {
    return inst.back ();
  }

  TableStateInstance *getInstanceAtTime (ros::Time t)
  {
    TableStateInstance* ret = inst.back ();
    for (std::vector<TableStateInstance*>::reverse_iterator it = inst.rbegin (); it != inst.rend (); it++)
      if ((*it)->time_instance <= ret->time_instance)
        ret = *it;
      else
        break;
    return ret;
//    return inst.back ();
    // should do a reverse_iterator
  }
};

class TableMemory
{
  protected:
    ros::NodeHandle nh_;
    std::string input_table_topic_;
    ros::Subscriber table_sub_;
    ros::ServiceServer table_memory_clusters_service_;
    ros::ServiceClient table_reconstruct_clusters_client_;
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
      TableStateInstance *inst = new TableStateInstance ();
      for (unsigned int i = 0; i < new_table->objects.size(); i++)
      {
        TableObject *to = new TableObject ();
	      to->point_cluster = new_table->objects[i].points;
        inst->time_instance = new_table->header.stamp;
        inst->objects.push_back (to);
      }
      old_table.inst.push_back (inst);
      old_table.new_flag = true;
    }
   
    // service call from PROLOG 
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
          for (unsigned int j = 0; j < tables[i].getCurrentInstance ()->objects.size(); j++)
          {
            TableObject *to = tables[i].getCurrentInstance ()->objects[j];
            std::cerr << to->semantic_type << std::endl;
            //msg.objectid = j;
          }

        }
      }

      return true;
    }

    void
      reconstruct_table_objects (int table_num)
    {
      return;
      Table &t = tables[table_num];
      table_reconstruct_clusters_client_ = nh_.serviceClient<ias_table_srvs::ias_reconstruct_object> ("ias_reconstruct_object", true);
      //if (table_reconstruct_clusters_client_.exists ())
      {
        for (int i = 0; i < (signed int) t.getCurrentInstance ()->objects.size (); i++)
        {
          TableObject* to = t.getCurrentInstance ()->objects.at (i); 
          // formulate a request
          ias_table_srvs::ias_reconstruct_object::Request req;
          req.cloud_in = to->point_cluster;
          req.prob_thresh = 0.99;
          req.ransac_thresh = 0.05;
          req.angle_thresh = 10;
          req.interesting_types = ias_table_msgs::TableObject::PLANE;
          
          //call service
          ias_table_srvs::ias_reconstruct_object::Response resp;
          table_reconstruct_clusters_client_.call (req, resp); 
          
          // update our table object with detected shapes
          if (resp.objects.size () > 0)
          {
            ias_table_msgs::TableObject rto = resp.objects.at(0);
            to->type = rto.type;
            to->coeffs = rto.coefficients;
            to->score = rto.score;
            //to->center;
            //to->triangles;
            //TODO: what if we saw multiple things in one cluster??
          }
        }
      }
    }

    // incoming data...
    void
      table_cb (const tabletop_msgs::Table::ConstPtr& table)
    {
      int table_found = -1;
      ROS_INFO ("Looking for table in list of known tables.");
      for (int i = 0; i < (signed int) tables.size (); i++)
      {
        if (compare_table (tables[i], table))
        { 
          // found same table earlier.. so we append a new table instance measurement
          update_table (tables[i], table);
          table_found = i;
          break;
        }
      }
      if (! table_found == -1)
      {
        ROS_INFO ("Not found. Creating new table.");
        Table t;
        t.center.x = table->table_max.x - table->table_min.x / 2.0;
        t.center.y = table->table_max.y - table->table_min.y / 2.0;
        t.center.z = table->table_max.z - table->table_min.z / 2.0;
        t.polygon  = table->table;

        tables.push_back (t);
        table_found = tables.size () - 1;
        // also append the new (first) table instance measurement.
        update_table (tables[table_found], table);
      }
      

      reconstruct_table_objects (table_found);
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

