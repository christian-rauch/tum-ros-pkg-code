// #include <unistd.h>

#include <map>
#include <ctime>
#include <ros/node_handle.h>
#include <ias_table_msgs/TableWithObjects.h>
#include <ias_table_msgs/PrologReturn.h>
#include <ias_table_srvs/ias_table_clusters_service.h>
#include <ias_table_srvs/ias_reconstruct_object.h>
#include <point_cloud_mapping/cloud_io.h>
#include <point_cloud_mapping/geometry/areas.h>
#include <point_cloud_mapping/geometry/statistics.h>
#include <geometry_msgs/Polygon.h>

// COP/JLO stuff
#include <vision_srvs/srvjlo.h>
#include <vision_srvs/cop_call.h>
#include <vision_msgs/partial_lo.h>
#include <vision_msgs/cop_answer.h>

struct TableObject
{
  TableObject (): type (-1), lo_id (0) { }
  geometry_msgs::Point32 center;
  sensor_msgs::PointCloud point_cluster;
  geometry_msgs::Point32 minP;
  geometry_msgs::Point32 maxP;
  int type;
  unsigned long long lo_id;
  std::vector <double> coeffs;
  double score;
  std::vector<int> triangles;
  std::string semantic_type;
  std::vector<std::string> color;
};

struct TableStateInstance
{
  ros::Time time_instance;
  std::vector<TableObject*> objects;
};

struct Table
{
  unsigned new_flag;
  geometry_msgs::Point32 center;
  geometry_msgs::Polygon polygon;

  std::vector<TableStateInstance*> inst;
  
  TableStateInstance *getCurrentInstance ()
  {

    if (inst.size() == 0)
    {
      ROS_ERROR ("TableStateInstance requested in empty list.");
      return new TableStateInstance ();
    }

    return inst.back ();
  }

  std::vector<TableStateInstance*> getLastInstances (unsigned int n)
  {
    std::vector<TableStateInstance*> ret;
    for (std::vector<TableStateInstance*>::reverse_iterator it = inst.rbegin (); it != inst.rend (); it++)
      {
        if (n != 0)
          {
            ret.push_back(*it);
            n--;
          }
        else
          break;
      }
    return ret;
  }

  TableStateInstance *getInstanceAtTime (ros::Time t)
  {
    TableStateInstance* ret = inst.back ();
    for (std::vector<TableStateInstance*>::reverse_iterator it = inst.rbegin (); it != inst.rend (); it++)
      if ((*it)->time_instance <= t)
        ret = *it;
      else
        break;
    return ret;
  }

};

class TableMemory
{
  protected:
    ros::NodeHandle &nh_;
    std::string input_table_topic_;
    std::string input_cop_topic_;
    ros::Subscriber table_sub_;
    ros::Subscriber cop_sub_;
    ros::ServiceServer table_memory_clusters_service_;
    ros::ServiceClient table_reconstruct_clusters_client_;
    int counter_;
    float color_probability_;
    //insert lo_ids waiting for prolog update
    std::vector<unsigned long long> update_prolog_;

    
    // THE structure... :D
    std::vector<Table> tables;
   //map<LO Id, vector[tableId, instId, objectId]>
    std::map <unsigned long long, std::vector<long> > lo_ids;
    TableObject *getObjectFromLOId (unsigned int id)
    {
      std::vector<long> idxs = lo_ids[id];
      return tables[idxs[0]].inst[idxs[1]]->objects[idxs[2]];
    }

    ias_table_msgs::PrologReturn getPrologReturn(unsigned int id)
   {
     ias_table_msgs::PrologReturn ret;
     std::vector<long> idxs = lo_ids[id];
     ret.table_id = idxs[1];
     ret.stamp =  tables[idxs[0]].inst[idxs[1]]->time_instance;
     ret.cluster_center =  tables[idxs[0]].inst[idxs[1]]->objects[idxs[2]]->center;
     ret.cluster_colors =  tables[idxs[0]].inst[idxs[1]]->objects[idxs[2]]->color;
     return ret;
   }

  public:
    TableMemory (ros::NodeHandle &anode) : nh_(anode), counter_(0), color_probability_(0.2)
    {
      nh_.param ("input_table_topic", input_table_topic_, std::string("table_with_objects"));       // 15 degrees
      nh_.param ("input_cop_topic", input_cop_topic_, std::string("/tracking/out"));       // 15 degrees
      table_sub_ = nh_.subscribe (input_table_topic_, 1, &TableMemory::table_cb, this);
      cop_sub_ = nh_.subscribe (input_cop_topic_, 1, &TableMemory::cop_cb, this);
      table_memory_clusters_service_ = nh_.advertiseService ("table_memory_clusters_service", &TableMemory::clusters_service, this);
    }
    
    bool compare_table (Table& old_table, const ias_table_msgs::TableWithObjects::ConstPtr& new_table)
    {
      // if center of new (invcomplete) table is within old 
      // table bounds, it's the same
      geometry_msgs::Point32 center;
      center.x = new_table->table_max.x - new_table->table_min.x / 2.0;
      center.y = new_table->table_max.y - new_table->table_min.y / 2.0;
      center.z = new_table->table_max.z - new_table->table_min.z / 2.0;
      if (cloud_geometry::areas::isPointIn2DPolygon (center, old_table.polygon))
        return true;
      return false;
    }

    void
      update_table (int table_num, const ias_table_msgs::TableWithObjects::ConstPtr& new_table)
    {
      Table &old_table = tables[table_num];
      ROS_INFO ("Table found. Updating table with new TableInstance.");
      TableStateInstance *inst = new TableStateInstance ();
      for (unsigned int i = 0; i < new_table->objects.size(); i++)
      {
        TableObject *to = new TableObject ();

        to->point_cluster = new_table->objects[i].points;
        cloud_geometry::statistics::getMinMax (to->point_cluster, to->minP, to->maxP);

        to->center.x = to->minP.x + (to->maxP.x - to->minP.x) * 0.5;
        to->center.y = to->minP.y + (to->maxP.y - to->minP.y) * 0.5;
        to->center.z = to->minP.z + (to->maxP.z - to->minP.z) * 0.5;

        inst->objects.push_back (to);
      }
      inst->time_instance = new_table->header.stamp;
      old_table.inst.push_back (inst);
      
    }
   
    // service call from PROLOG 
    bool
      clusters_service (ias_table_srvs::ias_table_clusters_service::Request &req, 
                          ias_table_srvs::ias_table_clusters_service::Response &resp)
    {
      for (unsigned int up = 0; update_prolog_.size(); up++)
        {
          ias_table_msgs::PrologReturn pr =  getPrologReturn (update_prolog_[up]);
          resp.prolog_return.push_back(pr);
        }
      //TODO lock
      update_prolog_.clear();
      return true;
    }

    void cop_cb (const boost::shared_ptr<const vision_msgs::cop_answer> &msg)
    {
      ROS_INFO ("got answer from cop! (Errors: %s)\n", msg->error.c_str());
      for(unsigned int i = 0; i < msg->found_poses.size(); i++)
      {
        const vision_msgs::aposteriori_position &pos =  msg->found_poses [i];
        ROS_INFO ("Found Obj nr %d with prob %f at pos %d\n", (int)pos.objectId, pos.probability, (int)pos.position);
        //this here asumes that color classes are returned in FIFO fashion wrt to cop query (see cop_call function)!!!
        if(pos.probability >= color_probability_)
        {
          TableObject * to = getObjectFromLOId (pos.position);
          for (unsigned int cls = 0; cls < pos.classes.size (); cls++)
          {
            ROS_INFO("Object color is %s", pos.classes[cls].c_str());
            //possible returns: [red (or any other color), object type (Jug)]
            //color is a vector of strings in case object contains multiple color hypotheses
            to->color.push_back(pos.classes[cls]);
          }
        }
      }
      ROS_INFO ("End!\n");
    }
  

    bool update_jlo (int table_num)
    {
      ros::ServiceClient jlo_client_ = nh_.serviceClient<vision_srvs::srvjlo> ("/located_object", true);

      // TODO: 
      //if (!jlo_client_.exists ()) return false;

      for (unsigned int o_idx = 0; o_idx < tables[table_num].getCurrentInstance ()->objects.size (); o_idx++)
      {
        TableObject *o = tables[table_num].getCurrentInstance ()->objects [o_idx];
        geometry_msgs::Point32 extents;
        extents.x = o->center.x - o->minP.x;
        extents.y = o->center.y - o->minP.y;
        extents.z = o->center.z - o->minP.z;
        
        // create service client call to jlo
        vision_srvs::srvjlo call;
        call.request.command = "update";
        //world frame
        call.request.query.parent_id = 1;
        call.request.query.id = 0;

        //fill in pose
        int width = 4;
        for(int r = 0; r < width; r++)
        {
          for(int c = 0; c < width; c++)
          {
            if(r == c)
              call.request.query.pose[r * width + c] = 1;
            else
              call.request.query.pose[r * width + c] = 0;
          }
        }
        
        call.request.query.pose[3]  = o->center.x;
        call.request.query.pose[7]  = o->center.y;
        call.request.query.pose[11] = o->center.z;

        //fill in covariance matrix: 0.9 * 1/2*max(cluster)-min(cluster) [m]
        width = 6;
        for(int r = 0; r < width; r++)
        {
          for(int c = 0; c < width; c++)
          {
            if(r == c)
              call.request.query.cov [r * width + c] = 0.0;
            else
              call.request.query.cov [r * width + c] = 0;
          }
        }

        call.request.query.cov [0]  = extents.x * 0.9;
        call.request.query.cov [7]  = extents.y * 0.9;
        call.request.query.cov [13] = extents.z * 0.9;

        if (!jlo_client_.call(call))
        {
          ROS_ERROR ("Error in ROSjloComm: Update of pose information not psossible!\n");
          return false;
        }
        else if (call.response.error.length() > 0)
        {
          ROS_ERROR ("Error from jlo: %s!\n", call.response.error.c_str());
          return false;
        } 
        
        ROS_INFO ("New Id: %lld (parent %lld)\n", (long long int)call.response.answer.id, (long long int)call.response.answer.parent_id);
        width = 4;
        for(int r = 0; r < width; r++)
        {
          for(int c = 0; c < width; c++)
          {
             printf("%f", call.response.answer.pose[r * width + c]);
          }
          printf("\n");
        }
        printf("\n");

        o->lo_id = call.response.answer.id;
      }
      return true;
    }

    bool call_cop (int table_num)
    { 
      ros::ServiceClient cop_client_ = nh_.serviceClient<vision_srvs::cop_call> ("/tracking/in", true);
      std::vector <std::string> colors;
      colors.push_back ("blue");
      colors.push_back ("black");
      colors.push_back ("green");
      colors.push_back ("red");
      colors.push_back ("white");

      for (unsigned int col = 0; col < colors.size(); col ++)
      {
        /** Create the cop_call msg*/
        vision_srvs::cop_call call;
        call.request.outputtopic = input_cop_topic_;
        call.request.object_classes.push_back (colors[col]);
        call.request.action_type = 0;
        call.request.number_of_objects = tables[table_num].getCurrentInstance ()->objects.size ();
        
        for (unsigned int o_idx = 0; o_idx < tables[table_num].getCurrentInstance ()->objects.size (); o_idx++)
        {
          TableObject *o = tables[table_num].getCurrentInstance ()->objects [o_idx];
          
          // save the indices in our vector of vector of vector so we can find a object
          // in our storage from the positionID (lo_id)
          std::vector<long> idxs (3);
          idxs[0] = table_num;
          idxs[1] = tables[table_num].inst.size()-1;
          idxs[2] = o_idx;
          lo_ids [o->lo_id] = idxs;

          update_prolog_.push_back(o->lo_id);

          vision_msgs::apriori_position pos;
          pos.probability = 1.0;
          pos.positionId = o->lo_id;
          
          call.request.list_of_poses.push_back (pos);
        } 
          
        if(!cop_client_.call(call))
        {
          ROS_INFO("Error calling cop\n");
          return false;
        }
        else
        {
          ROS_INFO("Called cop \n");
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
          req.interesting_types = ias_table_msgs::TableObjectReconstructed::PLANE;
          
          //call service
          ias_table_srvs::ias_reconstruct_object::Response resp;
          table_reconstruct_clusters_client_.call (req, resp); 
          
          // update our table object with detected shapes
          if (resp.objects.size () > 0)
          {
            ias_table_msgs::TableObjectReconstructed rto = resp.objects.at(0);
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
      table_cb (const ias_table_msgs::TableWithObjects::ConstPtr& table)
    {
      int table_found = -1;
      ROS_INFO ("Looking for table in list of known tables.");
      for (int i = 0; i < (signed int) tables.size (); i++)
      {
        if (compare_table (tables[i], table))
        { 
          // found same table earlier.. so we append a new table instance measurement
          update_table (i, table);
          table_found = i;
          break;
        }
      }
      if (table_found == -1)
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
        update_table (table_found, table);
      }
      
      reconstruct_table_objects (table_found);
      update_jlo (table_found);
      call_cop (table_found);
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

  ros::NodeHandle nh("~");
  TableMemory n(nh);
  ros::spin ();

  return (0);
}

