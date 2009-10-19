// #include <unistd.h>

#include <ctime>
#include <ros/node.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud.h>
#include <point_cloud_mapping/cloud_io.h>
#include <tabletop_msgs/Table.h>
#include <ias_table_msgs/TableWithObjects.h>
#include <ias_table_srvs/ias_reconstruct_object.h>
#include <geometry_msgs/PolygonStamped.h>
#include <mapping_msgs/Object.h>

#include "Sherlock.h"

double epsilon = 0.05;
double prob_thresh = 0.99;
int unsigned min_nr_points_in_shape = 5000;

using namespace ias_table_srvs;

class ObjectReconstruction
{
  protected:
    ros::NodeHandle nh_;
    std::string input_cloud_topic_;
    ros::Subscriber cloud_sub_;
    ros::ServiceServer reconstruct_object_srv_;
    Sherlock *sherlock;

  public:
    ObjectReconstruction (ros::NodeHandle &anode) : nh_(anode)
    {
      nh_.param ("input_table_topic", input_cloud_topic_, std::string("table_pcds"));
      cloud_sub_ = nh_.subscribe (input_cloud_topic_, 1, &ObjectReconstruction::cloud_cb, this);
      sherlock = new Sherlock ();
      reconstruct_object_srv_ = nh_.advertiseService ("reconstruct_object", &ObjectReconstruction::reconstruct_object, this);
    }
    
    void
      cloud_cb (const sensor_msgs::PointCloudConstPtr& points)
    {
      ias_reconstruct_object::Request req;
      req.cloud_in = *points; 
      ias_reconstruct_object::Response resp;
      reconstruct_object (req, resp);
    }

    bool
      reconstruct_object (ias_reconstruct_object::Request &req, ias_reconstruct_object::Response &resp)
    {
      sherlock->SetData (boost::shared_ptr<sensor_msgs::PointCloud const> (&req.cloud_in));
//      sherlock->SetShapeTypes (req.interesting_types);
      sherlock->DetectShapes ();

      // return detected shapes
      for (unsigned int i = 0; i < sherlock->shapes.size(); i++)
      {
        mapping_msgs::Object o;
        o.type = mapping_msgs::Object::MESH;
        std::set<int> inl = sherlock->shapes[i]->GetInliers ();
        o.vertices.resize (inl.size ());
        for (std::set<int>::iterator it = inl.begin (); it != inl.end (); it++)
        { 
          geometry_msgs::Point p;
          p.x = req.cloud_in.points.at (*it).x;
          p.y = req.cloud_in.points.at (*it).y;
          p.z = req.cloud_in.points.at (*it).z;
          o.vertices.push_back (p);
        }
        ROS_INFO ("score %i: %i", i, sherlock->shapes[i]->score);
      }
      return true;
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
  ros::init (argc, argv, "object_reconstruction");

  ros::NodeHandle nh("/");
  ObjectReconstruction n (nh);
  ros::spin ();

  return (0);
}


