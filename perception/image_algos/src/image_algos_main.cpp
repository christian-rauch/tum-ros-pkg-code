// #include <unistd.h>

#include <map>
#include <ctime>
#include <ros/node_handle.h>
#include <ias_table_msgs/TableWithObjects.h>

// image_algos plugin stuff
#include <image_algos/image_algos.h>
#include <image_algos/color_find_hsv.h>
#include <image_algos/pcd_to_image_projector_algo.h>
#include <pluginlib/class_loader.h>

//openCV
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

//boost
#include <boost/thread/mutex.hpp>

using namespace image_algos;

/// holds a single object on a table, with timestamp, orig. pcd data and reconstructed representations
struct TableObject
{
  TableObject (): name(""),  sensor_type(""), object_type(""), object_color(""), 
                  object_geometric_type(""), perception_method(""){ }
  geometry_msgs::Point32 center;
  sensor_msgs::PointCloud point_cluster;
  geometry_msgs::Point32 minP;
  geometry_msgs::Point32 maxP;
  //unsigned long long lo_id;
  std::vector <double> coeffs;
  double score;
  std::vector<int> triangles;
  std::string name;
  std::string sensor_type;
  std::string object_type;
  std::string object_color;
  std::string object_geometric_type;
  std::string perception_method;
};

/// holds a single snapshot of a table
struct TableStateInstance
{
  ros::Time time_instance;
  std::vector<TableObject*> objects;
};

struct Table
{
  geometry_msgs::Point32 center;
  geometry_msgs::Polygon polygon;
  //sensor_msgs::Image roi;
  IplImage *roi;
  std::vector<TableStateInstance*> inst;
};

class ImageAlgosMain
{
protected:
  ros::NodeHandle &nh_;
  
  // topic names
  std::string input_table_topic_;
  std::string input_image_topic_;
  std::string output_table_roi_topic_;
  
  // publishers and subscribers
  ros::Publisher table_image_pub_;
  ros::Subscriber table_sub_;
  ros::Subscriber image_sub_;

  IplImage* image_, *clone_;
  boost::mutex  image_lock_;
  sensor_msgs::CvBridge bridge_;
  // plugin loader
  pluginlib::ClassLoader<image_algos::ImageAlgo> *cl;

  typedef struct _NamedAlgorithm 
  {
    std::string name;
    ImageAlgo * algorithm;
    _NamedAlgorithm (std::string n) : name(n) {};
  } NamedAlgorithm;
  
  std::vector<NamedAlgorithm> algorithm_pool;
  
  //table object
  Table t;
  //utility params
  bool got_image_;
public:
  ImageAlgosMain (ros::NodeHandle &anode) : nh_(anode)
  {
    nh_.param ("input_table_topic", input_table_topic_, std::string("/table_with_objects"));
    nh_.param ("input_image_topic", input_image_topic_, std::string("/cop/right/camera"));
    nh_.param ("output_table_roi_topic", output_table_roi_topic_, std::string("table_roi"));       // 15 degrees
    table_sub_ = nh_.subscribe (input_table_topic_, 1, &ImageAlgosMain::table_cb, this);
    image_sub_ = nh_.subscribe (input_image_topic_, 1, &ImageAlgosMain::image_cb, this);
    table_image_pub_ = nh_.advertise<sensor_msgs::Image> (output_table_roi_topic_, 1);
    algorithm_pool.push_back (NamedAlgorithm ("image_algos/ColorFindHSV"));
    algorithm_pool.push_back (NamedAlgorithm ("image_algos/PCDToImageProjector"));
    load_plugins ();
    got_image_ = false;
  }

  /*!
   * \brief loads a given plugin
   * \param algo_name : name of algorithm as stated in plugins.xml file
   * \param algorithm : reference to pointer which will hold the loaded algorithm
   */
  bool load_algorithm (std::string algo_name, ImageAlgo*& algorithm)
  {
    try
    {
      cl->loadLibraryForClass(algo_name);
      ROS_DEBUG("Loaded library with plugin %s inside", algo_name.c_str());
    }
    catch(pluginlib::PluginlibException &ex)
    {
      ROS_ERROR("Failed to load library with plugin %s inside. Exception: %s", algo_name.c_str(), ex.what());
    }
    
    if (cl->isClassLoaded(algo_name))
    {
      algorithm = cl->createClassInstance(algo_name);
      algorithm->init (nh_);
      return true;
    }
    else ROS_ERROR("Cannot create ImageAlgo Class of type %s", algo_name.c_str ());
    return false;
  }
  
  /*!
   * \brief loads plugins
   */
  void load_plugins ()
  {
    cl = new pluginlib::ClassLoader <image_algos::ImageAlgo> ("image_algos", "image_algos::ImageAlgo");
    
    ROS_INFO("ClassLoader instantiated");
    std::vector<std::string> plugins = cl->getDeclaredClasses();
    
    for (std::vector<std::string>::iterator it = plugins.begin(); it != plugins.end() ; ++it)
    {
      ROS_INFO("%s is in package %s and is of type %s", it->c_str(), cl->getClassPackage(*it).c_str(), cl->getClassType(*it).c_str());
      ROS_INFO("It does \"%s\"", cl->getClassDescription(*it).c_str());
    }
    
    for (unsigned int i = 0; i < algorithm_pool.size(); i++)
      load_algorithm (algorithm_pool[i].name, algorithm_pool[i].algorithm);
  }
  
  ImageAlgo* find_algorithm (std::string name)
  {
    for (unsigned int i = 0; i < algorithm_pool.size(); i++)
      if (algorithm_pool[i].name == name)
        return algorithm_pool[i].algorithm;
    return NULL;
  }
  
  void color_find (const boost::shared_ptr<const sensor_msgs::Image> image)
  {
    ImageAlgo * color_find = find_algorithm ("ColorFindHSV");
    ros::Publisher pub_image = nh_.advertise <ColorFindHSV::OutputType>
      (((ColorFindHSV*)color_find)->default_output_topic (), 5);
    
    // call ColorFindHSV
    color_find->pre();
    //std::cerr << "[color_find] Calling Color with a PCD with " <<
    //to->point_cluster.points.size () << " points." << std::endl;
    std::string process_answer_color_find = ((ColorFindHSV*)color_find)->process(image);
    ROS_INFO("got response: %s", process_answer_color_find.c_str ());
    boost::shared_ptr <const ias_table_msgs::TableObject> color_image = (((ColorFindHSV*)color_find)->output ());
    //image_pub_.publish (color_image);
    color_find->post();
  }


  // incoming data...
  void table_cb (const ias_table_msgs::TableWithObjects::ConstPtr& table)
  {
    t.center.x = table->table_min.x + ((table->table_max.x - table->table_min.x) / 2.0);
    t.center.y = table->table_min.y + ((table->table_max.y - table->table_min.y) / 2.0);
    t.center.z = table->table_min.z + ((table->table_max.z - table->table_min.z) / 2.0);
    for (unsigned int i = 0; i < table->table.points.size(); i++)
      t.polygon.points.push_back (table->table.points.at(i));
    ROS_INFO("Table polygon with size %ld", t.polygon.points.size());

    
    ImageAlgo *proj_points = find_algorithm ("image_algos/PCDToImageProjector");
    proj_points->pre();
    //got_cloud_ = true;
    if (got_image_)
    {
      std::string process_answer_proj_points = ((PCDToImageProjector*)proj_points)->process (t.polygon, clone_);
      ROS_INFO("Got response for table: %s", process_answer_proj_points.c_str ());
      t.polygon.points.clear();
      PCDToImageProjector::OutputType table_roi = ((PCDToImageProjector*)proj_points)->output ();
      //table_image_pub_.publish(bridge_.cvToImgMsg(table_roi));
      ((PCDToImageProjector*)proj_points)->post ();
      for (unsigned int i = 0; i < table->objects.size(); i++)
      {
        sensor_msgs::PointCloud point_cluster = table->objects[i].points;
        std::string process_answer_proj_points = ((PCDToImageProjector*)proj_points)->process (point_cluster, clone_);
        ROS_INFO("Got response for cluster: %s", process_answer_proj_points.c_str ());
        PCDToImageProjector::OutputType cluster_roi = ((PCDToImageProjector*)proj_points)->output ();
        sleep(1);
        table_image_pub_.publish(bridge_.cvToImgMsg(cluster_roi));
        ((PCDToImageProjector*)proj_points)->post ();
      }
      cvReleaseImage(&clone_);
      got_image_ = false;
    }
    //proj_points->post();

    //color_find(image);
    //print_mem_stats (table_found);
  }

  void image_cb(const sensor_msgs::ImageConstPtr& image_msg)
  {
    if (!got_image_)
    {
      image_ = NULL;
      try 
      {
        image_ = bridge_.imgMsgToCv(image_msg, "bgr8");
        clone_ = cvCloneImage(image_);
        ROS_INFO("[ImageAlgosMain: ] got image.");
      }
      catch (sensor_msgs::CvBridgeException& ex) 
      {
        ROS_ERROR("[PointCloudToImageProjector:] Failed to convert image");
        return;
      }
      got_image_ = true;
    }
  }
};

int main (int argc, char* argv[])
{
  ros::init (argc, argv, "image_algos_main");
  
  ros::NodeHandle nh("~");
  ImageAlgosMain n(nh);
  ros::spin ();
  
  return (0);
}

