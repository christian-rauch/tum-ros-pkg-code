
#define XML_NODE_SWISSRANGER   "SwissRangerRemoteSensor"
#define XML_PROPERTY_TOPIC     "Topic"

#include <sensor_msgs/PointCloud.h>
#include <point_cloud_mapping/cloud_io.h>
#include <sstream>

#include <XMLTag.h>

class SwissRangerReading : public cop::Reading
{
public:
  SwissRangerReading(const sensor_msgs::PointCloudConstPtr& pcdcloud) :
      Reading(ReadingType_PointCloud)
  {
      m_image = (*pcdcloud);
  }

  SwissRangerReading() :
      Reading(ReadingType_PointCloud)
  {
  }

  ~SwissRangerReading()
  {
  }

 /**
   *   Save the image in a file and put a string in the xml
   */
  virtual cop::XMLTag* Save()
  {
    using namespace cop;
    /*TODO: Save to file and save filename to an XMLTag*/
    XMLTag* tag = new XMLTag("PointCloud");

    std::ostringstream os;
    os << "pcd_"<< tag->date() << ".pcd";
    tag->AddProperty("FileName", os.str());
    cloud_io::savePCDFile (os.str().c_str(), m_image);    
                
    return  tag;
  }
  /**
   *   Copy the image
   */
  virtual cop::Reading* Clone()
  {
    /*TODO: Copy m_pcd and create new reading*/
    return  NULL;
  }
  sensor_msgs::PointCloud m_image;
};

