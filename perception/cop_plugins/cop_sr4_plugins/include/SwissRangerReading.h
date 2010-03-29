
#define XML_NODE_SWISSRANGER   "SwissRangerRemoteSensor"
#define XML_PROPERTY_TOPIC     "Topic"

#include <sensor_msgs/PointCloud.h>
class SwissRangerReading : public cop::Reading
{
public:
  SwissRangerReading(const sensor_msgs::PointCloudConstPtr& pcdcloud) :
      Reading(ReadingType_PointCloud)
  {
      m_pcd = (*pcdcloud);
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
    /*TODO: Save to file and save filename to an XMLTag*/
    return  NULL;
  }
  /**
   *   Copy the image
   */
  virtual cop::Reading* Clone()
  {
    /*TODO: Copy m_pcd and create new reading*/
    return  NULL;
  }
  sensor_msgs::PointCloud m_pcd;
};

