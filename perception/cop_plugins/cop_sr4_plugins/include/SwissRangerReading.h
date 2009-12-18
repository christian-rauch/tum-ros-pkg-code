
#define XML_NODE_SWISSRANGER   "SwissRangerRemoteSensor"
#define XML_PROPERTY_TOPIC     "Topic"

#define PCD_READING_TYPE 2

#include <sensor_msgs/PointCloud.h>
class SwissRangerReading : public cop::Reading
{
public:
  SwissRangerReading(const sensor_msgs::PointCloudConstPtr& pcdcloud) :
      Reading(PCD_READING_TYPE)
  {
      m_pcd = (*pcdcloud);
  }

  SwissRangerReading() :
      Reading(PCD_READING_TYPE)
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

