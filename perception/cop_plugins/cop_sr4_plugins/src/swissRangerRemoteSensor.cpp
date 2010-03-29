#include "pluginlib/class_list_macros.h"

#define BOOST_THREAD

#include "Sensor.h"
#include "XMLTag.h"

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud.h>

#include "SwissRangerReading.h"
#include "ClusterDetector.h"
#include "SegmentPrototype.h"

class SwissRangerRemoteSensor : public cop::Sensor
{
public:
  SwissRangerRemoteSensor() :
      m_grabbing(false)
  {
    printf("Successfully initialized plugin class SwissRangerRemoteSensor\n");
    m_max_cameraImages = 1;
  }

  ~SwissRangerRemoteSensor()
  {
    Stop();
  }

  /**
  *  This can be overwritten to get the data necessary to reconstruct a saved reading
  */
  virtual void SetData(cop::XMLTag* tag)
  {
    Sensor::SetData(tag);
    printf("Set data\n");
    m_stTopic = tag->GetProperty(XML_PROPERTY_TOPIC, "cloud_sr");

  }
  /**
  *  Get Type of the camera by its Name
  */
  virtual std::string GetName() const{return XML_NODE_SWISSRANGER;};

  /**
  * GetReading
  * @param Frame frame number, to specify an offset or a specific file
  * @throws char* with an error message in case of failure
  */
  virtual cop::Reading*	GetReading(const long &Frame = -1)
  {
    if((signed)m_images.size() < (Frame - m_deletedOffset + 1) || m_images.size() == 0)
    {
      if(m_grabbing)
      {
        while(m_grabbing && ((signed)m_images.size() < (Frame - m_deletedOffset + 1) || m_images.size() == 0))
        {
          printf("waiting for the camera %s to start grabbing\n", GetSensorID().c_str());
          sleep(0.2);
        }
        printf("Got a new image: %d\n", (int)m_images.size());
      }
      if((signed)m_images.size() < (Frame - m_deletedOffset + 1) || m_images.size() == 0)
      {
        printf("unexpected error\n");
        throw "Asking for images from a camera that has no images";
      }
    }
    if(Frame == -1 || (Frame - m_deletedOffset < 0 && (unsigned)(Frame - m_deletedOffset) >= m_images.size()))
    {
      return GetReading_Lock(m_images.size() -1);
      /*return m_images[m_images.size() -1];*/
    }
    return GetReading_Lock(Frame - m_deletedOffset);
  }
  /**
  * CanSee
  * Checks if a pose is inside the view of this sensor
  * @param pose pose that has to be looked at
  */
  virtual bool	CanSee(cop::RelPose &pose) const
  {
    /*TODO: iomplement check*/
    return true;
  }

  void CallBack(const sensor_msgs::PointCloudConstPtr& cloud)
  {
    SwissRangerReading* reading = new SwissRangerReading(cloud);
    PushBack(reading);
    while(m_images.size() > m_max_cameraImages)
    {
      if(DeleteReading())
        continue;
      else
      {
        printf("SR: Could not delete an image!");
        break;
      }
    }
  }

  /**
  *    Start
  *   @brief overwrite to start up the data reading, is called at least once after creation
  */
  virtual bool	Start()
  {
    ros::NodeHandle nh;
    printf("Subscribe to topic %s \n", m_stTopic.c_str());
    m_cloudSub = nh.subscribe (m_stTopic, 1, &SwissRangerRemoteSensor::CallBack, this);
    m_grabbing = true;
    return true;
  }
  /**
  *    Start
  *   @brief overwrite to stop call this in the destructor if necessary, will be used to shut down cameras
  */
  virtual bool	Stop()
  {
    m_grabbing = false;
    return true;
    /*TODO unsunscribe*/
  }
  /**
   *   @return the pose of this sensor
   */
  cop::RelPose* GetRelPose(){return m_relPose;}

  virtual cop::XMLTag* Save()
  {
    cop::XMLTag* tag = new cop::XMLTag(GetName());
    cop::Sensor::SaveTo(tag);
    tag->AddProperty(XML_PROPERTY_TOPIC, m_stTopic);
    return tag;
  }
  /**
  *   Can this Sensor be used like a camera, (incl. Calibration, Showing, usw.)
  */
  virtual bool IsCamera(){return false;}

private:
    std::string m_stTopic;
    ros::Subscriber m_cloudSub;
    bool m_grabbing;

};
/**  Sensor Plugin*/
PLUGINLIB_REGISTER_CLASS(SwissRangerRemoteSensor, SwissRangerRemoteSensor, cop::Sensor)
/** Cluster Detector Plugin */
PLUGINLIB_REGISTER_CLASS(ClusterDetector, cop::ClusterDetector, cop::LocateAlgorithm)
/** Cluster Descriptor Plugin */
PLUGINLIB_REGISTER_CLASS(SegmentPrototype, cop::SegmentPrototype, cop::Descriptor)

