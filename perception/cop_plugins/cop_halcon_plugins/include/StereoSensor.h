#ifndef STEREOSENSOR_H
#define STEREOSENSOR_H

#include "Sensor.h"
#include "cpp/HalconCpp.h"


#define XML_NODE_STEREOSENSOR "StereoSensor"
namespace cop
{
  class XMLTag;
  class Camera;
  /**
    * class CameraDriver
    * @brief Implements an reduced interface for the halcon acquisition interface
    */
  class StereoSensor : public Sensor
  {
  public:
     /**************************************************/
     /** Constructor Camera Driver                     *
     ***************************************************
     *  \param stConfigFile xml-configuration file,
                  contains info about ptu, cameratype...
     ***************************************************/
      StereoSensor ();
      /**
      *  Get Type of the camera by its Name
      */
      virtual std::string GetName() const{return XML_NODE_STEREOSENSOR;};
      /**
      * Empty Destructor
      */
      virtual ~StereoSensor ( );
      /**
      * GetReading
      * @param Frame frame number, to specify an offset or a specific file
      * @throws char* with an error message in case of failure
      */
      virtual Reading* GetReading(const long &Frame);

      virtual bool  CanSee(RelPose &pose) const;

      virtual XMLTag* Save();
      virtual void SetData(XMLTag* tag);

      virtual bool Start();
      virtual bool Stop();
      /**
      * m_stSensorName
      *  @brief Stereo works as a composition of names sensors
      */
      virtual bool RequiresSensorList();
      /**
      *  SetSensorList
      *  @brief Stereo works as a composition of names sensors
      */
      virtual void SetSensorList(std::vector<Sensor*>);


      virtual void Show(const long frame);
  private:
       Camera* m_camRight;
       std::string m_nameRight;
       Camera* m_camLeft;
       std::string m_nameLeft;

       bool m_mapInitialized;
       Halcon::Hobject m_mapRight;
       Halcon::Hobject m_mapLeft;
       Halcon::HTuple m_calibRight;
       Halcon::HTuple m_calibLeft;

       Halcon::HTuple m_relPoseLeft2Right;
       Halcon::HTuple m_relPoseRect;
       Halcon::HTuple camposerectLeft;
       Halcon::HTuple camposerectRight;

   };
}


#endif /*STEREOSENSOR_H*/

