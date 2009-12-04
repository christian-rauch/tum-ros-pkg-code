

#include "StereoSensor.h"
#include "XMLTag.h"
#include "Camera.h"
#include "cpp/HalconCpp.h"

#define XML_PROPERTY_SENSORNAME_RIGHT "SensorName_CamRight"
#define XML_PROPERTY_SENSORNAME_LEFT "SensorName_CamLeft"
#define XML_PROPERTY_RELPOSEFILENAME "RelPoseFileName"

extern volatile bool g_stopall;


// Procedures
void binocular_xyz_proc (Halcon::Hobject ImageL, Halcon::Hobject MapL, Halcon::Hobject ImageR,
    Halcon::Hobject MapR, Halcon::Hobject *X, Halcon::Hobject *Y, Halcon::Hobject *Z,
    Halcon::Hobject *RegionZ, Halcon::HTuple CamParamRect1, Halcon::HTuple CamParamRect2,
    Halcon::HTuple RelPoseRect)
{
  using namespace Halcon;

  // Local iconic variables
  Hobject  ImageMappedL, ImageMappedR, Rectangle;
  Hobject  ImageReducedR, ImageReducedL, ImageEquHistoR, ImageEquHistoL;
  Hobject  Disparity, Score, RegionComplement;

  map_image(ImageL, MapL, &ImageMappedL);
  map_image(ImageR, MapR, &ImageMappedR);
  gen_rectangle1(&Rectangle, 200, 200, 1600, 2200);
  reduce_domain(ImageMappedR, Rectangle, &ImageReducedR);
  reduce_domain(ImageMappedL, Rectangle, &ImageReducedL);
  equ_histo_image(ImageReducedR, &ImageEquHistoR);
  equ_histo_image(ImageReducedL, &ImageEquHistoL);
  binocular_disparity_mg(ImageEquHistoL, ImageEquHistoR, &Disparity, &Score, 1, 30,
      1, -100, "false", "default_parameters", "fast");
  disparity_image_to_xyz(Disparity, &(*X), &(*Y), &(*Z), CamParamRect1, CamParamRect2,
      RelPoseRect);
  threshold((*Z), &(*RegionZ), 0.01, 10);
  complement((*RegionZ), &RegionComplement);
  overpaint_region((*Z), RegionComplement, 0, "fill");
  overpaint_region((*Y), RegionComplement, 0, "fill");
  overpaint_region((*X), RegionComplement, 0, "fill");
  return;
}

using namespace cop;

StereoSensor::StereoSensor() :
  m_camRight(NULL),
  m_nameRight("uninit_1234679_right"),
  m_camLeft(NULL),
  m_nameLeft("uninit_1234679_left"),
  m_mapInitialized(false)
{

}


StereoSensor::~StereoSensor()
{
}

bool StereoSensor::Start()
{
  return true;
}

bool StereoSensor::Stop()
{
  return true;
}

XMLTag* StereoSensor::Save()
{
  XMLTag* tag = new XMLTag(GetName());
  Sensor::SaveTo(tag);
  tag->AddProperty(XML_PROPERTY_SENSORNAME_RIGHT, m_nameRight);
  tag->AddProperty(XML_PROPERTY_SENSORNAME_LEFT, m_nameLeft);
  return tag;
}
void StereoSensor::SetData(XMLTag* tag)
{
  m_nameRight = tag->GetProperty(XML_PROPERTY_SENSORNAME_RIGHT, m_nameRight);
  m_nameLeft = tag->GetProperty(XML_PROPERTY_SENSORNAME_LEFT, m_nameLeft);
  std::string relPoseFileName = tag->GetProperty(XML_PROPERTY_RELPOSEFILENAME, "");
  try
  {
    Halcon::read_pose(relPoseFileName.c_str(),  &m_relPoseLeft2Right);
  }
  catch(...)
  {
    printf("StereoSensor::SetData: Error reading relaitive pose  %s -> %s\n", m_nameLeft.c_str(), m_nameRight.c_str());
  }
}


bool StereoSensor::RequiresSensorList()
{
  if(m_camRight == NULL || m_camLeft == NULL)
    return true;
  else
    return false;
}
/**
*  SetSensorList
*  @brief Stereo works as a composition of names sensors
*/
void StereoSensor::SetSensorList(std::vector<Sensor*> cameras)
{
  m_mapInitialized = false;
  for(std::vector<Sensor*>::const_iterator it = cameras.begin();
            it !=  cameras.end(); it++)
  {
    if((*it)->IsCamera() )
    {
      if( (*it)->GetSensorID().compare(m_nameLeft) == 0 )
      {
        m_camLeft = (Camera*)(*it);
      }
      if( (*it)->GetSensorID().compare(m_nameRight) == 0 )
      {
        m_camRight = (Camera*)(*it);
      }
    }
  }
}

/**
* GetReading
* @param Frame frame number, to specify an offset or a specific file
* @throws char* with an error message in case of failure
*/
Reading* StereoSensor::GetReading(const long &Frame)
{
  Reading* result = NULL;
  /** TODO buffer!: this only implements the creation on query*/
  if(m_camRight != NULL && m_camLeft != NULL && !g_stopall)
  {
    Image* rightImg = (Image*)m_camRight->GetReading(Frame);
    Image* leftImg = (Image*)m_camLeft->GetReading(Frame);
    try
    {
      if(!m_mapInitialized)
      {
        Halcon::HTuple leftCalibBef = m_camLeft->m_calibration.CamParam();
        Halcon::HTuple  rightCalibBef = m_camRight->m_calibration.CamParam();
        Halcon::gen_binocular_rectification_map(&m_mapLeft, &m_mapRight,
                                                leftCalibBef, rightCalibBef,
                                                 m_relPoseLeft2Right,
                                                 1, "geometric", "bilinear",
                                           &m_calibLeft, &m_calibRight, &camposerectLeft,
                                            &camposerectRight, &m_relPoseRect);
        m_mapInitialized = true;
      }
      Halcon::Hobject X,Y,Z, region;
      binocular_xyz_proc(*leftImg->GetHImage(), m_mapLeft, *rightImg->GetHImage(), m_mapRight, &X, &Y, &Z, &region, m_calibLeft, m_calibRight, m_relPoseRect);
      /**  TODO: fill result
        result = new SwissRangerReading, DisparityBla, PointCloud, ...();
      */
    }
    catch(...)
    {
      printf("Error in StereoSensor::GetReading !\n\n");
    }
  }
  return result;
}

void StereoSensor::Show(const long frame)
{
  /** TODO
        publish results on a topic?,
        static topic?
        topic name as parameter / rosparameter?
        + concept of rosparam <-> XMLTag
  */

}


bool StereoSensor::CanSee(RelPose &pose) const
{
  if(m_camRight != NULL && m_camLeft != NULL && !g_stopall)
    return m_camRight->CanSee(pose) && m_camLeft->CanSee(pose);
  else
    return false;
}


