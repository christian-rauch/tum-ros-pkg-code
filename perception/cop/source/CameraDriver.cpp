/*
 * Copyright (C) 2009 by Ulrich Friedrich Klank <klank@in.tum.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


  /************************************************************************
                        CameraDriver.cpp - Copyright klank

**************************************************************************/

#include "CameraDriver.h"
#ifdef HALCONIMG
#include "cpp/HalconCpp.h"
#endif
#include "XMLTag.h"

#ifdef BOOST_THREAD
#include "boost/bind.hpp"
#define BOOST(A) A
#else
#define BOOST(A)
#endif


#ifdef BOOST_THREAD
    /*printf("Sleeping\n");*/
void Sleeping(long ms)
 {
#ifdef BOOST_1_35
  BOOST(boost::system_time t);
#else
  boost::xtime t;
#endif

#ifdef BOOST_1_35
  BOOST(t = get_system_time());
  BOOST(t += boost::posix_time::seconds((double)ms / 1000));
#else
  boost::xtime_get(&t, boost::TIME_UTC);
  t.nsec += ms * 1000000;  //TODO Check
#endif
    boost::thread::sleep(t);
 }
#else
#endif


using namespace cop;

extern volatile bool g_stopall;

// Constructors/Destructors
//

#define XML_ATTRIBUTE_HASPTU "HasPTU"
#define XML_ATTRIBUTE_ISSTOC "IsSTOC"
#define XML_ATTRIBUTE_GRABBERNAME "GrabberName"
#define XML_ATTRIBUTE_CAMERATYPE "CameraType"
#define XML_ATTRIBUTE_CAMCOLOR "CamColor"
#define XML_ATTRIBUTE_IMGWIDTH "SetImageWidth"
#define XML_ATTRIBUTE_IMGHEIGHT "SetImageHeight"
#define XML_ATTRIBUTE_PANTOTILTHEIGHT "PanToTiltHeight"
#define XML_ATTRIBUTE_PANTOTILTWIDTH  "PanToTiltWidth"
#define XML_ATTRIBUTE_TILTTOENDHEIGHT "TiltToEndHeight"
#define XML_ATTRIBUTE_TILTTOENDWIDTH  "TiltToEndWidth"
#define XML_ATTRIBUTE_CAMPORT         "CamPort"
#define XML_ATTRIBUTE_CALIBFILE       "CalibFileName"
#define XML_ATTRIBUTE_HRESOLUTION     "Hresolution"
#define XML_ATTRIBUTE_VRESOLUTION     "Vresolution"
#define XML_ATTRIBUTE_STARTROW        "Startrow"
#define XML_ATTRIBUTE_STARTCOLUMN     "Startcolumn"
#define XML_ATTRIBUTE_FIELD           "Field"
#define XML_ATTRIBUTE_BITSPERCHANNEL  "Bitsperchannel"
#define XML_ATTRIBUTE_COLORSPACE       "Colospace"
#define XML_ATTRIBUTE_GAIN            "Gain"
#define XML_ATTRIBUTE_DEVICE          "Device"
#define XML_ATTRIBUTE_EXTERNALTRIGGER "Externaltrigger"
#define XML_ATTRIBUTE_LINEIN          "Linein"


CameraDriver::CameraDriver ( XMLTag* ConfigFile) :
  Camera(ConfigFile),
  m_hasPTU(false),
  m_isSTOC(false),
#ifdef PTU_USED
  m_ptuClient(NULL),
#endif
  m_port(0),
  m_imageWidth(0),
  m_imageHeight(0),
  m_fg(NULL),
  m_grabbing(false),
  m_isYUV(false)
{
#ifdef _DEBUG
  printf("In CameraDriverConstructor:\n");
#endif
  try
  {
    if(ConfigFile != NULL)
    {
      try
      {
        XMLTag* tag = ConfigFile->GetChild(XML_NODE_RELPOSE);
        if(tag != NULL)
        {
          m_relPose = RelPoseFactory::FRelPose(tag);
          if(m_relPose != NULL)
          {
#ifdef _DEBUG
          printf("  CD: Camera is localized at position %ld\n", m_relPose->m_uniqueID);
#endif
          }
#ifdef _DEBUG
          else
            printf("  CD: Camera has no location\n");
#endif
        }
        else
        {
#ifdef _DEBUG
          printf("  CD: Camera has no location\n");
#endif
        }
      }
      catch(...)
      {
        printf("  CD: error Reading RelPose\n");
      }
      m_hasPTU = ConfigFile->GetPropertyInt(XML_ATTRIBUTE_HASPTU) != 0;
      printf("HASPTU:=%s\n",ConfigFile->GetPropertyInt(XML_ATTRIBUTE_HASPTU) ? "true" : "false");
      m_isSTOC = ConfigFile->GetPropertyInt(XML_ATTRIBUTE_ISSTOC) != 0;
      printf("IsStoc:=%s\n",ConfigFile->GetPropertyInt(XML_ATTRIBUTE_ISSTOC) ? "true" : "false");
      if(m_hasPTU)
      {
#ifdef PTU_USED
#ifdef _DEBUG
        printf("Init PTU Client\n");
#endif
        try
        {
          /* PTU Client init: the first four parameter are the metrics of the PTU, TODO get those from configFile*/
          double heightPanToTilt = ConfigFile->GetPropertyDouble(XML_ATTRIBUTE_PANTOTILTHEIGHT);
          double lengthPanToTilt = ConfigFile->GetPropertyDouble(XML_ATTRIBUTE_PANTOTILTWIDTH);
          double heightTiltToEnd = ConfigFile->GetPropertyDouble(XML_ATTRIBUTE_TILTTOENDHEIGHT);
          double lengthTiltToEnd = ConfigFile->GetPropertyDouble(XML_ATTRIBUTE_TILTTOENDWIDTH);
          m_ptuClient = new PTUClient(heightPanToTilt,lengthPanToTilt,
                        heightTiltToEnd,lengthTiltToEnd ,
                        RelPoseFactory::FRelPose(ConfigFile->GetChild(XML_NODE_RELPOSE)),
                        "roboradig30", 6665, 0);
          printf("Make a relpose from PTU\n");
          m_relPose = RelPoseFactory::FRelPose(*m_ptuClient);
        }
        catch(...)
        {
          printf("PTU Driver could not instantiated\n");
          m_ptuClient = NULL;
        }
  #endif
      }
#ifdef PTU_USED
      if(m_ptuClient == NULL)
  #endif
      {
        m_relPose = RelPoseFactory::FRelPose(ConfigFile->GetChild(XML_NODE_RELPOSE));
      }
      m_grabberName = ConfigFile->GetProperty(XML_ATTRIBUTE_GRABBERNAME);
      if(m_grabberName.length() == 0)
      {
        m_grabberName = "1394IIDC";
      }
      m_stCameraType = ConfigFile->GetProperty(XML_ATTRIBUTE_CAMERATYPE, "default");
      m_isColor = ConfigFile->GetPropertyInt(XML_ATTRIBUTE_CAMCOLOR) != 0;
      m_imageWidth = ConfigFile->GetPropertyInt(XML_ATTRIBUTE_IMGWIDTH);
      m_imageHeight = ConfigFile->GetPropertyInt(XML_ATTRIBUTE_IMGHEIGHT);
      m_port = ConfigFile->GetPropertyInt(XML_ATTRIBUTE_CAMPORT, -1);
      m_stCalibName  = ConfigFile->GetProperty(XML_ATTRIBUTE_CALIBFILE);
      m_hresolution     = ConfigFile->GetPropertyInt(XML_ATTRIBUTE_HRESOLUTION    , 1);
      m_vresolution     = ConfigFile->GetPropertyInt(XML_ATTRIBUTE_VRESOLUTION    , 1);
      m_startRow        = ConfigFile->GetPropertyInt(XML_ATTRIBUTE_STARTROW       );
      m_startColumn     = ConfigFile->GetPropertyInt(XML_ATTRIBUTE_STARTCOLUMN    );
      m_field           = ConfigFile->GetProperty(XML_ATTRIBUTE_FIELD          , "default");
      m_BitsPerChannel  = ConfigFile->GetPropertyInt(XML_ATTRIBUTE_BITSPERCHANNEL , -1);
      m_colorSpace      = ConfigFile->GetProperty(XML_ATTRIBUTE_COLORSPACE     , m_isColor ? "yuv" : "default");
      m_gain            = ConfigFile->GetPropertyInt(XML_ATTRIBUTE_GAIN           , -1);
      m_externalTrigger = ConfigFile->GetProperty(XML_ATTRIBUTE_EXTERNALTRIGGER, "default");
      m_device          = ConfigFile->GetProperty(XML_ATTRIBUTE_DEVICE, "default");
      m_lineIn          = ConfigFile->GetPropertyInt(XML_ATTRIBUTE_LINEIN         , -1);

      if(m_stCalibName.length() > 0)
      {
        ReadCamParam(m_stCalibName);
      }
    }
  }
  catch(char const* text)
  {
    printf("Error Loading CameraDriver: %s\n", text);
  }
#ifdef _DEBUG
  printf("Successfully initialized Camera Driver\n");
#endif
}

void CameraDriver::threadfunc()
{
  int i = 0;
#ifdef BOOST_THREAD
#ifdef BOOST_1_35
  BOOST(boost::system_time t);
#else
  boost::xtime t;
#endif

#ifdef BOOST_1_35
  BOOST(t = get_system_time());
#else
  boost::xtime_get(&t, boost::TIME_UTC);
#endif
#endif
  /*#define GRAY_IMAGE 0
    #define RGB_IMAGE 1
    #define YUV_IMAGE 2
    #define HSV_IMAGE 4
    #define GRAY_DISPARITY_IMAGE 8
    #define YUV_DISPARITY_IMAGE 16*/
  int image_type = m_isYUV ? (m_isSTOC ? YUV_DISPARITY_IMAGE : YUV_IMAGE ): (m_isSTOC ? GRAY_DISPARITY_IMAGE : GRAY_IMAGE);
  if( m_colorSpace.compare("default") != 0)
  {
        if(m_colorSpace.compare("rgb") != 0)
            image_type = RGB_IMAGE;
        else if(m_colorSpace.compare("yuv") != 0)
            image_type = YUV_IMAGE;
        else if(m_colorSpace.compare("gray") != 0)
            image_type = GRAY_IMAGE;
  }
  /* Special case: handles for diorect show are unique for a thread ...*/
#ifdef HALCONIMG
  if(this->m_grabberName.compare("DirectShow") == 0)
  {
        printf("open_frame_grabber(%s, %d, %d, %d, %d, %d, %d, %s, %d, %s, %d, %s, %s, %s, %d, %d)",m_grabberName.c_str(),
                                   m_hresolution,
                                   m_vresolution,
                                   m_imageWidth,
                                   m_imageHeight,
                                   m_startRow,
                                   m_startColumn,
                                   m_field.c_str(),
                                   m_BitsPerChannel,
                                   m_colorSpace.c_str(),
                                   m_gain,
                                   m_externalTrigger.c_str(),
                                   m_stCameraType.c_str(),
                                   m_device.c_str(),
                                   m_port,
                                   m_lineIn);
        Halcon::set_system("do_low_error", "true");
        m_fg = new Halcon::HFramegrabber(m_grabberName.c_str(),
                                   m_hresolution,
                                   m_vresolution,
                                   m_imageWidth,
                                   m_imageHeight,
                                   m_startRow,
                                   m_startColumn,
                                   m_field.c_str(),
                                   m_BitsPerChannel,
                                   m_colorSpace.c_str(),
                                   m_gain,
                                   m_externalTrigger.c_str(),
                                   m_stCameraType.c_str(),
                                   m_device.c_str(),
                                   m_port,
                                   m_lineIn);
  }

  Halcon::grab_image_start (m_fg->GetHandle(), -1);
#endif
  while(m_grabbing && !g_stopall)
  {
#ifdef HALCONIMG
    if(m_fg != NULL)
    {
#ifdef PTU_USED
      if(m_hasPTU && m_ptuClient != NULL)
      {
//				m_ptuClient->Read();
      }
#endif
      Halcon::Hobject* obj = new Halcon::Hobject();
      if(obj == NULL)
      {

      }
      //printf("Grabbing\n");
      try
      {
        try
        {
          Halcon::grab_image_async(obj, m_fg->GetHandle(), -1);
        }
        catch(Halcon::HException ex)
        {
          printf("Error grabbing images: %s, retrying once in a few ms\n", ex.message);
          Sleeping(500);
          Halcon::grab_image_async(obj, m_fg->GetHandle(), -1);
        }
        if(m_isSTOC)
        {
          Halcon::Hobject tmp1, tmp2;
          Halcon::decompose3(*obj, obj, &tmp1, &tmp2);
        }
        if(m_calibration.m_radialDistortionHandling)
        {
          //Halcon::HTuple chan, width, height, pointer;
          //Halcon::count_channels(*m_calibration.m_radialDistMap, &chan);
          //printf("Undistortion img: \n");
          //Halcon::disp_obj(*(m_calibration.m_radialDistMap), GetWindow()->WindowHandle());
          Halcon::map_image(*obj, *m_calibration.m_radialDistMap, obj);
          //GetWindow()->Click();
          //Halcon::disp_obj(*(m_calibration.m_radialDistMap), GetWindow()->WindowHandle());
        }
        i++;
#ifdef BOOST_THREAD
        if(i > 10000)
        {
#ifdef BOOST_1_35
          BOOST(boost::system_time t2);
#else
          boost::xtime t2;
#endif

#ifdef BOOST_1_35
          BOOST(t2 = get_system_time());
/*          cout << BOOST(t2 - t) << "for 10000 frames (not deleted frames: "<< m_temp_images.size()<<")" << std::endl;*/
#else
          boost::xtime_get(&t2, boost::TIME_UTC);
          printf("%d,%d s for 10000 frames (not deleted frames: %d)\n", t2.sec - t.sec, (t2.nsec - t.nsec) % 1000000000, m_temp_images.size());
#endif
          i = 0;
          t = t2;
        }
#endif
      }
      catch(Halcon::HException ex)
      {
        printf("Error in Grabbing: %s\n", ex.message);
        break;
      }
      Image* img = new Image(obj, image_type);
      PushBack(img);
    }
#endif
    while(m_images.size() > m_max_cameraImages)
    {
      if(DeleteImg())
        continue;
      else
      {
        printf("Camera Driver: Could not delete an image!");
        break;
      }
    }
    Sleeping(50);
  }
  printf("End Camera Thread\n");
}

CameraDriver::~CameraDriver ( )
{
  Stop();
#ifdef HALCONIMG
  delete m_fg;
#endif
#ifdef PTU_USED
  delete m_ptuClient;
#endif
}

//
// Methods
//
Reading* CameraDriver::GetReading(const long &Frame)
{
  if((signed)m_images.size() < (Frame - m_deletedOffset + 1) || m_images.size() == 0)
  {
    if(m_grabbing)
    {
      while(m_grabbing && ((signed)m_images.size() < (Frame - m_deletedOffset + 1) || m_images.size() == 0))
      {
#ifdef BOOST_THREAD
        printf("Sleeping a while and");
#ifdef BOOST_1_35
  BOOST(boost::system_time t);
#else
  boost::xtime t;
#endif

#ifdef BOOST_1_35
        BOOST(t = get_system_time());
        BOOST(t += boost::posix_time::seconds(1));
#else
        boost::xtime_get(&t, boost::TIME_UTC);
        t.sec += 1;
#endif

        boost::thread::sleep(t);
#else
#endif
        printf("waiting for the camera to start grabbing\n");
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
  /*return m_images[Frame - m_deletedOffset];*/
}

bool  CameraDriver::CanSee (RelPose &pose) const
{
  printf("Can See\n");
  if(m_relPose == NULL)
     return false;
#ifdef _DEBUG
  printf("Can a camera at Pose %ld See %ld\n",m_relPose->m_uniqueID, pose.m_uniqueID);
#endif
  if(pose.m_uniqueID == m_relPose->m_uniqueID) /*lazy people just search in front of the camera, allow it*/
    return true;
  RelPose* pose_rel = RelPoseFactory::GetRelPose(pose.m_uniqueID, m_relPose->m_uniqueID);
  Matrix m = pose_rel->GetMatrix();
  RelPoseFactory::FreeRelPose(pose_rel);
  double x = m.element(0,3);
  double y = m.element(1,3);
  double z = m.element(2,3);
  if(z > 0.0)
  {

#ifdef  HALCONIMG
    try
    {
      Halcon::HTuple R, C;
      Halcon::project_3d_point(x,y,z, m_calibration.CamParam(), &R , &C);
      if(R >= 0 && R < m_calibration.m_height
        && C >= 0 && C < m_calibration.m_width)
        return true;
    }
    catch(Halcon::HException ex)
    {
      printf("Error: %s\n", ex.message);
    }
  }
#endif
  //TODO what when the position can be in the image, but not in the center?
#ifdef _DEBUG
  printf("A Camera can not see pose (rel to camera): %f %f %f\n", x,y,z);
#endif
  return false;
}

#ifndef USE_YARP_COMM
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

/*void Camera::PublishImageOnRosTopic(const Image& img)
{
  sensor_msgs::Image msg;

}*/
#endif
bool CameraDriver::Start()
{
  if(m_grabberName.compare("1394IIDC") == 0 || m_grabberName.compare("GigEVision") == 0)
  {
#ifdef HALCONIMG
#ifdef _DEBUG
    if(m_grabberName.compare("1394IIDC") == 0)
      printf("Trying to open 1394-Camera\n");
    else
      printf("Trying to open GigEVision-Camera\n");
#endif
    if(m_stCameraType.length() == 0)
      m_stCameraType = "7:0:0";
#ifdef _DEBUG
    printf("Camera Type: %s\n", m_stCameraType.c_str());
#endif
    try
    {
      m_fg = new Halcon::HFramegrabber(m_grabberName.c_str(),
                                       m_hresolution,
                                       m_vresolution,
                                       m_imageWidth,
                                       m_imageHeight,
                                       m_startRow,
                                       m_startColumn,
                                       m_field.c_str(),
                                       m_BitsPerChannel,
                                       m_colorSpace.c_str(),
                                       m_gain,
                                       m_externalTrigger.c_str(),
                                       m_stCameraType.c_str(),
                                       m_device.c_str(),
                                       m_port,
                                       m_lineIn);
      if(m_grabberName.compare("GigEVision") == 0)
      {
         m_fg->SetFramegrabberParam("GtlGVSPDiscardIncompleteBuffers", "enable");
         m_fg->SetFramegrabberParam("AcquisitionFrameRateAbs", 2); //TODO framerate

      }
      /*if(m_stCameraType.c_str()[0] == '7')
      {
        m_fg->SetFramegrabberParam("packet_size", 8192);
        if(m_imageWidth > 0)
        {
          printf("Force Image Width to %d\n", m_imageWidth);
          m_fg->SetFramegrabberParam("horizontal_resolution",	m_imageWidth	);
        }
        if(m_imageHeight > 0)
        {
          printf("Force Image Height to %d\n", m_imageHeight);
          m_fg->SetFramegrabberParam("vertical_resolution",		m_imageHeight	);
        }
        Halcon::HTuple tup = m_fg->GetFramegrabberParam("frame_rate");
        Halcon::HTuple tup2 = m_fg->GetFramegrabberParam("packet_size");
        printf("... done: (%f fps in %d byte) \n", tup[0].D(), tup2[0].L());
      }
      else
        printf("... done \n");*/
    }
    catch(Halcon::HException &ex)
    {
      printf("Error opening camera: %s, DeviceName: %s\n", ex.message, m_device.c_str());
      return false;
    }
    catch(...)
    {
      m_fg = NULL;
      printf("Camera not connected, or errors in driver configuration\n");
      return false;
    }
#endif
    //mfg.OpenFramegrabber();
  }
  else if(m_grabberName.compare("LeutronVision") == 0)
  {
    //TODO
  }
  else if(m_grabberName.compare("DirectShow") == 0)
  {
    /* Windows handle are not usable in different threads? so build m_fg in the thread*/
  }
  else
  {
#ifdef HALCONIMG
      printf("open_frame_grabber(%s, %d, %d, %d, %d, %d, %d, %s, %d, %s, %d, %s, %s, %s, %d, %d)",m_grabberName.c_str(),
                                       m_hresolution,
                                       m_vresolution,
                                       m_imageWidth,
                                       m_imageHeight,
                                       m_startRow,
                                       m_startColumn,
                                       m_field.c_str(),
                                       m_BitsPerChannel,
                                       m_colorSpace.c_str(),
                                       m_gain,
                                       m_externalTrigger.c_str(),
                                       m_stCameraType.c_str(),
                                       m_device.c_str(),
                                       m_port,
                                       m_lineIn);

      m_fg = new Halcon::HFramegrabber(m_grabberName.c_str(),
                                       m_hresolution,
                                       m_vresolution,
                                       m_imageWidth,
                                       m_imageHeight,
                                       m_startRow,
                                       m_startColumn,
                                       m_field.c_str(),
                                       m_BitsPerChannel,
                                       m_colorSpace.c_str(),
                                       m_gain,
                                       m_externalTrigger.c_str(),
                                       m_stCameraType.c_str(),
                                       m_device.c_str(),
                                       m_port,
                                       m_lineIn);
#endif
  }
  m_grabbing = true;

#ifdef BOOST_THREAD
  m_grabbingThread = new thread(bind(&CameraDriver::threadfunc, this)) ;
#else
#endif
  return true;
}
bool CameraDriver::Stop()
{
  m_grabbing = false;
  printf("\nTrying to Stop Cemaera Grabbing Thread (Sleeping 1s)\n");
  Sleeping(1000);
  printf("\nJoining Cemaera Grabbing Thread \n");
#ifdef BOOST_THREAD
  m_grabbingThread->join();
#ifdef HALCONIMG
  Halcon::close_all_framegrabbers();
#endif
  delete m_grabbingThread;
#else
#endif
  return false;
}

XMLTag* CameraDriver::Save()
{
  XMLTag* tag = new XMLTag(GetName());
  Camera::SaveTo(tag);
  tag->AddProperty(XML_ATTRIBUTE_CALIBFILE, m_stCalibName);
  tag->AddProperty(XML_ATTRIBUTE_HASPTU, m_hasPTU ? 1 : 0);
  tag->AddProperty(XML_ATTRIBUTE_GRABBERNAME, m_grabberName);
  tag->AddProperty(XML_ATTRIBUTE_CAMERATYPE, m_stCameraType);
  tag->AddProperty(XML_ATTRIBUTE_CAMCOLOR, (int)m_isColor);
  tag->AddProperty(XML_ATTRIBUTE_IMGWIDTH, m_imageWidth);
  tag->AddProperty(XML_ATTRIBUTE_IMGHEIGHT, m_imageHeight);
  tag->AddProperty(XML_ATTRIBUTE_CAMPORT, m_port);

  tag->AddProperty(XML_ATTRIBUTE_HRESOLUTION    ,m_hresolution);
  tag->AddProperty(XML_ATTRIBUTE_VRESOLUTION    ,m_vresolution);
  tag->AddProperty(XML_ATTRIBUTE_STARTROW       ,m_startRow);
  tag->AddProperty(XML_ATTRIBUTE_STARTCOLUMN    ,m_startColumn);
  tag->AddProperty(XML_ATTRIBUTE_FIELD          ,m_field);
  tag->AddProperty(XML_ATTRIBUTE_BITSPERCHANNEL ,m_BitsPerChannel);
  tag->AddProperty(XML_ATTRIBUTE_COLORSPACE     ,m_colorSpace);
  tag->AddProperty(XML_ATTRIBUTE_GAIN           ,m_gain);
  tag->AddProperty(XML_ATTRIBUTE_DEVICE         ,m_device);
  tag->AddProperty(XML_ATTRIBUTE_EXTERNALTRIGGER,m_externalTrigger);
  tag->AddProperty(XML_ATTRIBUTE_LINEIN         ,m_lineIn);

#ifdef PTU_USED
  if(m_hasPTU && m_ptuClient != NULL)
  {
    tag->AddProperty(XML_ATTRIBUTE_PANTOTILTHEIGHT,m_ptuClient->m_heightPanToTilt);
    tag->AddProperty(XML_ATTRIBUTE_PANTOTILTWIDTH,   m_ptuClient->m_lengthPanToTilt);
    tag->AddProperty(XML_ATTRIBUTE_TILTTOENDHEIGHT,  m_ptuClient->m_heightTiltToEnd);
    tag->AddProperty(XML_ATTRIBUTE_TILTTOENDWIDTH, m_ptuClient->m_lengthTiltToEnd);
  }
#endif
  return tag;
}


void CameraDriver::Show(long frame)
{
#ifdef HALCONIMG
  try
  {
    if(m_win == NULL)
      GetWindow();
    Image* img = GetImage(frame);
    if(img != NULL)
    {
      Halcon::Hobject* obj = img->GetHImage();
      Halcon::HTuple p,t,height, width, chan;
      Halcon::get_image_pointer1(*obj, &p,&t,&width, &height);
      #ifdef _DEBUG
      printf("Showing with: height %d width %d\n", height[0].I(), width[0].I());
      #endif
      if(width[0].I() < 700)
        m_win->SetWindowExtents(20,10,(int)(width[0].I() * 1.5 ) + 10, (int)(height[0].I()* 1.5 ) + 20);
      else
        m_win->SetWindowExtents(20,10,(int)(width[0].I() * 0.5 ) + 10, (int)(height[0].I()* 0.5 ) + 20);

      m_win->SetPart(0,0,height, width);
      //TODO Check if this is not colored!


      Halcon::count_channels(*obj, &chan);
      if(chan[0].I() == 3 && m_isYUV)
      {
        Halcon::Hobject obj_temp1,obj_temp2,obj_temp3;
        Halcon::decompose3(*obj, &obj_temp1,&obj_temp2,&obj_temp3);
        Halcon::trans_to_rgb(obj_temp1,obj_temp2,obj_temp3,&obj_temp1,&obj_temp2,&obj_temp3, "yuv");
        Halcon::compose3(obj_temp1,obj_temp2,obj_temp3,&obj_temp1);
        Halcon::disp_obj(obj_temp1, m_win->WindowHandle());
      }
      else
        Halcon::disp_obj(*obj, m_win->WindowHandle());
      img->Free();
    }
  }
  catch(Halcon::HException ex)
  {
    printf("Showing not possible: %s \n", ex.message);
  }
#endif
}

