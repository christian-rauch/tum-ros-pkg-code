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
                        NetworkCamera.cpp - Copyright klank

**************************************************************************/

#include "NetworkCamera.h"
#ifdef HALCONIMG
#include "cpp/HalconCpp.h"
#endif
#include "XMLTag.h"
#ifdef USE_YARP_CAMERA
#include <yarp/os/all.h>
#endif
#ifdef BOOST_THREAD
#include "boost/bind.hpp"
#define BOOST(A) A
#else
#define BOOST(A)
#endif


extern volatile bool g_stopall;

// Constructors/Destructors
//
#define XML_ATTRIBUTE_CAMERAPORTNAME "CameraPortname"
#define XML_ATTRIBUTE_CALIBFILE       "CalibFileName"
/*#define XML_ATTRIBUTE_HASPTU "HasPTU"
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
#define XML_ATTRIBUTE_LINEIN          "Linein"*/


NetworkCamera::NetworkCamera ( XMLTag* ConfigFile) :
  Camera(),
  m_hasPTU(false),
  m_isSTOC(false),
#ifdef PTU_USED
  m_ptuClient(NULL),
#endif
  m_port(0),
  m_imageWidth(0),
  m_imageHeight(0),
#ifdef USE_YARP_CAMERA
  m_inputPort(NULL),
#endif
  m_grabbing(false),
  m_isYUV(false)
{

#ifdef _DEBUG
  printf("In NetworkCameraConstructor:\n");
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

      m_stPortName = ConfigFile->GetProperty(XML_ATTRIBUTE_CAMERAPORTNAME, "/copCamera");
      std::string stFileName  = ConfigFile->GetProperty(XML_ATTRIBUTE_CALIBFILE);
      if(stFileName.length() > 0)
      {
        ReadCamParam(stFileName);
      }
    }
    Start();
  }
  catch(char const* text)
  {
    printf("Error Loading NetworkCamera: %s\n", text);
  }
}

void NetworkCamera::threadfunc()
{
//  int i = 0;
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
#ifdef USE_YARP_CAMERA
  int image_type = m_isYUV ? (m_isSTOC ? YUV_DISPARITY_IMAGE : YUV_IMAGE ): (m_isSTOC ? GRAY_DISPARITY_IMAGE : GRAY_IMAGE);
#endif

  while(m_grabbing && !g_stopall)
  {
#ifdef HALCONIMG
      //printf("Grabbing\n");
#ifdef USE_YARP_CAMERA
      yarp::sig::FlexImage* flex_img  = NULL;
      try
      {
        if(m_inputPort->getPendingReads() > 1)
          printf("To many pending reads %d\n", m_inputPort->getPendingReads());

        flex_img  = m_inputPort->read(false);
      }
      catch(...)
      {
        printf("Yarp error\n");
        continue;
      }
      if(flex_img == NULL)
        continue;
      Halcon::Hobject* obj = new Halcon::Hobject();
      if(obj == NULL)
      {
          printf("Out of memory\n");
          break;
      }
      try
      {
        unsigned char* img_start = flex_img->getRawImage();
        m_imageWidth = flex_img->width();
        m_imageHeight = flex_img->height();
        if(flex_img->getRowSize() > m_imageWidth)
        {
          Halcon::Hobject red,green,blue;
          Halcon::gen_image_const(&red, "byte", m_imageWidth, m_imageHeight);
          Halcon::gen_image_const(&green, "byte", m_imageWidth, m_imageHeight);
          Halcon::gen_image_const(&blue, "byte", m_imageWidth, m_imageHeight);
          unsigned char* pBase = img_start ;
          for(int r = 0; r < m_imageHeight; r++)
          {
            for (int c = 0; c < m_imageWidth; c++)
            {
              Halcon::set_grayval(red,r,c, pBase[0]);
              Halcon::set_grayval(green,r,c, pBase[1]);
              Halcon::set_grayval(blue,r,c, pBase[2]);
              pBase += 3;
            }
          }
          Halcon::compose3(red,green,blue, obj);
        }
        else
        {
          Halcon::gen_image1(obj, "byte", m_imageWidth, m_imageHeight, (Hlong)img_start );
        }
        if(m_calibration.m_radialDistortionHandling)
        {
          Halcon::map_image(*obj, *m_calibration.m_radialDistMap, obj);
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
#endif
    while(m_images.size() > MAX_CAMERA_IMAGES)
    {
      if(DeleteImg())
        continue;
      else
      {
        printf("Camera Driver: Could not delete an image!");
        break;
      }
    }
#ifdef BOOST_THREAD
    /*printf("Sleeping\n");*/
#ifdef BOOST_1_35
  BOOST(boost::system_time t3);
#else
  boost::xtime t3;
#endif

#ifdef BOOST_1_35
  BOOST(t3 = get_system_time());
  BOOST(t3 += boost::posix_time::millisec(5));
#else
/*  Show(-1);*/
  boost::xtime_get(&t3, boost::TIME_UTC);
  t3.nsec +=  5000000;  //TODO Check
#endif

    boost::thread::sleep(t3);
#else
#endif
#else
  break;
#endif
  }
  printf("End Camera Thread\n");
}

NetworkCamera::~NetworkCamera ( )
{
  Stop();
#ifdef USE_YARP_CAMERA
  delete m_inputPort;
#endif
}

//
// Methods
//
Image* NetworkCamera::GetImage(const long &Frame)
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
      printf("unexspected error\n");
      throw "Asking for images from a camera that has no images";
    }
  }
  if(Frame == -1 || (Frame - m_deletedOffset < 0 && (unsigned)(Frame - m_deletedOffset) >= m_images.size()))
  {
    return GetImage_Lock(m_images.size() -1);
    /*return m_images[m_images.size() -1];*/
  }
  return GetImage_Lock(Frame - m_deletedOffset);
  /*return m_images[Frame - m_deletedOffset];*/
}

bool  NetworkCamera::CanSee(RelPose &pose)
{
  return true;
}

double  NetworkCamera::LookAt(RelPose &pose)
{
  return 0.0;
}

bool NetworkCamera::Start()
{
#ifdef USE_YARP_CAMERA
  m_inputPort = new BufferedPort<yarp::sig::FlexImage>;
  m_inputPort->setStrict(false);

  bool ret= m_inputPort->open(m_stPortName.c_str());

    if (!ret)
    {
        printf("Error: port failed to open, quitting.\n");
        return false;
    }
    else
      m_grabbing = true;

  #ifdef BOOST_THREAD
    m_grabbingThread = new thread(bind(&NetworkCamera::threadfunc, this)) ;
  #else
  #endif
#endif
  return true;
}
bool NetworkCamera::Stop()
{
  m_grabbing = false;
  printf("\nTrying to Stop Cemaera Grabbing Thread\n");
#ifdef BOOST_THREAD
#ifdef BOOST_1_35
  BOOST(boost::system_time t);
#else
  boost::xtime t;
#endif

#ifdef BOOST_1_35
  BOOST(t = get_system_time());
  BOOST(t += boost::posix_time::millisec(100));
#else
  boost::xtime_get(&t, boost::TIME_UTC);
  t.nsec +=  500000000;  //TODO Check
#endif

    boost::thread::sleep(t);
#else
#endif
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

XMLTag* NetworkCamera::Save()
{
  XMLTag* tag = new XMLTag(GetName());
  Camera::SaveTo(tag);
  tag->AddProperty(XML_ATTRIBUTE_CAMERAPORTNAME         ,m_stPortName);

  return tag;
}


void NetworkCamera::Show(long frame)
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

