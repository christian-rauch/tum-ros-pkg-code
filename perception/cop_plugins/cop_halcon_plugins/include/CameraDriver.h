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
                        CameraDriver.h - Copyright klank

**************************************************************************/


#ifndef CAMERADRIVER_H
#define CAMERADRIVER_H
#define MAX_CAMERA_IMAGES 2

#include "Camera.h"
#ifdef PTU_USED
#include "PTUClient.h"
#endif

#include <string>

namespace Halcon
{
    class HFramegrabber;
}
#define XML_NODE_CAMERADRIVER "CameraDriver"

#ifdef BOOST_THREAD
#include <boost/thread.hpp>
using namespace boost;
#else
#endif

namespace cop
{
  class XMLTag;
  /**
    * class CameraDriver
    * @brief Implements an reduced interface for the halcon acquisition interface
    */
  class CameraDriver : public Camera
  {
  public:

    // Constructors/Destructors
    //


     /**************************************************/
     /** Constructor Camera Driver                     *
     ***************************************************
     *  \param stConfigFile xml-configuration file,
                  contains info about ptu, cameratype...
     ***************************************************/
      CameraDriver ();
      /**
      *  Get Type of the camera by its Name
      */
      virtual std::string GetName() const{return XML_NODE_CAMERADRIVER;};
      /**
     * Empty Destructor
     */
      virtual ~CameraDriver ( );
      /**
      * GetImage
      * @param Frame frame number, to specify an offset or a specific file
      * @throws char* with an error message in case of failure
      */
      virtual Reading*  GetReading(const long &Frame);

      virtual bool  Start();
      virtual bool  Stop();

      virtual XMLTag* Save();
      virtual void SetData(XMLTag* tag);

      virtual void Show(const long frame);
  protected:
     void threadfunc();
  private:
      std::string m_stCalibName;
      bool m_hasPTU;
      bool m_isSTOC;
      std::string m_grabberName;
      std::string m_stCameraType;
      bool m_isColor;
      int m_port;
      int m_imageWidth;
      int m_imageHeight;
      Halcon::HFramegrabber* m_fg;
      bool m_grabbing;
      bool m_isYUV;
      int m_fps;

      int         m_hresolution;
      int         m_vresolution;
      int         m_startRow;
      int         m_startColumn;
      std::string m_field;
      int         m_BitsPerChannel;
      std::string m_colorSpace;
      int         m_gain;
      std::string m_device;
      std::string m_externalTrigger;
      int         m_lineIn;

  #ifdef PTU_USED
      PTUClient* m_ptuClient;
  #endif

  #ifdef BOOST_THREAD
      boost::thread* m_grabbingThread;
  #else
  #endif

  };
}
#include <sensor_msgs/Image.h>


namespace cop
{
#define XML_NODE_NAME "CameraDriverRelay"
  class CameraDriverRelay : public SensorNetworkRelay<CameraDriver, sensor_msgs::Image>
  {
  public:
    CameraDriverRelay() {};
    virtual std::string GetName() const {return XML_NODE_NAME;}
    sensor_msgs::Image ConvertData (Reading* img);
    virtual XMLTag* Save();
    virtual void SetData(XMLTag* tag);
  };
}
#endif // CAMERADRIVER_H
