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


/**
*	klank 18.11.2009
*/

#ifndef SENSOR_H
#define SENSOR_H

#include <vector>
#include "Reading.h"


#ifdef BOOST_THREAD
#include <boost/thread.hpp>
using namespace boost;
#else
#endif

#define XML_NODE_SENSOR "Sensor"
#define XML_PROPERTY_SENSORNAME "SensorName"


namespace cop
{
  class RelPose;
  class XMLTag;
  /**
  *   Class Camera
  *   @brief Provides an interface for camera usage
  */
  class Sensor
  {
  public:

      /***
      *   @brief Constructor with pose, initializes parameters
      */
      Sensor() :
         m_FrameCount(0),
         m_deletedOffset(0),
         m_max_cameraImages(2)
      {}
      /***
      *   @brief Constructor with pose, initializes parameters, sets the sensors pose
      */
      Sensor(RelPose* pose) :
         m_relPose(pose),
         m_FrameCount(0),
         m_deletedOffset(0),
         m_max_cameraImages(2)
      {};
      /**
      *   The destructor is virtual
      */
      virtual ~Sensor();

      /**
      *   @brief Load sensor parameter from xml
      *   Only derivatives of Camera can use this constructor to initialize values
      */
      static Sensor* SensorFactory(XMLTag* tag);

  public:
      virtual void SaveTo(XMLTag* tag);
      /**
      *  Get Type of the camera by its Name
      */
      virtual std::string GetName() const {return XML_NODE_SENSOR;};
      /**
      *  Show
      *  @param Frame frame number, to specify an temporal offset or a specific file
      *  @brief should display the sensors currenbt reading, if wanted
      */
      virtual void Show(const long frame = -1){}
      /**
      * GetReading
      * @param Frame frame number, to specify an offset or a specific file
      * @throws char* with an error message in case of failure
      */
      virtual Reading*	GetReading(const long &Frame = -1) = 0;
      /**
      * CanSee
      * Checks if a pose is inside the view of this sensor
      * @param pose pose that has to be looked at
      */
      virtual bool CanSee (RelPose &pose) const {return false;}
      /**
      *    Start
      *   @brief overwrite to start up the data reading, is called at least once after creation
      */
      virtual bool	Start()				= 0;
      /**
      *    Start
      *   @brief overwrite to stop call this in the destructor if necessary, will be used to shut down cameras
      */
      virtual bool	Stop()				= 0;
      /**
       *   @return the pose of this sensor
       */
      RelPose* GetRelPose(){return m_relPose;}

      virtual XMLTag* Save() = 0;
      /**
      *   Can this Sensor be used like a camera, (incl. Calibration, Showing, usw.)
      */
      virtual bool IsCamera()const  {return false;}

      /**
      * GetSensorID
      * This contains optinally an identifier for the sensor (Loaded from)
      */
      std::string GetSensorID(){return m_stSensorName;}

      /**
      * m_stSensorName
      *  @brief Stereo works as a composition of names sensors
      */
      virtual bool RequiresSensorList(){return false;}
      /**
      *  SetSensorList
      *  @brief Stereo works as a composition of names sensors
      */
      virtual void SetSensorList(std::vector<Sensor*>){};


      /**
      * DeleteReading
      *  @brief removes entries of the reading buffer, this function will not release still references functions (@see cop::Reading::Free)
      *
      */
      virtual bool DeleteReading();
  protected:
     void PushBack(Reading* img);
     Reading* GetReading_Lock(size_t index);
     virtual void SetData(XMLTag* tag);

  protected:
      RelPose*		m_relPose;
      std::vector<Reading*> m_images;
      std::vector<Reading*> m_temp_images;
      long				m_FrameCount;
      long				m_deletedOffset;
      unsigned long m_max_cameraImages;
      std::string m_stSensorName;

  #ifdef BOOST_THREAD
      boost::mutex m_mutexImageList;
  #else
  #endif
  };
}
#endif /*CAMERA_H*/
