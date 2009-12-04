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
                        ImageInputSystem.h - Copyright klank


**************************************************************************/


#ifndef IMAGEINPUTSYSTEM_H
#define IMAGEINPUTSYSTEM_H

#include <string>
#include <vector>
#include "RelPoseFactory.h"

#include "Sensor.h"

#define XML_NODE_IMAGEINPUTSYSTEM "ImageInputSystem"


namespace cop
{
  /**
    * class ImageInputSystem
    * @brief manages different cameras and the camera selection
    */
  class ImageInputSystem
  {
  public:

    // Constructors/Destructors
    //


    /**
     * Empty Constructor
     */
    ImageInputSystem (  XMLTag* ConfigFile );

    /**
     * Empty Destructor
     */
    virtual ~ImageInputSystem ( );

    XMLTag* Save();

    /**
    *
    */
    void AddSensor(Sensor* cam);
    Sensor* GetSensor(unsigned int index){if(index < m_cameras.size())return m_cameras[index];else return NULL;}

    /**
     * GetBestCamera
     *  @brief Selected depening on the position that should be observed a camera and returns it.
     *	@return Camera
     *	@param  pose the pose that should be observed
     *  @param camera index will be returned
     *  @param offset specify this if the first camera is not good
     *	@param  camera retrieves the selected camera
     *   @throws char* with an error message in case of failure
     */

    std::vector<Sensor*> GetBestSensor(RelPose &pose);
    std::vector<Sensor*> GetAllSensors() const {return m_cameras;};
  private:
    std::vector<Sensor*> m_cameras;
  };
}
#endif // IMAGEINPUTSYSTEM_H
