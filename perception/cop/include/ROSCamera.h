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

 
#ifndef ROSCOPCamera_H
#define ROSCOPCamera_H

#include "Camera.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>


#define XML_NODE_ROSCOPCAMERA "ROSCOPCamera"
/**
*   Class ROSCOPCamera
*   @brief Provides a connection to ros-Image topics
*/
class ROSCOPCamera : public Camera
{
public:
    /***
    *   @brief Constructor, initializes camera from xml file
    */
    ROSCOPCamera(XMLTag* tag);

    /**
    *   The destructor
    */
    virtual ~ROSCOPCamera();

public:
   virtual std::string GetName(){return XML_NODE_ROSCOPCAMERA;};

    /**
    * GetImage
    * @param Frame frame number, to specify an offset or a specific file
    * @throws char* with an error message in case of failure
    */
    virtual Image*	GetImage(const long &Frame);


    virtual bool	CanSee(RelPose &pose){return true; /*TODO implement*/};
    virtual double  LookAt(RelPose &pose){return true;};
    virtual XMLTag* Save();


private:
    void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
    virtual bool	Start();
    virtual bool	Stop();
    std::string m_stCalibName;
    std::string m_stImageTopic;
    bool m_grabbing;

};

#endif /*ROSCOPCamera_H*/
