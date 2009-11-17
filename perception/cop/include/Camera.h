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
*	klank 31.10.2007
*/

#ifndef CAMERA_H
#define CAMERA_H

#include "Image.h"
#include <vector>
#include "RelPoseFactory.h"

#ifdef BOOST_THREAD
#include <boost/thread.hpp>
using namespace boost;
#else
#endif

#ifdef HALCONIMG
namespace Halcon
{
    class HWindow;
    class HTuple;
}
#endif

#define XML_NODE_CALIBRATION "CalibMatrix"
#define XML_NODE_WIDTH "Width"
#define XML_NODE_HEIGHT "Height"
#define XML_NODE_CAMERA "Camera"


#ifdef HALCONIMG
namespace Halcon
{
    class HTuple;
}
#endif

/**
*   Class Calibration
*   @brief stores the calibration and provides converions to different formats, stores a radial distortion map
*/
class Calibration
{
public:
    Calibration():m_radialDistortionHandling(false){};
    Calibration(XMLTag* tag);
    ~Calibration();
    std::vector<double> m_calibrationmatrix;
    int m_width;
    int m_height;
    bool m_radialDistortionHandling;
#ifdef HALCONIMG
    void SetCamParam(Halcon::HTuple& t);
    /**
    * CamParam
    *
    * @throws char* with an error message in case of failure
    */
    Halcon::HTuple CamParam();
    /**
    * CamParam
    * @param width specifies the wished images width for which the parameters should be converted, not yet tested
    * @param height specifies the wished images height for which the parameters should be converted, not yet tested
    * @throws char* with an error message in case of failure
    */
    Halcon::HTuple CamParam(int width, int height);


    /**
    * CamParam
    * @param scale_image isotropic image scale apllied to the camparams by this factor
    * @throws char* with an error message in case of failure
    */
    Halcon::HTuple CamParam(double scale_image);
    /**
    * CamMatrix
    *
    * @throws char* with an error message in case of failure
    **/
    Halcon::HTuple CamMatrix();

    Halcon::HTuple* m_camdist;
    Halcon::Hobject* m_radialDistMap;
#endif
    void SaveTo(XMLTag* tag);
    //TODO usw
};

/**
*   Class Camera
*   @brief Provides an interface for camera usage
*/
class Camera
{
public:
    /***
    *   @brief Empty Constructor, initializes parameters
    */
    Camera();
    /***
    *   @brief Constructor with pose, initializes parameters, sets the camera pose
    */
    Camera(RelPose* pose) :
#ifdef HALCONIMG
       m_win(NULL),
#endif
       m_relPose(pose),
       m_FrameCount(0),
       m_deletedOffset(0)
    {};
    /**
    *   The destructor is virtual
    */
    virtual ~Camera();
protected:
    /**
    *   @brief Load camera parameter from xml
    *   Only derivatives of Camera can use this constructor to initialize values
    */
    Camera(XMLTag* tag);

public:
    /**
    *   CameraFactory
    *   @brief static function to create cameras
    *   @param tag contains the specification of the camera
    */
    static Camera*	CamFactory(XMLTag* tag);
    void			SaveTo(XMLTag* tag);
    /**
    *  Get Type of the camera by its Name
    */
    virtual std::string GetName(){return XML_NODE_CAMERA;};

    Calibration		m_calibration;
    int				LastAskedFrame;
    /**
    * GetImage
    * @param Frame frame number, to specify an offset or a specific file
    * @throws char* with an error message in case of failure
    */
    virtual Image*	GetImage(const long &Frame) = 0;
  /**
  * CanSee
  * Checks if a pose is inside the view of this camera
  */

    virtual bool	CanSee(RelPose &pose) = 0;
    virtual double  LookAt(RelPose &pose) = 0;
    virtual long GetCurFrame(){return -1;}
  virtual void NextFrame(){}

    virtual bool	Start()				= 0;
    virtual bool	Stop()				= 0;
//#ifdef HALCONIMG
    virtual void ReadCamParam(std::string filename = "");
//#endif

#ifdef HALCONIMG
  virtual Halcon::HWindow* GetWindow();
  virtual void DeleteWindow();
     Halcon::HWindow* m_win;
#endif
    RelPose*		m_relPose;
    virtual void Show(long frame = -1);
    virtual void WriteToFile(std::string fileName, const long& Frame = -1);
    virtual XMLTag* Save() = 0;
protected:
   void PushBack(Image* img);
   bool DeleteImg();
   Image* GetImage_Lock(size_t index);

protected:
    std::vector<Image*> m_images;
    std::vector<Image*> m_temp_images;
    long				m_FrameCount;
    long				m_deletedOffset;
#ifdef BOOST_THREAD
    boost::mutex m_mutexImageList;
#else
#endif
};

#endif /*CAMERA_H*/
