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
                        Image.h - Copyright klank

**************************************************************************/


#ifndef IMAGE_H
#define IMAGE_H

#include <string>
#ifdef OPENCV_USED
#include "cv.h"
#include "highgui.h"
#endif /*OPENCV_USED*/

#define NO_TYPE 0
#ifdef HALCONIMG
#define HALCONIMAGE 1
#endif
/**
  * class Image
  */

#define GRAY_IMAGE 0
#define RGB_IMAGE 1
#define YUV_IMAGE 2
#define HSV_IMAGE 4
#define GRAY_DISPARITY_IMAGE 8
#define YUV_DISPARITY_IMAGE 16
#define NO_IMAGE 32

#define XML_NODE_IMAGEFILE "ImageFile"
#define XML_ATTRIBUTE_FILENAME "FileName"
#define XML_ATTRIBUTE_IMGTYPE "ImgType"

#ifdef HALCONIMG
namespace Halcon
{
    class Hobject;
}
#endif

class XMLTag;
/******************************************************************
*           class Image                                           */
/** ****************************************************************
*   @brief Abstraction of an images, can be derived for different 
            images or acquisition devices
*   Provides Halcon Image and IplImage
*******************************************************************/
class Image
{
public:

  // Constructors/Destructors
  //  


  /**
   * Empty Constructor
   */
  Image (int type);
  Image ( const Image& img);

    
    /**
    * Constructor Image
    *   @param tag contains a file name or similar information
    *   @throws char* with an error message in case of failure
    */
    Image ( XMLTag* tag);
    
#ifdef HALCONIMG
    Image ( Halcon::Hobject* img,int type);
#endif
  /**
   * Empty Destructor
   *   @throws char* with an error message in case of failure
   */
  virtual ~Image ( );
  XMLTag* Save();
  void Delete(XMLTag* tag) ;

  Image* Clone();
  
  int GetColorSpace(){return m_type;}
  void Hold()
  {
      m_usageCount++;
  }
  void Free(){
      m_usageCount--;
  }
#ifdef HALCONIMG
     int GetType() const{return HALCONIMAGE;}
     Halcon::Hobject* GetHImage();
     Halcon::Hobject* ZoomImage(int width, int height);
#ifdef OPENCV_USED
     IplImage* GetIplImage();
#endif /*OPENCV_USED*/
#else
      int GetType() const{return NO_TYPE;}
#endif

virtual unsigned long date() const
{
    return m_timestamp;
}
protected:


private:

  // Static Private attributes
  //  
#ifdef HALCONIMG
    Halcon::Hobject* m_image;
#endif
public:
    int m_type;
    int m_usageCount;
    unsigned long m_timestamp;
  // Private attributes
  //  


  // Private attribute accessor methods
  //  


  // Private attribute accessor methods
  //  



};

#endif // IMAGE_H
