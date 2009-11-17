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

 
/*****************************************************************
                        Image.cpp - Copyright klank

**************************************************************************/

#include "Image.h"
#ifdef HALCONIMG
#include "cpp/HalconCpp.h"
#endif
#include "XMLTag.h"
#include <sstream>

#ifdef OPENCV_USED
#include "himage2iplimage.h"
#endif /*OPENCV_USED*/


// Constructors/Destructors
//

Image::Image (int type)  :
#ifdef HALCONIMG
 m_image(NULL),
#endif
  m_type(type),
  m_timestamp((unsigned long)time(NULL))

{
}

Image::Image ( const Image& img) :
#ifdef HALCONIMG
    m_image(img.m_image),
#endif
  m_type(img.m_type),
  m_usageCount(0),
  m_timestamp((unsigned long)time(NULL))
{
};

#ifdef HALCONIMG
Image::Image ( Halcon::Hobject* img, int type) :
    m_image(img),
    m_type(type),
    m_usageCount(0),
    m_timestamp((unsigned long)time(NULL))
{
}
#endif

Image::~Image ( )
{
#ifdef HALCONIMG
    if(m_usageCount > 0)
        throw "deleting error";
    if(m_image!= NULL)
        delete m_image;
#endif
}

Image::Image (XMLTag* tag)	:
  m_type(GRAY_IMAGE),
  m_usageCount(0)
{
#ifdef HALCONIMG
    std::string stFileName = tag->GetProperty(XML_ATTRIBUTE_FILENAME);

    if(stFileName.length() == 0)
        throw "No Image-Filename Specified in xml tag";
    m_image = new Halcon::Hobject();
    try
    {
        Halcon::read_image(m_image, stFileName.c_str());
    }
    catch(...)
    {
        delete m_image;
        m_image = NULL;
        throw "File not found: read image from file";
    }
    Halcon::HTuple num;
    Halcon::count_obj(*m_image, &num);
    m_type = tag->GetPropertyInt(XML_ATTRIBUTE_IMGTYPE, num == 1 ? GRAY_IMAGE : RGB_IMAGE);

#endif
}


//
// Methods
//
#ifdef HALCONIMG
Halcon::Hobject* Image::GetHImage()
{
    //Halcon::HTuple a,b,c,d;
    //Halcon::get_image_pointer1(*m_image, &a,&b,&c,&d);
    return m_image;
}
#ifdef OPENCV_USED
IplImage* Image::GetIplImage()
{
    Hlong img[3], *img2[3], width, height;
    IplImage* ret = NULL;
    Hlong channels = 0;
    char type[20];
    Halcon::count_channels(*m_image, &channels);
    Herror err;
    if(channels == 3)
        err = Halcon::get_image_pointer3(*m_image, &img[0], &img[1], &img[2],type ,&width, &height);
    else
        err = Halcon::get_image_pointer1(*m_image, &img[0],type ,&width, &height);
    if(err == 2)
    {
        img2[0] = (Hlong*)img[0];
        img2[1] = (Hlong*)img[1];
        img2[2] = (Hlong*)img[2];
        ret = HImage2IplImage(img2 , channels, width, height,1 );
    }
    return ret;
}
#endif

#endif


XMLTag* Image::Save()
{
    XMLTag* tag = new XMLTag(XML_NODE_IMAGEFILE);
    std::ostringstream os;
    os << "img" << tag->date() << ".png";
    try
    {
#ifdef HALCONIMG
        Halcon::write_image(*m_image, "png", 0, os.str().c_str());
#endif
        tag->AddProperty(XML_ATTRIBUTE_FILENAME, os.str());
        tag->AddProperty(XML_ATTRIBUTE_IMGTYPE, m_type);
    }
    catch(...)
    {
        printf("Image File not found!!\n");
    }
    return tag;
}

void Image::Delete(XMLTag* tag)
{

}
Image* Image::Clone()
{
#ifdef HALCONIMG
    Halcon::Hobject* obj = new Halcon::Hobject();
    Halcon::copy_image(*m_image, obj);
    Image* img = new Image(obj, m_type);
    return img;
#else
    return NULL;
#endif
}



