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


#ifndef DEFORMSHAPEMODEL_H
#define DEFORMSHAPEMODEL_H
/*#ifdef DEFORMSHAPE_AVAILABLE*/

#include "Descriptor.h"

#define XML_NODE_DEFORMSHAPEMODEL "DeformShapeModel"

namespace Halcon
{
	class HTuple;
	class Hobject;
}

namespace cop
{
  class Signature;
  class Camera;
  class Image;
  /********************************************************************
  *   class DeformShapeModel                                          */
  /********************************************************************
  *   @brief The model that enables using DeformShapeBased
  *
  *********************************************************************/
  class DeformShapeModel :
    public Descriptor
  {

  public:
    DeformShapeModel(Class* classref);

    DeformShapeModel();

    ~DeformShapeModel(void);

    virtual ElemType_t GetType()const{return DESCRIPTOR_DEFORMSHAPE;}
    virtual std::string GetNodeName() const {return XML_NODE_DEFORMSHAPEMODEL;}


    long GetDeformShapeHandle(){return m_handle;}
    /*First match, needed for tracking*/
    double DefineDeformShapeModel(Image* img, Halcon::Hobject* region, Camera* cam, RelPose* pose);


    /***********************************************************************
    * ShapeModel::Show                                                     */
    /************************************************************************
    * @brief if this Descriptor can be showed, show it.
    * @param pose A position where this descriptor should be displayed,
    * @param camera that took the picture where the descriptor was displayed
    *************************************************************************/
    virtual void Show(RelPose* pose, Sensor* cam);
  protected:
    bool LoadFromFile(std::string fileName);

    virtual void SetData(XMLTag* tag);
    virtual void SaveTo(XMLTag* tag);


  private:
    long m_handle;
    std::string m_filename;
    bool m_bWritten;
    Halcon::Hobject* m_regionTemp;
    bool m_autoLearned;
  };
}
/*#endif *//*DEFORMSHAPE_AVAILABLE*/
#endif
