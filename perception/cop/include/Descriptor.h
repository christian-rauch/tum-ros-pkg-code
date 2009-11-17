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
                        Descriptor.h - Copyright klank

**************************************************************************/


#ifndef DESCRIPTOR_H
#define DESCRIPTOR_H

#include "Class.h"

class RelPose;
class Image;

#define XML_NODE_DESCRIPTOR "Descriptor"
#define XML_ATTRIBUTE_MODELID "ModelID"

namespace Halcon
{
	class Hobject;
}


/**
  * class Descriptor
  *  @brief Represents visual models in the SignatureDB, part of the Elem hirarchy
  */
class Descriptor : public Elem
{
public:

	// Constructors/Destructors
	//  
	/**
	* Empty Constructor
	*/
	Descriptor ( Class* classref);
	Descriptor ( XMLTag* tag);

	/**
	* Empty Destructor
	*/
	virtual ~Descriptor ( );

  /***********************************************************************
    *   SetLastMatchedImage                                             */
  /************************************************************************
  *  @brief sets the last iamge on that this model was matched
  *  @param img  an image 
  *  @param pose a pose that specifies the result in the given image 
  *  of a locating algorithm
  *  @throws char* with an error message in case of failure
	**************************************************************************/
	void SetLastMatchedImage(Image* img, RelPose* pose);
	/**
	*/
  /***********************************************************************
  * ShapeModel::GetLastMatchedImage                                      */
  /************************************************************************
	*	@returns the last iamge on that this model was matched
  *************************************************************************/
	Image* GetLastMatchedImage(){return m_imgLastMatchImage;}
  /***********************************************************************
  * ShapeModel::GetLastMatchedPose                                                     */
  /************************************************************************
  * @brief The descrtiptor remebers the last matched pose/image combination for refinement.
  *************************************************************************/
	RelPose* GetLastMatchedPose(){return m_poseLastMatchImage;}
  /***********************************************************************
  * ShapeModel::GetQuality                                                     */
  /************************************************************************
  * @returns the quality of the descriptor                                *
  *************************************************************************/
	double GetQuality(){return m_qualityMeasure;}
  /***********************************************************************
  * ShapeModel::Evaluate                                                     */
  /************************************************************************
  * @brief Puts a result to a descriptor to set its quality.
  * @param eval a value from 0.0 (bad) to 1.0 (good)
  *************************************************************************/
	void Evaluate(double eval){m_qualityMeasure = eval  + m_qualityMeasure / 2; } //TOCHECK: how to combine and TODO if(m_qualityMeasure == 0.0) delete this;

  /***********************************************************************
  * ShapeModel::Show                                                     */
  /************************************************************************
  * @brief if this Descriptor can be showed, show it.
  * @param pose A position where this descriptor should be displayed, 
  * @param camera that took the picture where the descriptor was displayed
  *************************************************************************/
  virtual void Show(RelPose* , Camera* ){};
  /***********************************************************************
  * ShapeModel::GetNodeName                                                     */
  /************************************************************************
  * @brief  The node name for saving and loading to XML
  *************************************************************************/
	virtual std::string GetNodeName() const{return XML_NODE_DESCRIPTOR;}

  /***********************************************************************
  * ShapeModel::SaveTo                                                    */
  /************************************************************************
  * @brief  Adds the descriptors parameter to a XMLTag
  *************************************************************************/
	virtual void SaveTo(XMLTag* tag);
  // Public attribute accessor methods
  //  
	Class* GetClass();
protected:
  // Static Protected attributes
  //  
	Class* m_class;
protected:
	Image* m_imgLastMatchImage;
	RelPose* m_poseLastMatchImage;

	double m_qualityMeasure;
};

#endif // DESCRIPTOR_H

