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
                        CombinedShapeDescr.h - Copyright klank


**************************************************************************/


#ifndef COMBINEDSHAPEDESCR_H
#define COMBINEDSHAPEDESCR_H
#include "LocateAlgorithm.h"

#include <string>

#define XML_NODE_COMBINEDSHAPEDESCRALG "CombinedShapeDescrAlg"
#ifdef HALCONIMG
namespace Halcon
{
	class HTuple;
}
#endif
/**
  * class CombinedShapeDescr
  * @brief A 3D-Locate algorithm that combines two approaches for higher robustness
  */
class CombinedShapeDescr : public LocateAlgorithm
{
public:

  // Constructors/Destructors
  //  


  /**
   * Empty Constructor
   */
  CombinedShapeDescr ();
  CombinedShapeDescr (XMLTag* tag);

  /**
   * Empty Destructor
   */
  virtual ~CombinedShapeDescr ( );

  // Methods
  //  
  XMLTag* Save();

  // Public attributes
  //  


  // Public attribute accessor methods
  //  
  std::vector<RelPose*> Perform(std::vector<Camera*> cam, RelPose* pose, Signature& Object, int &numOfObjects, double& qualityMeasure);

	double CheckSignature(Signature& Object);

	std::vector<RelPose*> Inner(Image* img, RelPose* camPose,Calibration* calib,RelPose* lastKnownPose, Signature& object, int &numOfObjects, double& qualityMeasure, bool trackPossible = true);
	std::vector<RelPose*> InnerDesc(Image* img,RelPose* camPose, Calibration* calib, RelPose* lastKnownPose, Signature& object, int &numOfObjects, double& qualityMeasure);

private:

  // Private attributes
  //  
	double m_minScore;
	double m_greediness;
	int	   m_levels;

#ifdef HALCONIMG
	Halcon::HTuple* m_paramNameList;
	Halcon::HTuple* m_paramList;
#endif		
};

#endif // SHAPEBASED3D_H
