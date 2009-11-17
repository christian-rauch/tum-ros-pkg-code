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
                        LocateAlgorithm.h - Copyright klank


**************************************************************************/


#ifndef LOCATEALGORITHM_H
#define LOCATEALGORITHM_H
#include "Algorithm.h"

#include <string>
#include <vector>



/**
  * class LocateAlgorithm
  * @brief Specialisation of algoithms that return a 3d position as an result
  */
class LocateAlgorithm : public Algorithm<std::vector<RelPose*> >
{
public:

  // Constructors/Destructors
  //  


  /**
   * Empty Constructor
   */
  LocateAlgorithm ();

  /**
   * Empty Destructor
   */
  virtual ~LocateAlgorithm ( );

  // Static Public attributes
  //  

  // Public attributes
  //  
	static LocateAlgorithm* LocAlgFactory(XMLTag* tag);

  // Public attribute accessor methods
  //  
  virtual std::vector<RelPose*> Perform(std::vector<Camera*> cam, RelPose* pose, Signature& Object, int &numOfObjects, double& qualityMeasure) = 0;//{throw "Perform for LocateAlgorithm not implemented\n";}

	virtual double CheckSignature(Signature& Object);

	virtual bool TrackingPossible(const Image& img, const Signature& sig, RelPose* pose);
	virtual XMLTag* Save() = 0;//{throw "Save for LocateAlgorithm not implemented\n";}
};

#endif // LOCATEALGORITHM_H
