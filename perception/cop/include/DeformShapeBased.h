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

 
#ifndef DEFORMSHAPEBASED_H
#define DEFORMSHAPEBASED_H
#ifdef DEFORMSHAPE_AVAILABLE

#include "LocateAlgorithm.h"
#include "Image.h"


#define XML_NODE_DEFORMSHAPEBASEDALG "DeformShapeBasedAlg"

/********************************************************************
*   class DeformShapeBased                                          */
/********************************************************************
*   @brief a planar locating algorithm
*
*********************************************************************/
class DeformShapeBased :
	public LocateAlgorithm
{
public:
	/**
		Constructor
	*/
	DeformShapeBased(void);
	DeformShapeBased(XMLTag* tag);
	/**
		Destructor
	*/
	~DeformShapeBased(void);
	/**
		Action function, prepares images
	*/
	std::vector<RelPose*> Perform(std::vector<Camera*> cam, RelPose* pose, Signature& Object, int &numOfObjects, double& qualityMeasure);
	/**
		Final algorithm, for call for special images
	*/
	std::vector<RelPose*> Inner(Image* img,RelPose* camPose, Calibration* calib, RelPose* lastKnownPose, Signature& object, int &numOfObjects, double& qualityMeasure);
	/**
		Test if the already there are models learned
	*/
	double CheckSignature(Signature& Object);
	/**
		Display the results.
	*/
	void Show(Camera* cam);

	/**
		Save the parameters or any 
	*/
	XMLTag* Save();

};
#endif /*DEFORMSHAPE_AVAILABLE*/
#endif /*DEFORMSHAPEBASED_H*/
