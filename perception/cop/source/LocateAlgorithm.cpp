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
                        LocateAlgorithm.cpp - Copyright klank
**************************************************************************/

#include "LocateAlgorithm.h"
#include "XMLTag.h"
#include "ShapeBased3D.h"
#include "ColorBased.h"
#include "DescriptorBased.h"
#include "FindCalTab.h"
#include "BlobLocalizer.h"
#include "CombinedShapeDescr.h"
#include "DeformShapeBased.h"
#include "SupportingPlaneDetector.h"
#include "ClusterDetector.h"
#include "SimulatedLocate.h"
#include "CheckColorClass.h"
#include "TwoInOneAlg.h"
// Constructors/Destructors
//
#include <time.h>
#define TRACKING_POSSIBLE_MS 2

LocateAlgorithm::LocateAlgorithm () :
Algorithm<std::vector<RelPose*> >()
{
}

LocateAlgorithm::~LocateAlgorithm ( )
{
}

LocateAlgorithm* LocateAlgorithm::LocAlgFactory(XMLTag* tag)
{
	if(tag->GetName().compare(XML_NODE_SHAPEBASED3DALG) == 0)
		return new ShapeBased3D(tag);
	else if(tag->GetName().compare(XML_NODE_FINDCALTAB) == 0)
    return new FindCalTab(tag);
  else if(tag->GetName().compare(XML_NODE_BLOBLOCALIZER) == 0)
    return new BlobLocalizer(tag);
	else if(tag->GetName().compare(XLM_NODE_COLORBASED) == 0)
		return new ColorBased();
	else if(tag->GetName().compare(XLM_NODE_COLORBASED) == 0)
		return new ColorBased();
	else if(tag->GetName().compare(XML_NODE_SUPPORTINGPLANEDETECTOR) == 0)
		return new SupportingPlaneDetector();
	else if(tag->GetName().compare(XML_NODE_SIMLOCATE) == 0)
		return new SimulatedLocate();
	else if(tag->GetName().compare(XML_NODE_CHECKCOLORCLASS) == 0)
		return new CheckColorClass(tag);
	else if(tag->GetName().compare(XML_NODE_TWOINONEALG) == 0)
		return new TwoInOneAlg(tag);
#ifdef SWISS_RANGER_SERVICE
	else if(tag->GetName().compare(XML_NODE_CLUSTERDETECTOR) == 0)
		return new ClusterDetector(tag);
#endif
#ifdef DESCRIPTOR_AVAILABLE
	else if(tag->GetName().compare(XML_NODE_DESCRIPTORBASEDALG) == 0)
		return new DescriptorBased();
	else if(tag->GetName().compare(XML_NODE_COMBINEDSHAPEDESCRALG) == 0)
		return new CombinedShapeDescr(tag);
#endif
#ifdef DEFORMSHAPE_AVAILABLE
	else if(tag->GetName().compare(XML_NODE_DEFORMSHAPEBASEDALG) == 0)
		return new DeformShapeBased(tag);
#endif
	else
		return NULL;
}


// Accessor methods
//
double LocateAlgorithm::CheckSignature(Signature& object)
{
	if(object.GetElement(0, DESCRIPTOR_COLOR))
		return 1.0;
	return 0.0;
}

bool LocateAlgorithm::TrackingPossible(const Image& img, const Signature& sig, RelPose* pose)
{
	unsigned long l = (unsigned long)time(NULL);
	l -= sig.date();
#ifdef _DEBUG
	printf("TrackingPossible? Last Match diff: %ld\n", l);
#endif
	if(l > 0 && l < TRACKING_POSSIBLE_MS && pose != NULL)
	{
		Matrix m = sig.m_relPose->GetCovarianceMatrix();
		//TODO Check Timestamp
#ifdef _DEBUG
		printf("Check 3 Cov Values of %ld: %f < 0.1; %f < 0.1;%f < 0.1;\n", sig.m_relPose->m_uniqueID, m.data()[0],m.data()[7], m.data()[14]);
#endif
		return m.data()[0] < 0.1 && m.data()[7] < 0.1 && m.data()[14] < 0.1;
	}
	return false;

}


// Other methods
//


