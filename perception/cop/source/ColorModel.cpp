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


#include "ColorModel.h"
#include "XMLTag.h"
#include "ShapeModel.h"

#ifdef HALCONIMG
#include <cpp/HalconCpp.h>
#endif

#define XML_NODE_PATCHSIZE "PatchSize"
#define XML_NODE_COLORSPEC "ColorSpec"


using namespace cop;

ColorModel::ColorModel(XMLTag* tag)
	: Descriptor(tag)
{
	if(tag != NULL)
	{
		XMLTag* patchSize = tag->GetChild(XML_NODE_PATCHSIZE);
		if(patchSize != NULL)
			m_patchSize = tag->Load(patchSize,&m_patchSize) ;
		XMLTag* colorspec = tag->GetChild(XML_NODE_COLORSPEC);
		if(colorspec != NULL)
			m_colorSpec = tag->Load(colorspec,&m_colorSpec) ;
	}
}

ColorModel::ColorModel(Class* classref, Signature* sig)
: Descriptor(classref)
{
	ShapeModel* sm = (ShapeModel*)sig->GetElement(0, DESCRIPTOR_SHAPE);
	Image* img = (Image*)sm->GetLastMatchedImage();
#ifdef HALCONIMG
	try
	{
		Halcon::Hobject xld = sm->GetContour(*sig->m_relPose);
		int num = 0;
		Halcon::count_obj(xld, (Hlong*)&num);
		printf("Point Num Xld: %d\n", num);
		Halcon::Hobject region;
		Halcon::gen_empty_region(&region);
		for(int i = 0; i < num; i++)
		{
			Halcon::Hobject obj;
			Halcon::gen_region_contour_xld(xld, &obj ,"filled");
			Halcon::union2(obj, region, &region);
		}
		Halcon::connection(region, &region);
		Halcon::fill_up(region, &region);
		Halcon::Hobject reducedimg;
		Halcon::union1(region, &region);
		Halcon::HTuple area, r,c;
		Halcon::area_center(region, &area, &r, &c);
		SetSize(area[0].L());
		Halcon::reduce_domain(*img->GetHImage(), region, &reducedimg);
		Halcon::count_channels(reducedimg, (Hlong*)&num);
		for(int i = 0; i < num; i++)
		{
			Halcon::HTuple abshisto;
			Halcon::HTuple relhisto;
			Halcon::Hobject obj;
			Halcon::access_channel(reducedimg, &obj, i + 1);
			Halcon::gray_histo(region, obj, &abshisto, &relhisto);
			for(int j = 0; j < relhisto.Num(); j++)
			{
				AddColorSpec(relhisto[j].D());
			}
		}
	}
	catch(Halcon::HException ex)
	{
		printf("Error while Learning: %s\n", ex.message);
	}
#endif
}

ColorModel::~ColorModel(void)
{
}

void ColorModel::SaveTo(XMLTag* tag)
{
	tag->AddChild(XMLTag::Tag(m_patchSize, XML_NODE_PATCHSIZE));
	tag->AddChild(XMLTag::Tag(m_colorSpec, XML_NODE_COLORSPEC));
}

double ColorModel::Compare(std::vector<double> vec)
{
	size_t length = m_colorSpec.size() > vec.size() ? vec.size() : m_colorSpec.size();
	double d = 0.0;
	for(unsigned int i = 0; i < length; i++)
	{
		d+= (vec[i] - m_colorSpec[i]) * (vec[i] - m_colorSpec[i]);
	}
	return d;
}
