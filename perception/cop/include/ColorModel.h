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

 
#ifndef COLORMODEL_H
#define COLORMODEL_H

#include "Descriptor.h"

#define XML_NODE_COLORMODEL "ColorModel"

class Signature;
/**
*   class ColorModel
*   @brief Signature that contains an color model for a simple ColorBased detection
*/
class ColorModel :
	public Descriptor
{
public:
	ColorModel(Class* classref, Signature* sig);
	ColorModel(XMLTag* tag);
	~ColorModel(void);

	virtual int GetType()const{return DESCRIPTOR_COLOR;}
	virtual std::string GetNodeName() const {return XML_NODE_COLORMODEL;}

	virtual double Compare(std::vector<double>);
	
	void SetSize(int size){m_patchSize = size;}
	int GetSize(){return m_patchSize;}
	inline void AddColorSpec(double histo){m_colorSpec.push_back(histo);}
	void SaveTo(XMLTag* tag);

	std::vector<double> m_colorSpec;
	int m_patchSize;
};

#endif /*COLORMODEL_H*/
