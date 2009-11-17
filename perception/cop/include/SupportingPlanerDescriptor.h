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

 
#ifndef SUPPORTINGPLANERDESCRIPTOR_H
#define SUPPORTINGPLANERDESCRIPTOR_H
#include "Descriptor.h"

#define XML_NODE_PLANE_DESCR "SupportingPlanerDescriptor"

class SupportingPlanerDescriptor :
    public Descriptor
{
public:
    SupportingPlanerDescriptor(Class* cl);
    SupportingPlanerDescriptor(XMLTag* node);
    virtual ~SupportingPlanerDescriptor(void);

	virtual std::string GetNodeName() const {return XML_NODE_PLANE_DESCR;}

	virtual int GetType() const {return DESCRIPTOR_PLANE;}

    virtual void Show(Camera* );
    Elem*& GetMarker(){return m_marker;}
    double& GetExtX(){return m_extX;}
    double& GetExtY(){return m_extY;}

protected:
    virtual void SaveTo(XMLTag* );

private:
    double m_extX;
    double m_extY;
    Elem* m_marker;
};
#endif /*SUPPORTINGPLANERDESCRIPTOR_H*/
