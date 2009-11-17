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

 
#include "ProveAlgorithm.h"
#include "XMLTag.h"
#include "PRAShapeVsPointDescr.h"
ProveAlgorithm::ProveAlgorithm(void)
{
}

ProveAlgorithm::~ProveAlgorithm(void)
{
}

ProveAlgorithm* ProveAlgorithm::ProveAlgFactory(XMLTag* tag)
{
	std::string name = tag->GetName();
	if(name.compare(XML_NODE_PRASP) == 0)
	{
		return new PRAShapeVsPointDescr(tag);
	}
	return NULL;
}

