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

 
#ifndef CHECKCOLORCLASS_H
#define CHECKCOLORCLASS_H

#include "LocateAlgorithm.h"
#ifdef HALCONIMG
namespace Halcon
{
  class Hobject;
}
#endif

#define XML_NODE_CHECKCOLORCLASS "CheckColorClass"
class CheckColorClass :  public LocateAlgorithm
{
public:
  CheckColorClass(std::string path);
  CheckColorClass(XMLTag* tag);
  ~CheckColorClass(void);

  XMLTag* Save();
  // Public attribute accessor methods
  //
  std::vector<RelPose*> Perform(std::vector<Camera*> cam, RelPose* pose, Signature& Object, int &numOfObjects, double& qualityMeasure);
#ifdef HALCONIMG
  void Inner(Halcon::Hobject* img, Halcon::Hobject* region, std::string &color, double& qualityMeasure);
#endif

  double CheckSignature(Signature& Object);

  virtual std::string GetName(){return XML_NODE_CHECKCOLORCLASS;}
private:
  std::string m_stPath;
  int m_MLPHandle;
};

#endif /*FINDCALTAB_H*/
