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


#ifndef RFADEPTHREFINEMENT_H
#define RFADEPTHREFINEMENT_H
#include "RefineAlgorithm.h"

#ifndef USE_YARP_COMM
#include <robot_msgs/PointCloud.h>
#endif
class RFADepthRefinement :
  public RefineAlgorithm
{
public:
  RFADepthRefinement(void);
  RFADepthRefinement(XMLTag* tag);


  virtual ~RFADepthRefinement(void);

  virtual Descriptor* Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& sig, int &numOfObjects, double& qualityMeasure);

  double CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors);

#ifndef USE_YARP_COMM
  //void PointCloudCallback(const robot_msgs::PointCloud &pc);
#endif


  virtual XMLTag* Save();

  virtual std::string GetName(){return "RFADepthRefinement";}

};

#endif /*RFADEPTHREFINEMENT_H*/

