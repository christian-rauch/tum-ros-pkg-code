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

 
#ifndef CLUSTERDETECTOR_H
#define CLUSTERDETECTOR_H

#ifdef SWISS_RANGER_SERVICE

#define XML_NODE_CLUSTERDETECTOR "ClusterDetector"
#include "LocateAlgorithm.h"
class PlaneClusterResult;

class ClusterDetector : public LocateAlgorithm
{
public:
  ClusterDetector(int swiss, int ptu_base);
  ClusterDetector(XMLTag* tag);

  ~ClusterDetector();

  std::vector<RelPose*> Perform(std::vector<Camera*> cam, RelPose* pose, Signature& object, int &numOfObjects, double& qualityMeasure);

  std::vector<RelPose*> Inner(int &numOfObjects, double& qualityMeasure);

  double CheckSignature(Signature& object);

  bool TrackingPossible(const Image& img, const Signature& sig, RelPose* pose);

  XMLTag* Save();

  virtual std::string GetName(){return XML_NODE_CLUSTERDETECTOR;}

  static bool CallStaticPlaneClusterExtractor(PlaneClusterResult* response, int m_swissranger_jlo_id, int m_ptu_jlo_id);

private:
  int m_swissranger_jlo_id;
  int m_ptu_jlo_id;
};

#endif

#endif /*CLUSTERDETECTOR_H*/