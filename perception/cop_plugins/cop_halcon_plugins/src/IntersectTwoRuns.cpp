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


#include "IntersectTwoRuns.h"
#include "XMLTag.h"
#include "cpp/HalconCpp.h"
#include "Camera.h"
#include "RelPoseHTuple.h"

using namespace cop;


IntersectTwoRuns::IntersectTwoRuns(LocateAlgorithm* first, LocateAlgorithm* second) :
  m_firstAlg(first),
  m_index_sens1(0),
  m_index_sens2(1)
{
}

IntersectTwoRuns::IntersectTwoRuns()
{
}

void IntersectTwoRuns::SetData(XMLTag* tag)
{
  printf("Loading Algorithm IntersectTwoRuns\n");
  if(tag != NULL && tag->CountChildren() > 0)
  {
    m_firstAlg = LocateAlgorithm::LocAlgFactory(tag->GetChild(0));
    if(tag->CountChildren() > 1)
    {
      m_secondAlg = LocateAlgorithm::LocAlgFactory(tag->GetChild(1));
    }
    else
     m_secondAlg = m_firstAlg;
  }
  else
     throw "Error loading IntersectTwoRuns";
}


IntersectTwoRuns::~IntersectTwoRuns(void)
{
  delete m_firstAlg;
}


XMLTag* IntersectTwoRuns::Save()
{
  XMLTag* tag = new XMLTag(XML_NODE_INTERSECTTWORUNS);
  tag->AddChild(m_firstAlg->Save());
  if(m_firstAlg != m_secondAlg)
    tag->AddChild(m_secondAlg->Save());
  return tag;
}
// Public attribute accessor methods
//
std::vector<RelPose*> IntersectTwoRuns::Perform(std::vector<Sensor*> cam, RelPose* pose, Signature& object, int &numOfObjects, double& qualityMeasure)
{
  int numOfObjects_first = numOfObjects;
  double qualityMeasure_first = qualityMeasure;
  std::vector<RelPose*> results;
  std::vector<Sensor*> sens_set_1;
  std::vector<Sensor*> sens_set_2;
  if(cam.size() < m_index_sens1 || cam.size() < m_index_sens2 )
  {
    printf("Different behaviour than in CheckSignature?!?");
    throw "Error in IntersectTwoRuns: Got less sensors than necessary!";
  }
  sens_set_1.push_back(cam[m_index_sens1]);
  sens_set_2.push_back(cam[m_index_sens2]);

  std::vector<RelPose*> result_first = m_firstAlg->Perform(sens_set_1, pose, object, numOfObjects_first, qualityMeasure_first);
  numOfObjects = 0;
  if(numOfObjects_first > 0)
  {
    int numOfObjects_second = 1;
    double qualityMeasure_second = qualityMeasure;
    for(unsigned int i = 0; i < result_first.size(); i++)
    {
      std::vector<RelPose*> result_second = m_secondAlg->Perform(sens_set_2, result_first[i], object, numOfObjects_second, qualityMeasure_second);
      if(numOfObjects_second > 0)
      {
        Halcon::HTuple pose1, pose2, row1, column1,pose_rel_sens_tup,
                      row2, column2, cam1 =  ((Camera*)sens_set_1[0])->m_calibration.CamParam(),
                       cam2 =  ((Camera*)sens_set_1[0])->m_calibration.CamParam(),X,Y,Z, Dist;
        RelPoseHTuple::GetPose(result_first[i], &pose1);
        RelPoseHTuple::GetPose(result_second[0], &pose2);

        RelPose* pose_rel_sens = RelPoseFactory::GetRelPose(result_second[0]->m_parentID, result_first[i]->m_parentID);

        RelPoseHTuple::GetPose(pose_rel_sens, &pose_rel_sens_tup);
        RelPoseFactory::FreeRelPose(pose_rel_sens);
        try
        {
          Halcon::project_3d_point(pose1[0], pose1[1], pose1[2], cam1, &row1, &column1);
          Halcon::project_3d_point(pose2[0], pose2[1], pose2[2], cam2, &row2, &column2);
          Halcon::intersect_lines_of_sight(cam1, cam2, pose_rel_sens_tup, row1, column1, row2, column2, &X,&Y,&Z, &Dist);
        }
        catch(Halcon::HException ex)
        {
           printf("Error in IntersectTwoRuns: %s\n", ex.message);
        }
        if(result_first[i]->m_qualityMeasure > qualityMeasure_second)
        {
          Matrix m = result_first[0]->GetMatrix(0);
          Matrix cov = result_first[0]->GetCovarianceMatrix();
          m.element(0,3) = X[0].D();
          m.element(1,3) = Y[0].D();
          m.element(2,3) = Z[0].D();
          RelPose* final = RelPoseFactory::FRelPose(result_first[0]->m_parentID, m, cov);
          qualityMeasure = final->m_qualityMeasure =  result_first[i]->m_qualityMeasure;
          numOfObjects += 1;
          results.push_back(final);
        }
        else
        {
          Matrix m = result_second[0]->GetMatrix(0);
          Matrix cov = result_second[0]->GetCovarianceMatrix();

          m.element(0,3) = X[0].D();
          m.element(1,3) = Y[0].D();
          m.element(2,3) = Z[0].D();
          RelPose* final = RelPoseFactory::FRelPose(result_second[0]->m_parentID, m, cov);
          qualityMeasure = final->m_qualityMeasure =  qualityMeasure_second;
          numOfObjects += 1;
          results.push_back(final);
        }

        RelPoseFactory::FreeRelPose(result_second[0]);

        printf("Improved Pose by intersection:\n");
        RelPoseHTuple::Print(results[0]);
      }
      RelPoseFactory::FreeRelPose(result_first[i]);
    }
  }
  return results;
}

double IntersectTwoRuns::CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors)
{
  printf("IntersectTwoRuns::CheckSignature\n");
  double val1 = 0.0, val2 = 0.0;
  int score = 0;
  if(sensors.size() > 0)
  {
    for(size_t i = 0; i< sensors.size(); i++)
    {
      std::vector<Sensor*> sens_set_1;
      sens_set_1.push_back(sensors[i]);
      if(val1 == 0.0)
      {
       val1 = m_firstAlg->CheckSignature(object, sens_set_1);
       m_index_sens1 = i;
       score++;
      }
      else
      {
       val2 = m_secondAlg->CheckSignature(object, sens_set_1);
       m_index_sens2 = i;
       score++;
       break;
      }
    }
  }
  if(score < 2)
    return 0.0;
  return  val1 + val2;
}
