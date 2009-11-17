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

 
#ifdef DEFORMSHAPE_AVAILABLE

#include "DeformShapeBased.h"
#include "DeformShapeModel.h"
#include "XMLTag.h"
#ifdef HALCONIMG
#include "cpp/HalconCpp.h"
using namespace Halcon;
#endif

DeformShapeBased::DeformShapeBased()
{

}


DeformShapeBased::DeformShapeBased(XMLTag* tag)
{

}

DeformShapeBased::~DeformShapeBased(void)
{

}


/**
  Action function, prepares images
*/
std::vector<RelPose*> DeformShapeBased::Perform(std::vector<Camera*> cam, RelPose* lastKnownPose, Signature& object, int &numOfObjects, double& qualityMeasure)
{
  std::vector<RelPose*> result;
  if(cam.size() > 0)
  {
    Image* img = cam[0]->GetImage(-1);
    RelPose* camPose = cam[0]->m_relPose;
    Calibration* calib = &(cam[0]->m_calibration);
    if(img != NULL && camPose != NULL && calib  != NULL)
    {
      result = Inner(img, camPose, calib, lastKnownPose, object, numOfObjects, qualityMeasure);
    }
  }
  return result;
}
/**
  Final algorithm, for call for special images
*/
std::vector<RelPose*> DeformShapeBased::Inner(Image* img,RelPose* camPose, Calibration* calib, RelPose* lastKnownPose, Signature& object, int &numOfObjects, double& qualityMeasure)
{
  std::vector<RelPose*> result;
#ifdef HALCONIMG
  HTuple camparam = calib->CamParam();
  HTuple cammatrix = calib->CamMatrix();

  if(img->GetType() == HALCONIMAGE)
  {
    bool trackPossible = TrackingPossible(*img, object, lastKnownPose);
    //int n = 4;
    //double Partly = 1.0 - (0.3* (n-1));
    HTuple empty;
    HTuple  rotationAngleStart, rotationAngleExtend,
        scaleCMin, scaleCMax, scaleRMin, scaleRMax, minScore,
        numMatches, maxOverlap, numLevels, greediness, paramName, paramValue;
    HTuple  score,hommat,hommatfirst,pose, cov, error;

    printf("Get Descriptor Model\n");
    DeformShapeModel* dsm = (DeformShapeModel*)(object.GetElement(0, DESCRIPTOR_DEFORMSHAPE ));
    if(dsm != NULL)
    {
      //hommatfirst = *dsm->GetHomMatFirst();
      try
      {
        Hobject* imgs = img->GetHImage();
        int handle = dsm->GetDeformShapeHandle();
        numMatches = numOfObjects;
        HTuple pose, cov;
        if(!trackPossible || true)
        {
          rotationAngleStart = -1.5;
          rotationAngleExtend = 1.5;
          minScore = 0.6;
          maxOverlap = 1.0;
          numLevels = 0;
          greediness = 0.8;
          scaleCMin = 0.9;
          scaleCMax = 1.1;
          scaleRMin = 0.7;
          scaleRMax = 1.3;
          paramName = "subpixel";
          paramValue = "least_squares_very_high";
          find_planar_calib_deformable_model(*imgs, handle,rotationAngleStart, rotationAngleExtend,
            scaleCMin, scaleCMax, scaleRMin, scaleRMax, minScore,
            numMatches, maxOverlap, numLevels, greediness, paramName, paramValue,
            &pose, &cov, &score);
        }
        else
        {
          printf("Reduced search possible\n");
          rotationAngleStart = -0.9;
          rotationAngleExtend = 0.9;
          minScore = 0.7;
          maxOverlap = 1.0;
          numLevels = 0;
          greediness = 0.9;
          scaleCMin = 0.9;
          scaleCMax = 1.1;
          scaleRMin = 0.7;
          scaleRMax = 1.3;
          paramName = "subpixel";
          paramValue = "least_squares_very_high";

          find_planar_calib_deformable_model(*imgs, handle,rotationAngleStart, rotationAngleExtend,
            scaleCMin, scaleCMax, scaleRMin, scaleRMax, minScore,
            numMatches, maxOverlap, numLevels, greediness, paramName, paramValue,
            &pose, &cov, &score);
            printf("here\n");
        }
        printf("Finished Search (%ld results)\n", score.Num());
        if(score.Num() > 0)
        {
          printf("Estimated Pose:\n");
          int i;
          for(i = 0; i < 6; i++)
          {
            printf("%f, ", pose[i].D());
          }
          printf("%d\n", pose[6].I());
          printf("\n");
          HTuple pose_sel, cov_sel;
          for(i = 0; i < score.Num(); i++)
          {
            tuple_select_range(pose, 0 + 7*i, 6 + 7*i, &pose_sel);
            tuple_select_range(cov, 0+ 6*i, 5+ 6*i, &cov_sel);
            RelPose* pose = RelPoseFactory::FRelPose(pose_sel, cov_sel, camPose);
            pose->m_qualityMeasure = score[i];
            result.push_back(pose);
          }
          numOfObjects = score.Num();
          qualityMeasure = score[0].D();
          dsm->SetLastMatchedImage(img, result[0]);
          img->Free();
        }
        else
        {
          img->Free();
          numOfObjects = 0;
          qualityMeasure = 0.0;
        }
        return result;
      }
      catch(Halcon::HException ex)
      {
        printf("Error in DeformShapeBased: %s\n", ex.message);
        qualityMeasure = 0.0;
      }
    }
  }
  img->Free();
#endif  /*HALCONIMG*/
  return result;
}

/**
  Test if the already there are models learned
*/
double DeformShapeBased::CheckSignature(Signature& object)
{
  if(object.GetElement(0,DESCRIPTOR_DEFORMSHAPE))
    return 1.0;
  else
    return 0.0;
}

/**
  Display the results.
*/
void DeformShapeBased::Show(Camera* cam)
{
#ifdef HALCONIMG
  /*try
  {
   Halcon::HWindow* win =  cam->GetWindow();
  }
  catch(Halcon::HException ex)
  {
    printf("Showing not possible: %s\n", ex.message);
  }*/
#endif

}


/**
  Save the parameters or any
*/
XMLTag* DeformShapeBased::Save()
{
  XMLTag* tag = new XMLTag(XML_NODE_DEFORMSHAPEBASEDALG);
  return tag;
}
#endif /*DEFORMSHAPE_AVAILABLE*/
