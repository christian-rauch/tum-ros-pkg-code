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
                        CombinedShapeDescr.cpp - Copyright klank

**************************************************************************/

#include "CombinedShapeDescr.h"
#include "XMLTag.h"
#ifdef HALCONIMG
#include "cpp/HalconCpp.h"
#include "HCPPdescriptor3d.h"

using namespace Halcon;
#endif

#include "ShapeModel.h"
#include "PointDescrModel.h"

#define XML_ATTRIBUTE_MINSCORE "MinScore"
#define XML_ATTRIBUTE_GREEDY   "Greediness"
#define XML_ATTRIBUTE_LEVELS   "Levels"


// Constructors/Destructors
//
CombinedShapeDescr::CombinedShapeDescr () :
	LocateAlgorithm(),
	m_minScore(0.7),
	m_greediness(0.8),
	m_levels(5)
{
#ifdef HALCONIMG
    m_paramList = new HTuple(9, 0.0);
    m_paramNameList = new HTuple(9, "");
    tuple_gen_const(9, 0.0, m_paramList);
    tuple_gen_const(9, "", m_paramNameList);
    (*m_paramNameList)[0] = "longitude_min";
    (*m_paramNameList)[1] = "longitude_max";
    (*m_paramNameList)[2] = "latitude_min";
    (*m_paramNameList)[3] = "latitude_max";
    (*m_paramNameList)[4] = "cam_roll_min";
    (*m_paramNameList)[5] = "cam_roll_max";
    (*m_paramNameList)[6] = "dist_min";
    (*m_paramNameList)[7] = "dist_max";
    (*m_paramNameList)[8] = "num_matches";
#endif
}

CombinedShapeDescr::CombinedShapeDescr (XMLTag* tag) :
        LocateAlgorithm(),
        m_minScore(0.95),
        m_greediness(0.9),
        m_levels(6)
{
  if(tag != NULL)
  {
    m_minScore = tag->GetPropertyDouble(XML_ATTRIBUTE_MINSCORE);
    if(m_minScore < 0.3 || m_minScore > 0.99)
    {
      m_minScore = 0.90;
    }
    m_greediness = tag->GetPropertyDouble(XML_ATTRIBUTE_GREEDY);
    if(m_greediness < 0.3 || m_greediness > 0.99)
    {
      m_greediness = 0.90;
    }
    m_levels = tag->GetPropertyInt(XML_ATTRIBUTE_LEVELS);
    if(m_levels < 2 || m_levels > 7)
    {
      m_levels = 6;
    }
  }
#ifdef HALCONIMG
	m_paramList = new HTuple(9, 0.0);
	m_paramNameList = new HTuple(9, "");
	(*m_paramNameList)[0] = "longitude_min";
	(*m_paramNameList)[1] = "longitude_max";
	(*m_paramNameList)[2] = "latitude_min";
	(*m_paramNameList)[3] = "latitude_max";
	(*m_paramNameList)[4] = "cam_roll_min";
	(*m_paramNameList)[5] = "cam_roll_max";
	(*m_paramNameList)[6] = "dist_min";
	(*m_paramNameList)[7] = "dist_max";
	(*m_paramNameList)[8] = "num_matches";
#endif
}


XMLTag* CombinedShapeDescr::Save()
{
	XMLTag* tag = new XMLTag(XML_NODE_COMBINEDSHAPEDESCRALG);
	tag->AddProperty(XML_ATTRIBUTE_MINSCORE, m_minScore);
	tag->AddProperty(XML_ATTRIBUTE_GREEDY, m_greediness);
	tag->AddProperty(XML_ATTRIBUTE_LEVELS, m_levels);
	return tag;
}

void printPose(Halcon::HTuple refpos)
{
	printf("%f %f %f // %f %f %f // %d\n", refpos[0].D(), refpos[1].D(), refpos[2].D(), refpos[3].D(), refpos[4].D(), refpos[5].D(), refpos[6].I());
}

CombinedShapeDescr::~CombinedShapeDescr ( )
{
#ifdef HALCONIMG
	delete m_paramList;
	delete m_paramNameList;
#endif
}
#ifdef HALCONIMG
void GuidedPoseRestictionComb(Halcon::HTuple* paramList,Halcon::HTuple* paramNameList, long model, double fraction, RelPose* pose, Halcon::HTuple camparam, ShapeModelParamSet* pm)
{
	Halcon::HTuple hpose, poseout, hommat3d, hommat3dInv, row, col, poserefabg, longitude, latitude, distance, camroll, refpos;
	pose->GetPose(&hpose);
  Halcon::trans_pose_shape_model_3d(model,hpose, "model_to_ref", &poseout);
	Halcon::pose_to_hom_mat3d(poseout, &hommat3d);
	Halcon::hom_mat3d_invert(hommat3d, &hommat3dInv);
	Halcon::convert_point_3d_cart_to_spher(hommat3dInv[3].D(), hommat3dInv[7].D(),hommat3dInv[11].D(), "-y", "-z", &longitude, &latitude, &distance);
	Halcon::project_3d_point(hommat3d[3].D(), hommat3d[7].D(), hommat3d[11].D(), camparam, &row, &col);
	Halcon::convert_pose_type(poseout, "Rp+T", "abg", "point", &poserefabg);
	camroll = - (((poserefabg[5].D()) / 360) * 2 * M_PI);
  Halcon::get_shape_model_3d_params(model, "reference_pose", &refpos);
	printPose(refpos);
	printPose(hpose);
	printPose(poseout);
	double dlong = pm->m_longitudeMin * fraction;//model->GetShapeModel3dParams((*paramNameList)[0].S())[0].D() * fraction;
  double dlat = pm->m_latitudeMin * fraction;//model->GetShapeModel3dParams((*paramNameList)[2].S())[0].D() * fraction;
	(*paramList)[0] = longitude - dlong;  /*long min*/
	(*paramList)[1] = longitude + dlong;	/*long max*/
	(*paramList)[2] = latitude - dlat;		 /*lat min*/
	(*paramList)[3] = latitude + dlat;		 /*lat max*/
  (*paramList)[4] = pm->m_camRollMin * fraction;//model->GetShapeModel3dParams((*paramNameList)[4].S());		 /*cam roll min*/
  (*paramList)[5] = pm->m_camRollMax * fraction;//model->GetShapeModel3dParams((*paramNameList)[5].S());		 /*cam roll max*/
  (*paramList)[6] = pm->m_distMin* fraction;//model->GetShapeModel3dParams((*paramNameList)[6].S());		 /*dist min*/
  (*paramList)[7] = pm->m_distMax* fraction;//model->GetShapeModel3dParams((*paramNameList)[7].S());		 /*dist max*/
}
#endif
//
// Methods
//
std::vector<RelPose*> CombinedShapeDescr::Perform(std::vector<Sensor*> sensors, RelPose* lastKnownPose, Signature& object, int &numOfObjects, double& qualityMeasure)
{
	std::vector<RelPose*> result;
	if(cam.size() > 0)
	{
		Calibration* calib = &(cam[0]->m_calibration);
		Image* img = cam[0]->GetImage(-1);
		RelPose* camPose = cam[0]->m_relPose;
        if(img != NULL && camPose != NULL)
        {
          std::vector<RelPose*> vec = InnerDesc(img, camPose, calib, lastKnownPose,  object, numOfObjects, qualityMeasure);
          if(vec.size() > 0)
            lastKnownPose = vec[0];
          result = Inner(img, camPose, calib, lastKnownPose,  object, numOfObjects, qualityMeasure, true);
        }
	}
	return result;
}

std::vector<RelPose*> CombinedShapeDescr::Inner(Image* img, RelPose* camPose,Calibration* calib, RelPose* lastKnownPose, Signature& object, int &numOfObjects, double& qualityMeasure, bool trackPossible)
{
	std::vector<RelPose*> result;
#ifdef HALCONIMG
	if(img->GetType() == HALCONIMAGE)
	{
		int n = 3;
		double Partly = 1.0 - (0.3* (n-1));
		HTuple  ViewPose, pose, CovPose, Score;

		/*printf("Get Shape Model\n");*/
		ShapeModel* sm = (ShapeModel*)(object.GetElement(0, DESCRIPTOR_SHAPE ));
    ShapeModelParamSet* pm =  sm->GetParamSet();
    double scale = pm->m_scale;
		/*printf("Start Shape model detection\n");*/
		qualityMeasure = 0.0;
		double minScore = m_minScore;
		int level = m_levels;

		Halcon::HTuple pointer, t, w, h;
		Halcon::Hobject* obj = img->GetHImage();
		Halcon::get_image_pointer1(*obj, &pointer, &t, &w, &h);
		if(scale < 1.0 && scale > 0.001)
		{
			Halcon::Hobject* zoomed = new Halcon::Hobject();
			Halcon::zoom_image_size(*obj, zoomed, (int)(w[0].I()*scale), (int)(h[0].I()*scale), "constant");
			obj = zoomed;
		}
		else
		{
			if(w != calib->m_width)
				printf("nix gut");
		}
		Halcon::get_image_pointer1(*obj, &pointer, &t, &w, &h);
    long model = sm->GetShapeModel(scale);
		(*m_paramList)[8] = numOfObjects;

		while(minScore > 0.59 && n > 0)
		{
			if(trackPossible)
			{
				GuidedPoseRestictionComb(m_paramList, m_paramNameList, model, Partly, lastKnownPose, calib->CamParam(), pm);
			}
			else
			{
			/* Restriction of Search array (TODO derive it from estimated position)*/

        (*m_paramList)[0] = pm->m_longitudeMin;/*(*m_paramList)[0] = model->GetShapeModel3dParams((*m_paramNameList)[0].S()) * Partly;/*long min*/
        (*m_paramList)[1] = pm->m_longitudeMax;/*model->GetShapeModel3dParams((*m_paramNameList)[1].S()) * Partly;		 /*long max*/
        (*m_paramList)[2] = pm->m_latitudeMin;/*model->GetShapeModel3dParams((*m_paramNameList)[2].S()) * Partly;		 /*lat min*/
        (*m_paramList)[3] = pm->m_latitudeMax;/*model->GetShapeModel3dParams((*m_paramNameList)[3].S()) * Partly;		 /*lat max*/
        (*m_paramList)[4] = pm->m_camRollMin;/*model->GetShapeModel3dParams((*m_paramNameList)[4].S());		 /*cam roll min*/
        (*m_paramList)[5] = pm->m_camRollMax;/*model->GetShapeModel3dParams((*m_paramNameList)[5].S());		 /*cam roll max*/
        (*m_paramList)[6] = pm->m_distMin;/*model->GetShapeModel3dParams((*m_paramNameList)[6].S());		 /*dist min*/
        (*m_paramList)[7] = pm->m_distMax;/*model->GetShapeModel3dParams((*m_paramNameList)[7].S());		 /*dist max*/
			}
			printf("Parameter for detection (%f %%):\n%f %f // %f %f \n %f %f // %f %f\n", Partly * 100,
/*long min*/																		  (*m_paramList)[0].D(),
/*long max*/																		  (*m_paramList)[1].D(),
/*lat min*/																			  (*m_paramList)[2].D(),
/*lat max*/																			  (*m_paramList)[3].D(),
/*cam roll min*/																	  (*m_paramList)[4].D(),
/*cam roll max*/																	  (*m_paramList)[5].D(),
/*dist min*/																		  (*m_paramList)[6].D(),
/*dist max*/																		  (*m_paramList)[7].D());
			n--;
#ifdef _DEBUG
			printf("Run FindShapeModel with minscore: %f \nlevels %d\n", minScore, level);
#endif
			try
			{
				if(model != NULL)
				{
          find_shape_model_3d(*obj, model, /*< Image here*/
                  minScore, m_greediness, level, /*< Min Score, Greediness, Levels*/
                  *m_paramNameList, *m_paramList, /*< Generic Params (gp), gp Values*/
                  &pose, &CovPose, &Score);/*< Cov,Score */
				}
			}
			catch(Halcon::HException ex)
			{
				printf("Finding failed: %s  (img %d, %d)\n", ex.message, w[0].I(), h[0].I());
			}
			printf("Finished detection: %d\n", Score.Num());
			if(Score.Num() > 0)
			{
//#ifdef _DEBUG
				printf("Find %d objects with max score %f\nPose: %f,%f,%f, // %f,%f,%f // %d\n", Score.Num(), Score[0].D(), pose[0].D(), pose[1].D(), pose[2].D(), pose[3].D(), pose[4].D(), pose[5].D(), pose[6].I());
//#endif
				qualityMeasure = Score[0].D();
				numOfObjects = Score.Num();
				if(Score[0].D() > 0.85)
				{
					m_minScore = minScore;
					m_levels = level;
				}
				break;
			}
			else
			{
				Partly = 1.0 - (0.05 * (n-1));
				numOfObjects = 0;
				//minScore -= 0.05;
/*
if(level > 2)
					level -= 1;*/
			}
		}
		if(scale < 1.0 && scale > 0.001)
			delete obj;
		img->Free();
		if(object.m_relPose != NULL)
			result.push_back(RelPoseFactory::FRelPose(pose, CovPose, camPose, object.m_relPose->m_uniqueID));
		else
      result.push_back(RelPoseFactory::FRelPose(pose, CovPose, camPose));
		if(Score.Num() > 0)
			sm->SetLastMatchedImage(img, result[0]);

		return result;
	}
#endif
	img->Free();
	return result;
}

double CombinedShapeDescr::CheckSignature(Signature& object)
{
	if(object.GetElement(0,DESCRIPTOR_SHAPE) && object.GetElement(0,DESCRIPTOR_FEATURE))
		return 1.0;
	else
		return 0.0;
}

std::vector<RelPose*> CombinedShapeDescr::InnerDesc(Image* img,RelPose* camPose, Calibration* calib, RelPose* lastKnownPose, Signature& object, int &numOfObjects, double& qualityMeasure)
{
  std::vector<RelPose*> result;
  HTuple camparam = calib->CamParam();
	HTuple cammatrix = calib->CamMatrix();

	if(img->GetType() == HALCONIMAGE)
	{
		bool trackPossible = TrackingPossible(*img, object, lastKnownPose);
		int n = 4;
		double Partly = 1.0 - (0.3* (n-1));
		HTuple empty;
		HTuple  matches,
				xDescriptorSource,yDescriptorSource,zDescriptorSource,
                rowDescriptorTarget,colDescriptorTarget,
                xFinalSource,yFinalSource,zFinalSource,
                rowFinalTarget,colFinalTarget;
		printf("Get Descriptor Model\n");
		PointDescrModel* sm = (PointDescrModel*)(object.GetElement(0, DESCRIPTOR_FEATURE ));
		Halcon::HTuple hommat, pose, cov;
		try
		{
			Hobject* imgs = img->GetHImage();
			int handle = sm->GetDescriptorHandle();
			Halcon::find_descriptor_model_3d(*imgs, handle,
										empty, empty, empty, empty,
										42, 1, cammatrix,
										&hommat,&pose, &cov, &matches,
										&xDescriptorSource,&yDescriptorSource,&zDescriptorSource,
										&rowDescriptorTarget,&colDescriptorTarget,
										&xFinalSource,&yFinalSource,&zFinalSource,
										&rowFinalTarget,&colFinalTarget);
			//printf("Descriptor Target\n");
			img->Free();
			//pose = RansacPose(xDescriptorSource, yDescriptorSource, zDescriptorSource, rowDescriptorTarget, colDescriptorTarget, camparam);
			/*printf("Point-correspondences after Ransac (blue)\n");
			for(int i = 0; i < rowFinalTarget.Num(); i++)
			{
				printf("2d: (%f, %f) -> 3d (%f, %f, %f)\n", rowFinalTarget[i].D(), colFinalTarget[i].D(), xFinalSource[i].D(),yFinalSource[i].D(),zFinalSource[i].D());
			}*/
			if(matches.Num() > 0 && matches[0].ValType() == 1)
			{
				//Halcon::convert_pose_type(pose,"Rp+T", "gba", "coordinate_system",&pose);
				printf("\n");
				printf("Estimated Pose:\n");
				for(int i = 0; i < pose.Num() -1; i++)
				{
					printf("%f, ", pose[i].D());
				}
				printf("%d\n", pose[pose.Num() -1].I());
				printf("\n");
        HTuple pose_sel, cov_sel;
        for(int i = 0; i < pose.Num() / 7; i++)
        {
          tuple_select_range(pose, 0 + 7*i, 6 + 7*i, &pose_sel);
          tuple_select_range(cov, 0+ 6*i, 5+ 6*i, &cov_sel);
          if(object.m_relPose != NULL && i == 0)
            result.push_back(RelPoseFactory::FRelPose(pose_sel, cov_sel, camPose, object.m_relPose->m_uniqueID));
          else
            result.push_back(RelPoseFactory::FRelPose(pose_sel, cov_sel, camPose));
        }
				numOfObjects = matches[0].I();
				qualityMeasure = 0.99;
				sm->SetLastMatchedImage(img, result[0]);
			}
			else
			{
				numOfObjects = 0;
				qualityMeasure = 0.99;
        result.push_back(object.m_relPose);
			}
			return result;
		}
		catch(Halcon::HException ex)
		{
			printf("Error  in DescriptorBased: %s\n", ex.message);
			qualityMeasure = 0.0;
		}
	}
	return result;
}
