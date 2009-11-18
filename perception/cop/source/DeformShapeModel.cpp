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
#include "DeformShapeModel.h"
#include "XMLTag.h"
#include "Camera.h"


#ifdef HALCONIMG
#include "cpp/HalconCpp.h"
using namespace Halcon;
#endif



#define XML_NODE_FILENAME "filename"
#define XML_NODE_XNORM "xnorm"
#define XML_NODE_YNORM "ynorm"

using namespace cop;


DeformShapeModel::DeformShapeModel(Class* classref, Signature* sig) :
	Descriptor(classref),
	m_handle(-1)
{

	/*
	sig->Get
	create_planar_uncalib_deformable_model();
	*/
}

DeformShapeModel::DeformShapeModel(XMLTag* tag) :
	Descriptor(tag),
	m_handle(-1),
	m_filename("")
{
  if(tag != NULL)
  {
    XMLTag* xml_filename = tag->GetChild(XML_NODE_FILENAME);
    if(xml_filename != NULL)
    {
        m_filename = xml_filename->GetCDataST();
        LoadFromFile(m_filename);
    }
    else
    {
       printf("Error loading DeformShapeModel: no filename tag found\n");
    }
  }
}

DeformShapeModel::~DeformShapeModel(void)
{
#ifdef HALCONIMG
	clear_deformable_model(m_handle);
#endif
}

bool DeformShapeModel::LoadFromFile(std::string fileName)
{
#ifdef HALCONIMG
	Hlong id;
	try
	{
            printf("Reading deform shape model\n\n\n\n");
            read_deformable_model(fileName.c_str(), &id);

            printf("Success reading %s=> %ld\n\n\n", fileName.c_str(), id);
	}
	catch(Halcon::HException ex)
	{
          printf("Error reading DeformShapeModel from file: %s\n", fileName.c_str());
          m_handle  = -1;
          return false;
	}
	m_handle = id;
#endif
	return true;
}

void DeformShapeModel::Show(RelPose* pose, Camera* cam)
{
#ifdef HALCONIMG
  Hobject xld;
  //get_deformable_model_contours(&xld, m_handle, 1);
  HWindow* hwin = cam->GetWindow();
  hwin->SetColor("red");
  hwin->SetLineWidth(3);
  HTuple row, col, pose_ht;
  pose->GetPose(&pose_ht);
  project_3d_point(pose_ht[0],pose_ht[1], pose_ht[2], cam->m_calibration.CamParam(), &row, &col);
  disp_cross(hwin->WindowHandle(), row, col, 60, 0.79);
  hwin->SetLineWidth(1);
#endif
}

#ifdef HALCONIMG
double DeformShapeModel::DefineDeformShapeModel(Halcon::Hobject* img, Halcon::Hobject* region, Camera* cam,RelPose* pose)
{
	Halcon::Hobject imgReduced;
	Halcon::HTuple score, autoTuple, empty, model_id, pose_in, pose_out, cov;

  pose->GetPose(&pose_in);

	if(m_handle != -1)
		Halcon::clear_deformable_model(m_handle);

	/*reduce region of interest to a selected part of the image, that will be used for generating the model*/
	Halcon::reduce_domain(*img,*region, &imgReduced);
	/*temp varibale for constant auto*/
	autoTuple = "auto";
	/*Create the model*/
  Halcon::create_planar_calib_deformable_model(imgReduced, cam->m_calibration.CamParam(), pose_in, autoTuple,
			-1.5, 1.5, autoTuple, pose_in[3].D() * 0.8 , pose_in[3].D() * 1.2, autoTuple, 0.95,1.05,
			autoTuple,"point_reduction_low", "ignore_part_polarity", autoTuple,autoTuple, empty,empty,&model_id);
	/*Calculate the homography that is needed as starting value for tracking*/
	Halcon::find_planar_calib_deformable_model(imgReduced, model_id, -0.01,0.01,0.9,
												1.1,1.0,1.0,0.8,1,1,0,0.9, empty, empty, &pose_out, &cov, &score);
	/*Save the model ID*/
	m_handle = model_id[0].I();
	if(score.Num() > 0)
		return score[0].D();
	else
		return 0.0;
}
#endif

void DeformShapeModel::SaveTo(XMLTag* tag)
{
  tag->AddChild(XMLTag::Tag(m_filename, XML_NODE_FILENAME));
}

#endif /*DEFORMSHAPE_AVAILABLE*/


