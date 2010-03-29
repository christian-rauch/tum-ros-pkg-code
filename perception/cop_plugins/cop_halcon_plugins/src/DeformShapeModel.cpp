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


/*#ifdef DEFORMSHAPE_AVAILABLE*/
#include "DeformShapeModel.h"
#include "XMLTag.h"
#include "Camera.h"
#include "RelPoseHTuple.h"

#include "cpp/HalconCpp.h"
using namespace Halcon;



#define XML_NODE_FILENAME "filename"
#define XML_NODE_XNORM "xnorm"
#define XML_NODE_YNORM "ynorm"

#define XML_ATTRIBUTE_AUTOGENERATED "auto"

using namespace cop;


DeformShapeModel::DeformShapeModel(Class* classref) :
	Descriptor(classref),
	m_handle(-1),
	m_bWritten(false),
	m_regionTemp(NULL),
	m_autoLearned(true)
{

	/*
	sig->Get
	create_planar_uncalib_deformable_model();
	*/
}

DeformShapeModel::DeformShapeModel() :
	m_handle(-1),
  m_bWritten(false),
  m_regionTemp(NULL),
  m_autoLearned(true)
{

}

void DeformShapeModel::SetData(XMLTag* tag)
{
	Descriptor::SetData(tag),
	m_handle = -1;
	m_filename = "";


  if(tag != NULL)
  {
    XMLTag* xml_filename = tag->GetChild(XML_NODE_FILENAME);
    if(xml_filename != NULL)
    {
        m_filename = xml_filename->GetCDataST();
        LoadFromFile(m_filename);
        m_bWritten = true;
    }
    else
    {
       printf("Error loading DeformShapeModel: no filename tag found\n");
       m_bWritten = false;
    }
    m_autoLearned = tag->GetProperty(XML_ATTRIBUTE_AUTOGENERATED, "false").compare("true") == 0 ? true : false;
  }
}

DeformShapeModel::~DeformShapeModel(void)
{
#ifdef HALCONIMG
	clear_deformable_model(m_handle);
	delete m_regionTemp;
#endif
}

bool DeformShapeModel::LoadFromFile(std::string fileName)
{
#ifdef HALCONIMG
	Hlong id;
	try
	{
      read_deformable_model(fileName.c_str(), &id);
      m_bWritten = true;
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

void DeformShapeModel::Show(RelPose* pose, Sensor* camin)
{

  if(camin != NULL && camin->IsCamera())
  {
    Camera* cam = (Camera*)camin;
    Hobject xld;
  //get_deformable_model_contours(&xld, m_handle, 1);
    try
    {
      HWindow* hwin = cam->GetWindow();
      hwin->SetColor("red");
      hwin->SetLineWidth(3);
      HTuple row, col, pose_ht, CamParam = cam->m_calibration.CamParam();
      RelPoseHTuple::GetPose(pose, &pose_ht, cam->m_relPose->m_uniqueID);
      /*TOCHECK why?*/
      /**  The next 4 lines rotate the pose to the training
          pose that is needed to display everything right
          This might be wrong for some pretrained things
      */
      if(m_autoLearned)
      {
        HTuple HomMat3D;
        pose_to_hom_mat3d (pose_ht, &HomMat3D);
        hom_mat3d_rotate_local(HomMat3D, M_PI, "y", &HomMat3D);
        hom_mat3d_to_pose (HomMat3D, &pose_ht);
      }

      project_3d_point(pose_ht[0],pose_ht[1], pose_ht[2], CamParam, &row, &col);
      disp_cross(hwin->WindowHandle(), row, col, 60, 0.79);

      HTuple ModelRow, ModelCol, ModelPose, hom2d(6), NumberContour, HomMat3D;
      Hobject ModelContours, ContoursAffinTrans, ContoursTrans, FoundContour, ObjectSelected;
      get_deformable_model_contours (&ModelContours, m_handle, 1);
      get_deformable_model_params (m_handle, "model_row", &ModelRow);
      get_deformable_model_params (m_handle, "model_col", &ModelCol);
      get_deformable_model_params (m_handle, "model_pose", &ModelPose);
      printf("model_pose: %fm %fm %fm - %f� %f� %f�\n", ModelPose[0].D(), ModelPose[1].D(), ModelPose[2].D(),
                                                     ModelPose[3].D(), ModelPose[4].D(), ModelPose[5].D());
      hom2d[0] = 1;hom2d[1] = 0;hom2d[2] = ModelRow;hom2d[3] = 0; hom2d[4] = 1; hom2d[5] = ModelCol;
      affine_trans_contour_xld (ModelContours, &ContoursAffinTrans, hom2d);
      contour_to_world_plane_xld (ContoursAffinTrans, &ContoursTrans, CamParam, ModelPose, "m");
      count_obj (ContoursTrans, &NumberContour);
      pose_to_hom_mat3d (pose_ht, &HomMat3D);
      gen_empty_obj (&FoundContour);
      for (int index = 1; index <= NumberContour; index++)
      {
        HTuple X,Y,Z, Xc, Yc, Zc, R, C;
        Hobject ModelWorld;
        select_obj (ContoursTrans, &ObjectSelected, index);
        get_contour_xld (ObjectSelected, &Y, &X);
        Halcon::tuple_gen_const(X.Num(), 0.0, &Z);
        affine_trans_point_3d (HomMat3D, X, Y, Z, &Xc, &Yc, &Zc);
        project_3d_point (Xc, Yc, Zc, CamParam, &R, &C);
        gen_contour_polygon_xld (&ModelWorld, R, C);
        concat_obj (FoundContour, ModelWorld, &FoundContour);
      }
      disp_obj(FoundContour, hwin->WindowHandle());

      if(m_regionTemp != NULL)
      {
        hwin->SetDraw("margin");
         disp_obj( *m_regionTemp, hwin->WindowHandle());
         delete m_regionTemp;
         m_regionTemp = NULL;
      }
      hwin->SetLineWidth(1);
    }
    catch(Halcon::HException ex)
    {
      printf("Problems showing DeformShapeModel: %s\n", ex.message);
    }
    catch(const char *text)
    {
      printf("Problems showing DeformShapeModel: %s\n", text);
    }
  }

}


double DeformShapeModel::DefineDeformShapeModel(Image* image, Halcon::Hobject* region, Camera* cam, RelPose* pose)
{
  Halcon::HTuple score;
  m_autoLearned = true;
  try
  {
    Halcon::Hobject imgReduced, *img;
    Halcon::HTuple  autoTuple, empty, model_id, pose_in, pose_out, cov, hom, hom_new;
    m_regionTemp = new Halcon::Hobject();

    RelPose* pose_image = image->GetPose();
    RelPoseHTuple::GetPose(pose, &pose_in, pose_image->m_uniqueID);
    Halcon::pose_to_hom_mat3d(pose_in, &hom);
    Halcon::hom_mat3d_rotate_local(hom, M_PI, "y", &hom_new);
    Halcon::hom_mat3d_to_pose(hom_new, &pose_in);
    img = image->GetHImage();

    if(m_handle != -1)
      Halcon::clear_deformable_model(m_handle);

    /*reduce region of interest to a selected part of the image, that will be used for generating the model*/
    Halcon::erosion_circle(*region,m_regionTemp, 5);
    Halcon::reduce_domain(*img, *m_regionTemp, &imgReduced);
    /*temp varibale for constant auto*/
    autoTuple = "auto";
    /*Create the model*/
    Halcon::create_planar_calib_deformable_model(imgReduced, cam->m_calibration.CamParam(), pose_in, autoTuple,
        -0.8, 0.8, autoTuple, 0.8 , 1.2, autoTuple, 0.8,1.2,
        autoTuple,"point_reduction_low", "use_polarity", autoTuple,autoTuple, empty,empty,&model_id);
    /*Calculate the homography that is needed as starting value for tracking*/
    Halcon::find_planar_calib_deformable_model(imgReduced, model_id, -0.01,0.01,0.9,
                          1.1,1.0,1.0,0.8,1,1,0,0.9, empty, empty, &pose_out, &cov, &score);
    printf("Compare Pose_in with Pose_out:\n %f - %f = %f\n %f - %f = %f\n %f - %f = %f\n", pose_in[0].D(), pose_out[0].D(), pose_in[0].D() - pose_out[0].D(), pose_in[1].D() , pose_out[1].D(), pose_in[1].D() - pose_out[1].D(), pose_in[2].D(), pose_out[2].D(), pose_in[2].D() - pose_out[2].D());
    m_handle = model_id[0].I();
    /*Save the model ID*/
  }
  catch(HException ex)
  {
     printf("Error Learning Deform Shape Model: %s\n", ex.message);
     throw "Error Learning Deform Shape Model";
  }
	if(score.Num() > 0)
		return score[0].D();
	else
		return 0.0;
}


void DeformShapeModel::SaveTo(XMLTag* tag)
{
  Descriptor::SaveTo(tag);
  if(!m_bWritten)
  {
    if(m_filename.length() == 0)
    {
      std::stringstream st;
      st << m_class->GetName() << ".dsm";
      m_filename = st.str();
    }
    try
    {
      Halcon::write_deformable_model(m_handle, m_filename.c_str());
    }
    catch(Halcon::HException ex)
    {
      printf("DeformShapeModel::SaveTo: Error: %s\n", ex.message);
    }
    m_bWritten = true;
  }
  tag->AddChild(XMLTag::Tag(m_filename, XML_NODE_FILENAME));
  tag->AddProperty(XML_ATTRIBUTE_AUTOGENERATED, (m_autoLearned ? "true" : "false"));
}

/*#endif*/ /*DEFORMSHAPE_AVAILABLE*/


