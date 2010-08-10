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

#include "HClusterDetector.h"
#include "SegmentPrototype.h"
#include "XMLTag.h"
#include "SwissRangerReading.h"
#include "BoostUtils.h"

#include "RelPoseHTuple.h"

#include <sys/time.h>


#define XML_ATTRIBUTE_SR4LO "sr_loid"
#define XML_ATTRIBUTE_PTULO "ptu_loid"

using namespace cop;

using namespace std;
using namespace ros;
//using namespace std_msgs;
using namespace sensor_msgs;
using namespace geometry_msgs;
#include <cpp/HalconCpp.h>


// Procedures
void extract_clusters (Halcon::HTuple HomMat3d, Halcon::HTuple X, Halcon::HTuple Y,
    Halcon::HTuple Z, Halcon::HTuple CamParam, Halcon::HTuple *table_height, Halcon::HTuple *Mean_X,
    Halcon::HTuple *Mean_Y, Halcon::HTuple *Mean_Z, Halcon::HTuple *cov33)
{
  using namespace Halcon;
  printf("inside \n");
  // Local iconic variables
  Hobject  ImageZ, ImageXOrig, ImageYOrig, Region;
  Hobject  Region1, RegionDifference1, RegionClosing, ConnectedRegions;
  Hobject  SelectedRegions, ObjectSelected;


  // Local control variables
  HTuple  Qx, Qy, Qz, Row, Column, AbsoluteHisto;
  HTuple  RelativeHisto, Min1, Max1, Range, Function, SmoothedFunction;
  HTuple  Min, Max, Y1, Index, i, thres, start, Or, Number;
  HTuple  index, Rows, Columns, Xreg, Yreg, Zreg, MeanX, MeanY;
  HTuple  MeanZ, Deviation, DeviationY, DeviationX;
  printf("Length of X: %ld\n", X.Num());
  affine_trans_point_3d(HomMat3d, X, Y, Z, &Qx, &Qy, &Qz);

  printf("CamParam: %f %f %f %f %f %f\n", CamParam[0].D(),CamParam[1].D(),CamParam[2].D(),CamParam[3].D(),CamParam[4].D(),CamParam[5].D());
  try
  {
    project_3d_point(X, Y, Z, CamParam, &Row, &Column);
  }
  catch(HException ex)
  {
    HTuple Row_temp, Column_temp;
    Row = HTuple();
    Column = HTuple();
    for(int i = 0; i < X.Num(); i++)
    {
      if(Z[i].D() > 0.15)
      {
        project_3d_point(X[i].D(), Y[i].D(), Z[i].D(), CamParam, &Row_temp, &Column_temp);
        tuple_concat(Row, Row_temp,&Row);
        tuple_concat(Column, Column_temp,&Column);
      }
    }
  }

  gen_image_const(&ImageZ, "float", 640, 480);
  gen_image_const(&ImageXOrig, "float", 640, 480);
  gen_image_const(&ImageYOrig, "float", 640, 480);
  set_grayval(ImageXOrig, Row.Int(), Column.Int(), Qx);
  set_grayval(ImageYOrig, Row.Int(), Column.Int(), Qy);
  //
  set_grayval(ImageZ, Row.Int(), Column.Int(), Qz);
  gen_region_points(&Region, Row.Int(), Column.Int());
  //
  //threshold (ImageZ, Region, -0.3, 2.0)
  gray_histo(Region, ImageZ, &AbsoluteHisto, &RelativeHisto);
  min_max_gray(Region, ImageZ, 0, &Min1, &Max1, &Range);
  printf("Min-max: %f - %f\n",Min1[0].D(), Max1[0].D());
  create_funct_1d_array(RelativeHisto, &Function);
  smooth_funct_1d_gauss(Function, 2, &SmoothedFunction);

  local_min_max_funct_1d(SmoothedFunction, "strict_min_max", "true", &Min, &Max);
  get_y_value_funct_1d(SmoothedFunction, Max, "constant", &Y1);
  (*table_height) = HTuple();
  for (Index=0; Index<=(Y1.Num())-1; Index+=1)
  {
    if (HTuple(Y1[Index])>0.03)
    {
      (*table_height).Append(Min1+((HTuple(Max[Index])*(Max1-Min1))/255));
    }
    printf("Local max of %f  at height %f \n ", Y1[Index].D(), (Min1+((HTuple(Max[Index])*(Max1-Min1))/255))[0].D());
  }

  if (HDevWindowStack::IsOpen())
    disp_obj(ImageZ, HDevWindowStack::GetActive());
  (*Mean_X) = HTuple();
  (*Mean_Y) = HTuple();
  (*Mean_Z) = HTuple();
  (*cov33) = HTuple();
  printf("number of maxima: %ld\n", (*table_height).Num());
  //
  for (i=0; i<=((*table_height).Num())-1; i+=1)
  {
    if (i==(((*table_height).Num())-1))
    {
      thres = 2.5;
    }
    else
    {
      thres = HTuple((*table_height)[i+1])-(0.1*HTuple((*table_height)[i+1]));
    }
    start = HTuple((*table_height)[i])+0.02;
    threshold(ImageZ, &Region1, start, thres);
    intersection(Region1, Region, &RegionDifference1);
    closing_circle(RegionDifference1, &RegionClosing, 5);
    connection(RegionClosing, &ConnectedRegions);
    select_shape(ConnectedRegions, &SelectedRegions, "area", "and", X.Num() / 150, 99999);
    tuple_or(HTuple((*table_height)[i])>0.5, ((*table_height).Num())==1, &Or);
    if (0 != Or)
    {
      count_obj(SelectedRegions, &Number);
      for (index=1; index<=Number; index+=1)
      {
        select_obj(SelectedRegions, &ObjectSelected, index);
        get_region_points(ObjectSelected, &Rows, &Columns);
        get_grayval(ImageXOrig, Rows, Columns, &Xreg);
        get_grayval(ImageYOrig, Rows, Columns, &Yreg);
        get_grayval(ImageZ, Rows, Columns, &Zreg);
        tuple_mean(Xreg, &MeanX);
        tuple_mean(Yreg, &MeanY);
        tuple_mean(Zreg, &MeanZ);
        double cov[9];
        memset(cov, 0.0, sizeof(*cov)*9);

        HTuple x_off = Xreg - MeanX;
        HTuple y_off = Yreg - MeanY;
        HTuple z_off = Zreg - MeanZ;
        tuple_abs(x_off, &x_off);
        tuple_abs(y_off, &y_off);
        tuple_abs(z_off, &z_off);
        HTuple cov0,cov1, cov2,cov3,cov4,cov5,cov6,cov7,cov8,cov9, x_off_sorted, y_off_sorted, z_off_sorted, indices;
        tuple_sort_index(x_off, &x_off_sorted);
        tuple_select_range(x_off_sorted, HTuple(x_off_sorted.Num()*0.05).Int(), HTuple(x_off_sorted.Num()*0.95).Int(), &x_off_sorted);

        tuple_sort_index(y_off, &y_off_sorted);
        tuple_select_range(y_off_sorted, HTuple(y_off_sorted.Num()*0.05).Int(), HTuple(y_off_sorted.Num()*0.95).Int(), &y_off_sorted);

        tuple_sort_index(z_off, &z_off_sorted);
        tuple_select_range(z_off_sorted, HTuple(z_off_sorted.Num()*0.05).Int(), HTuple(z_off_sorted.Num()*0.95).Int(), &z_off_sorted);

        tuple_concat(x_off_sorted,y_off_sorted, &indices);
        tuple_concat(indices,z_off_sorted, &indices);

        tuple_select(x_off, indices, &x_off);
        tuple_select(y_off, indices, &y_off);
        tuple_select(z_off, indices, &z_off);

        printf("Length before: %ld, length after: %ld\n", x_off_sorted.Num(), x_off.Num());

        tuple_sum(x_off*x_off, &cov0);
        tuple_sum(y_off*x_off, &cov1);
        tuple_sum(z_off*x_off, &cov2);
        tuple_sum(x_off*y_off, &cov3);
        tuple_sum(y_off*y_off, &cov4);
        tuple_sum(z_off*y_off, &cov5);
        tuple_sum(x_off*z_off, &cov6);
        tuple_sum(y_off*z_off, &cov7);
        tuple_sum(z_off*z_off, &cov8);

        cov33->Append((cov0 / x_off.Num()) * 1.05);
        cov33->Append((cov1 / x_off.Num())* 1.05);
        cov33->Append((cov2 / x_off.Num())* 1.05);

        cov33->Append((cov3 / x_off.Num())* 1.05);
        cov33->Append((cov4 / x_off.Num())* 1.05);
        cov33->Append((cov5 / x_off.Num())* 1.05);

        cov33->Append((cov6 / x_off.Num())* 1.05);
        cov33->Append((cov7 / x_off.Num())* 1.05);
        cov33->Append((cov8 / x_off.Num())* 1.05);


        (*Mean_X).Append(MeanX);
        (*Mean_Y).Append(MeanY);
        (*Mean_Z).Append(MeanZ);
        //
      }
    }
  }
  return;
}




HClusterDetector::HClusterDetector(int srid, int ptuid) :
  m_swissranger_jlo_id(srid),
  m_ptu_jlo_id(ptuid)
{
}

HClusterDetector::HClusterDetector()
{
}

void HClusterDetector::SetData(XMLTag* tag)
{

  std::string camparam = tag->GetProperty(XML_ATTRIBUTE_FILENAME, "");
  if(camparam.length() > 0)
  {
    Halcon::HTuple camparam_h;
    Halcon::read_cam_par(camparam.c_str(), &camparam_h);
    for(int i = 0; i < camparam_h.Num(); i++)
    {
      m_camparams.push_back(camparam_h[i].D());
    }
    printf("Read camparam: %s\n", camparam.c_str());
 }

  if(tag != NULL)
  {
      m_swissranger_jlo_id = tag->GetPropertyInt(XML_ATTRIBUTE_SR4LO, 0);
      if(m_swissranger_jlo_id == 0)
      {
        std::string name = tag->GetProperty(XML_ATTRIBUTE_SR4LO, "/sr4");
        RelPose* pose = RelPoseFactory::GetRelPose(name);
        if(pose == NULL)
          m_swissranger_jlo_id = 1;
        else
        {
          printf("Read HClusterDetector with %ld as camera position (%s)\n", pose->m_uniqueID, name.c_str());
          m_swissranger_jlo_id =pose->m_uniqueID;
          RelPoseFactory::FreeRelPose(pose);
        }
      }
      m_ptu_jlo_id = tag->GetPropertyInt(XML_ATTRIBUTE_PTULO, 0);
      if(m_ptu_jlo_id == 0)
      {
        std::string name = tag->GetProperty(XML_ATTRIBUTE_PTULO, "/base_link");
        RelPose* pose = RelPoseFactory::GetRelPose(name);
        if(pose == NULL)
          m_ptu_jlo_id = 1;
        else
        {
          m_ptu_jlo_id =pose->m_uniqueID;
          RelPoseFactory::FreeRelPose(pose);
        }
      }
  }
}


HClusterDetector::~HClusterDetector()
{

}

std::vector<RelPose*> HClusterDetector::Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& object, int &numOfObjects, double& qualityMeasure)
{
  std::vector<RelPose*> results;
    //Calibration* calib = &cam[0]->m_calibration;
  SegmentPrototype* proto = (SegmentPrototype*)object.GetElement(0, DESCRIPTOR_SEGMPROTO);
  printf("HClusterDetector::Perform: Got SegmentPrototype\n");
  for(std::vector<Sensor*>::const_iterator it = sensors.begin(); it != sensors.end(); it++)
  {
    if((*it)->GetName().compare(XML_NODE_SWISSRANGER) == 0)
    {
      try
      {
        results = Inner(*it, proto, numOfObjects, qualityMeasure);
      }
      catch (const char* text )
      {
         printf("Error in HClusterDetector: %s\n", text);
      }
      break;
    }
  }
  /*TODO plane clusters*/
  return results;
}



inline void normalize(double &a,double &b, double &c)
{
  double norm = sqrt(a*a + b*b + c*c);
  a /= norm;
  b /= norm;
  c /= norm;
}

inline double scalarproduct(const double &a,const double &b, const double &c, const double &d, const double &e, const double &f)
{
  return a * d + b* e + c*f;
}

inline void CrossProduct_l(const double b_x, const double b_y,const double b_z,const double c_x,const double c_y,const double c_z,double &a_x,double &a_y,double &a_z)
{
    a_x = b_y*c_z - b_z*c_y;
    a_y = b_z*c_x - b_x*c_z;
    a_z = b_x*c_y - b_y*c_x;
}



std::vector<RelPose*> HClusterDetector::Inner(Sensor* sens, SegmentPrototype* obj_descr, int &numOfObjects, double& qualityMeasure)
{
  std::vector<RelPose*> results;
  printf("Inner\n");

  qualityMeasure = 0.0;
  bool parallel = true;
  unsigned long ref_frame = m_ptu_jlo_id;

  if(obj_descr != NULL)
  {
    ref_frame = obj_descr->GetFrameId();
    parallel = obj_descr->m_parallel;
  }

  try
  {
    Halcon::HTuple HomMat3d, HomMat3Dinv,  CamParam(8),  table_height, Mean_X, Mean_Y, Mean_Z, cov33;
    for(size_t i = 0; i < m_camparams.size(); i++)
    {
      CamParam[i] = m_camparams[i];
    }
    SwissRangerReading* read = (SwissRangerReading*)sens->GetReading();
    if(read == NULL)
      return results;
    RelPose* ptu = RelPoseFactory::FRelPose(ref_frame);
    sensor_msgs::PointCloud& pcd = read->m_image;
    RelPoseHTuple::GetHommat(sens->GetRelPose(), &HomMat3d, ref_frame);
    hom_mat3d_invert(HomMat3d, &HomMat3Dinv);
    Halcon::HTuple X,  Y,  Z;
    std::vector<std::vector<double> > vec;
    for(size_t i = 0; i < (pcd.points.size()); i++)
    {
      X[i] = pcd.points[i].x;
      Y[i] = pcd.points[i].y;
      Z[i] = pcd.points[i].z;
    }
    printf("Before extract cluster\n");
    extract_clusters(HomMat3d, X, Y, Z, CamParam, &table_height, &Mean_X, &Mean_Y, &Mean_Z, &cov33);
    printf("After extract cluster\n");

    for(int x= 0; x < Mean_X.Num(); x++)
    {
      Matrix rotmat(4,4);
      rotmat << 1 << 0<< 0  << Mean_X[x].D()
             << 0 << 1 << 0 << Mean_Y[x].D()
             << 0 << 0<< 1  << Mean_Z[x].D()
             << 0 << 0 << 0 << 1;

      cout <<  "Matrix from plane_clusters:" << endl << rotmat << endl;
      Matrix cov (6,6);
     if(parallel)
     {
      cov << sqrt(cov33[9*x+0].D()) << sqrt(cov33[9*x+1].D()) << sqrt(cov33[9*x+2].D()) << 0   << 0   << 0
         << sqrt(cov33[9*x+3].D()) << sqrt(cov33[9*x+4].D()) << sqrt(cov33[9*x+5].D()) <<  0  << 0   << 0
         << sqrt(cov33[9*x+6].D()) << sqrt(cov33[9*x+7].D()) << sqrt(cov33[9*x+8].D()) << 0   << 0   << 0
         <<0    << 0    << 0    <<  ((obj_descr == NULL) ? 0.2 : obj_descr->m_covRotX) << 0   << 0
         <<0    << 0    << 0    << 0   << ((obj_descr == NULL) ? 0.2 :obj_descr->m_covRotY) << 0
         <<0    << 0    << 0    << 0   << 0   << ((obj_descr == NULL) ? 0.8 :obj_descr->m_covRotZ);
     }
     else
     {
       cov << sqrt(cov33[9*x+3].D()) << sqrt(cov33[9*x+4].D()) << sqrt(cov33[9*x+5].D()) << 0  << 0   << 0
         << sqrt(cov33[9*x+0].D()) << sqrt(cov33[9*x+1].D()) << sqrt(cov33[9*x+2].D()) << 0  << 0   << 0
         << sqrt(cov33[9*x+6].D()) << sqrt(cov33[9*x+7].D()) << sqrt(cov33[9*x+8].D()) << 0   << 0   << 0
         <<0    << 0    << 0    << ((obj_descr == NULL) ? 0.2 :obj_descr->m_covRotY) << 0   << 0
         <<0    << 0    << 0    << 0   << ((obj_descr == NULL) ? 0.2 :obj_descr->m_covRotX) << 0
         <<0    << 0    << 0    << 0   << 0   << ((obj_descr == NULL) ? 0.8 :obj_descr->m_covRotZ);
      }

      cout << "Cov from pc: "<< endl <<  cov << endl;
      RelPose* pose_temp = RelPoseFactory::FRelPose(ptu, rotmat, cov);
      double temp_qual = min(1.0, max(0.0, fabs((sqrt(cov33[9*x+4].D())*sqrt(cov33[9*x+0].D())*sqrt(cov33[9*x+8].D()))) * 500 ));
      if(x == 0)
       qualityMeasure =temp_qual;

      if(pose_temp == NULL)
        continue;
      pose_temp->m_qualityMeasure = temp_qual;
      results.push_back(pose_temp);
    }

    RelPoseFactory::FreeRelPose(ptu);

  }
  catch(Halcon::HException ex)
  {
    printf("HClusterExtractor: %s\n", ex.message);
  }


  return results;
}

double HClusterDetector::CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors)
{
  for(std::vector<Sensor*>::const_iterator it = sensors.begin(); it != sensors.end(); it++)
  {
    if((*it)->GetName().compare(XML_NODE_SWISSRANGER) == 0)
    {
      if(object.GetElement(0, DESCRIPTOR_SEGMPROTO) != NULL )
        return 0.1;
      else
        return 0.0;
    }
  }
  return 0.0;
}


bool HClusterDetector::TrackingPossible(const Reading& img, const Signature& sig, RelPose* pose)
{
    return false;
}

XMLTag* HClusterDetector::Save()
{
    XMLTag* tag = new XMLTag(XML_NODE_HCLUSTERDETECTOR);
    tag->AddProperty(XML_ATTRIBUTE_SR4LO, m_swissranger_jlo_id);
    tag->AddProperty(XML_ATTRIBUTE_PTULO, m_ptu_jlo_id);
    return tag;
}

