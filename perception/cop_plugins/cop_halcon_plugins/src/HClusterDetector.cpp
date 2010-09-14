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

#define XML_ATTRIBUTE_TABLENOISE      "table_noise"
#define XML_ATTRIBUTE_OBJECTSPLITSIZE "object_split_size"
#define XML_ATTRIBUTE_TABLEPERCENTAGE "table_percentage"

using namespace cop;

using namespace std;
using namespace ros;
//using namespace std_msgs;
using namespace sensor_msgs;
using namespace geometry_msgs;
#include <cpp/HalconCpp.h>





#ifndef NORM2
#define NORM2(A,B,C) sqrt((A)*(A)+(B)*(B)+(C)*(C))
#endif


void add_triple_tuple_to_pcd(const Halcon::HTuple &X, const Halcon::HTuple &Y, const Halcon::HTuple &Z, PointCloud &pcd)
{
  for(int i = 0; i < X.Num(); i++)
  {
    Point32 p;
    p.x = X[i].D();
    p.y = Y[i].D();
    p.z = Z[i].D();
    pcd.points.push_back(p);
  }
}


PointCloud triple_tuple_to_pcd(const Halcon::HTuple &X, const Halcon::HTuple &Y, const Halcon::HTuple &Z)
{
  PointCloud ret;
  for(int i = 0; i < X.Num(); i++)
  {
    Point32 p;
    p.x = X[i].D();
    p.y = Y[i].D();
    p.z = Z[i].D();
    ret.points.push_back(p);
  }
  return ret;
}

void update_cov_with_pcd(Halcon::HTuple *cov33, const int &index_start, const PointCloud &pcd, double &mean_x, double &mean_y, double &mean_z)
{
  using namespace Halcon;
  Point32 p;
  HTuple Xreg, Yreg, Zreg, mean_x_inner, mean_y_inner, mean_z_inner, x_off, y_off, z_off;

  for(size_t i = 0; i < pcd.points.size(); i++)
  {
    p = pcd.points[i];
    Xreg[i] = p.x;
    Yreg[i] = p.y;
    Zreg[i] = p.z;
  }

  tuple_mean(Xreg, &mean_x_inner);
  tuple_mean(Yreg, &mean_y_inner);
  tuple_mean(Zreg, &mean_z_inner);
  
  mean_x = mean_x_inner[0].D();
  mean_y = mean_y_inner[0].D();
  mean_z = mean_z_inner[0].D();
  
  z_off = Zreg - mean_z_inner;
  y_off = Yreg - mean_y_inner;
  x_off = Xreg - mean_x_inner;
  
  tuple_abs(x_off, &x_off);
  tuple_abs(y_off, &y_off);
  tuple_abs(z_off, &z_off);
  
  HTuple cov0,cov1, cov2,cov3,cov4,cov5,cov6,cov7,cov8,cov9, x_off_sorted, y_off_sorted, z_off_sorted, indices;
  tuple_sort_index(x_off, &x_off_sorted);
  tuple_select_range(x_off_sorted, HTuple(x_off_sorted.Num()*0.03).Int(), HTuple(x_off_sorted.Num()*0.97).Int(), &x_off_sorted);

  tuple_sort_index(y_off, &y_off_sorted);
  tuple_select_range(y_off_sorted, HTuple(y_off_sorted.Num()*0.03).Int(), HTuple(y_off_sorted.Num()*0.97).Int(), &y_off_sorted);

  tuple_sort_index(z_off, &z_off_sorted);
  tuple_select_range(z_off_sorted, HTuple(z_off_sorted.Num()*0.03).Int(), HTuple(z_off_sorted.Num()*0.97).Int(), &z_off_sorted);

  tuple_concat(x_off_sorted,y_off_sorted, &indices);
  tuple_concat(indices,z_off_sorted, &indices);

  tuple_select(x_off, indices, &x_off);
  tuple_select(y_off, indices, &y_off);
  tuple_select(z_off, indices, &z_off);

  //printf("Length before: %ld, length after: %ld\n", x_off_sorted.Num(), x_off.Num());

  tuple_sum(x_off*x_off, &cov0);
  tuple_sum(y_off*x_off, &cov1);
  tuple_sum(z_off*x_off, &cov2);
  tuple_sum(x_off*y_off, &cov3);
  tuple_sum(y_off*y_off, &cov4);
  tuple_sum(z_off*y_off, &cov5);
  tuple_sum(x_off*z_off, &cov6);
  tuple_sum(y_off*z_off, &cov7);
  tuple_sum(z_off*z_off, &cov8);
  /** compensate for the removed points*/
  (*cov33)[index_start*9 + 0] = sqrt(cov0[0].D() / x_off.Num() * 1.1);
  (*cov33)[index_start*9 + 1] = sqrt(cov1[0].D() / x_off.Num() * 1.1);
  (*cov33)[index_start*9 + 2] = sqrt(cov2[0].D() / x_off.Num() * 1.1);
  (*cov33)[index_start*9 + 3] = sqrt(cov3[0].D() / x_off.Num() * 1.1);
  (*cov33)[index_start*9 + 4] = sqrt(cov4[0].D() / x_off.Num() * 1.1);
  (*cov33)[index_start*9 + 5] = sqrt(cov5[0].D() / x_off.Num() * 1.1);
  (*cov33)[index_start*9 + 6] = sqrt(cov6[0].D() / x_off.Num() * 1.1);
  (*cov33)[index_start*9 + 7] = sqrt(cov7[0].D() / x_off.Num() * 1.1);
  (*cov33)[index_start*9 + 8] = sqrt(cov8[0].D() / x_off.Num() * 1.1);

}




double SymmMahalanobisDistance(double mean_1_x, double mean_1_y, double mean_1_z, double cov1_xx, double cov1_xy, double cov1_xz,
                               double cov1_yx, double cov1_yy, double cov1_yz, double cov1_zx, double cov1_zy, double cov1_zz,
                               double mean_2_x, double mean_2_y, double mean_2_z, double cov2_xx, double cov2_xy, double cov2_xz,
                               double cov2_yx, double cov2_yy, double cov2_yz, double cov2_zx, double cov2_zy, double cov2_zz)
{
  double vectempx, vectempy, vectempz, dist1;
  vectempx = ((mean_2_x - mean_1_x) * cov1_xx + (mean_2_y - mean_1_y) * cov1_yx +(mean_2_z - mean_1_z) * cov1_zx);
  vectempy = ((mean_2_x - mean_1_x) * cov1_xy + (mean_2_y - mean_1_y) * cov1_yy +(mean_2_z - mean_1_z) * cov1_zy);
  vectempz = ((mean_2_x - mean_1_x) * cov1_xz + (mean_2_y - mean_1_y) * cov1_yz +(mean_2_z - mean_1_z) * cov1_zz);


  dist1 = sqrt(fabs(vectempx * (mean_2_x - mean_1_x) + vectempy * (mean_2_y - mean_1_y) + vectempz * (mean_2_z - mean_1_z)));

  vectempx = ((mean_1_x - mean_2_x) * cov2_xx + (mean_1_y - mean_2_y) * cov2_yx +(mean_1_z - mean_2_z) * cov2_zx);
  vectempy = ((mean_1_x - mean_2_x) * cov2_xy + (mean_1_y - mean_2_y) * cov2_yy +(mean_1_z - mean_2_z) * cov2_zy);
  vectempz = ((mean_1_x - mean_2_x) * cov2_xz + (mean_1_y - mean_2_y) * cov2_yz +(mean_1_z - mean_2_z) * cov2_zz);

  dist1 = (dist1 + sqrt(fabs(vectempx * (mean_2_x - mean_1_x) + vectempy * (mean_2_y - mean_1_y) + vectempz * (mean_2_z - mean_1_z)))) / 2;
  return dist1;
}

void splitDim(Halcon::Hobject Image, Halcon::Hobject Region, const Halcon::HTuple &Mean, double min_object_size_for_split, Halcon::HTuple &split_x, Halcon::HTuple &size_x)
{
   using namespace Halcon;
  HTuple histo_abs, histo, Min1, Max1, Range, Min, Max, Function, SmoothedFunction, Y1;

    gray_histo(Region, Image, &histo_abs, &histo);
    min_max_gray(Region, Image, 0, &Min1, &Max1, &Range);
    create_funct_1d_array(histo, &Function);
    smooth_funct_1d_gauss(Function, 2, &SmoothedFunction);
    smooth_funct_1d_gauss(SmoothedFunction, 2, &SmoothedFunction);
    smooth_funct_1d_gauss(SmoothedFunction, 2, &SmoothedFunction);
    smooth_funct_1d_gauss(SmoothedFunction, 2, &SmoothedFunction);

    local_min_max_funct_1d(SmoothedFunction, "strict_min_max", "true", &Min, &Max);
    get_y_value_funct_1d(SmoothedFunction, Max, "constant", &Y1);
    //printf("%ld maxima in split dimension\n", Y1.Num());
    if(Y1.Num() > 0)
    {
      double curr = (Min1+((HTuple(Max[0])*(Max1-Min1))/255))[0].D();
      split_x = curr;
      double curr_scale = Y1[0].D();
      //printf("Curr center in split dimension: %f\n", curr);
      for(int j = 0; j < Y1.Num() - 1; j++)
      {
        double next = (Min1+((HTuple(Max[j+1])*(Max1-Min1))/255))[0].D();
        if(fabs(curr - next) > min_object_size_for_split)
        {
          split_x[split_x.Num() - 1] = split_x[split_x.Num() - 1].D();
          split_x.Append(next);
          size_x.Append(curr_scale);
          curr_scale = Y1[j+1].D();
          //printf("Additional center in in split dimension: %f\n", next);
        }
        else
        {
          split_x[split_x.Num() - 1] = (split_x[split_x.Num() - 1].D() + next) / 2;
          curr_scale += Y1[j+1].D();

        }
        curr = next;
      }
      size_x.Append(curr_scale);
    }
    else
    {
      split_x = Mean;
      size_x = 1.0;
    }

}


void extract_clusters (Halcon::HTuple HomMat3d, Halcon::HTuple X, Halcon::HTuple Y,
    Halcon::HTuple Z, Halcon::HTuple CamParam, Halcon::HTuple *table_height, Halcon::HTuple *Mean_X,
    Halcon::HTuple *Mean_Y, Halcon::HTuple *Mean_Z, Halcon::HTuple *cov33, std::vector<sensor_msgs::PointCloud> &pcds,
    sensor_msgs::PointCloud &table_pcd,
    double table_noise = 0.01, double min_object_size_for_split = 0.05, double min_table_percentage = 0.05)
{
  using namespace Halcon;
  //printf("inside \n");
  // Local iconic variables
  Hobject  ImageZ, ImageX, ImageY, Region;
  Hobject  Region1, RegionDifference1, RegionClosing, ConnectedRegions;
  Hobject  SelectedRegions, ObjectSelected;


  // Local control variables
  HTuple  Qxt, Qyt, Qzt,Qx, Qy, Qz, Row, Column, AbsoluteHisto, SizeCluster;
  HTuple  RelativeHisto, Min1, Max1, Range, Function, SmoothedFunction;
  HTuple  Min, Max, Y1, Index, i, thres, start, Or, Number;
  HTuple  index, Rows, Columns, Xreg, Yreg, Zreg, MeanX, MeanY;
  HTuple  MeanZ, Deviation, DeviationY, DeviationX;
  HTuple  Row_add, Row_sgn, Index_R, Col_add, Col_sgn, Index_C;
  //printf("Length of X: %ld\n", X.Num());
  affine_trans_point_3d(HomMat3d, X, Y, Z, &Qx, &Qy, &Qz);

  //printf("CamParam: %f %f %f %f %f %f\n", CamParam[0].D(),CamParam[1].D(),CamParam[2].D(),CamParam[3].D(),CamParam[4].D(),CamParam[5].D());
  try
  {
    HTuple zeros;
    tuple_find(Z, HTuple(0.0), &zeros);
    if(!(zeros.Num() == 1 && zeros[0].I() == -1))
    {
      printf("Found %ld zeros\n", zeros.Num());
      for(int i = 0; i  < zeros.Num(); i++)
      {
        Z[zeros[i].I()] = 0.0000001;
      }
    }
    project_3d_point(X, Y, Z, CamParam, &Row, &Column);
  }
  catch(HException ex)
  {
    printf("Fix failed: %s but s_x= %f s_y =%f\n", ex.message, CamParam[2].D(), CamParam[3].D()  );
  }

  gen_image_const(&ImageZ, "float", CamParam[6].I() + 1, CamParam[7].I() + 1);
  gen_image_const(&ImageX, "float", CamParam[6].I() + 1, CamParam[7].I() + 1);
  gen_image_const(&ImageY, "float", CamParam[6].I() + 1, CamParam[7].I() + 1);

  //clear tuple from Rows and Columns that are too big or below 0
  tuple_add(Row, 1, &Row_add);				//all rows are at least 0 + 1
  tuple_sgn(Row_add, &Row_sgn); 			//all rows > 0 become 1
  tuple_find(Row_sgn, 1, &Index_R); 		//find all rows > 0
  tuple_select(Row, Index_R, &Row); 		//select all rows > 0
  tuple_select(Column, Index_R, &Column);
  tuple_select(Qx, Index_R, &Qx);
  tuple_select(Qy, Index_R, &Qy);
  tuple_select(Qz, Index_R, &Qz);

  tuple_add(Column, 1, &Col_add); 			//all cols are at least 0 + 1
  tuple_sgn(Col_add, &Col_sgn); 			//all cols > 0 become 1
  tuple_find(Col_sgn, 1, &Index_C);			//find all cols > 0
  tuple_select(Column, Index_C, &Column);	//select all cols > 0
  tuple_select(Row, Index_C, &Row);
  tuple_select(Qx, Index_C, &Qx);
  tuple_select(Qy, Index_C, &Qy);
  tuple_select(Qz, Index_C, &Qz);

  tuple_sub(Row, CamParam[7].I(), &Row_add);	//all rows from 0-143 are < 0
  tuple_sgn(Row_add, &Row_sgn);					//all rows >= 0 <= 143 become -1
  tuple_find(Row_sgn, -1, &Index_R);			//select all rows > 0
  tuple_select(Row, Index_R, &Row);				//select all rows >= 0 <= 143
  tuple_select(Column, Index_R, &Column);
  tuple_select(Qx, Index_R, &Qx);
  tuple_select(Qy, Index_R, &Qy);
  tuple_select(Qz, Index_R, &Qz);

  tuple_sub(Column, CamParam[6].I(), &Col_add);	//all cols from 0-175 are < 0
  tuple_sgn(Col_add, &Col_sgn);						//all cols >= 0 <= 175 become -1
  tuple_find(Col_sgn, -1, &Index_C);				//select all cols > 0
  tuple_select(Column, Index_C, &Column);			//select all cols >= 0 <= 175
  tuple_select(Row, Index_C, &Row);
  tuple_select(Qx, Index_C, &Qx);
  tuple_select(Qy, Index_C, &Qy);
  tuple_select(Qz, Index_C, &Qz);


  HTuple rowcollect , colcollect;
  try
  {

    set_grayval(ImageX, Row.Int(), Column.Int(), Qx);
    set_grayval(ImageY, Row.Int(), Column.Int(), Qy);
    //
    set_grayval(ImageZ, Row.Int(), Column.Int(), Qz);
    gen_region_points(&Region, Row.Int(), Column.Int());
  }
  catch(HException ex)
  {
    printf("got an exception trying directly %s\n", ex.message);
    for(int i = 0; i < Row.Num(); i++)
    {
      if(Row[i].I() >= 0 && Column[i].I() >= 0 &&
         Row[i].I() < CamParam[7].I() + 1 && Column[i].I() < CamParam[6].I() + 1)
      {
          set_grayval(ImageX, Row[i].I(), Column[i].I(), Qx[i]);
          set_grayval(ImageY, Row[i].I(), Column[i].I(), Qy[i]);
          set_grayval(ImageZ, Row[i].I(), Column[i].I(), Qz[i]);
          tuple_concat(rowcollect, Row[i], &rowcollect);
          tuple_concat(colcollect, Column[i], &colcollect);
      }
      else
        printf("Error at: r %d c %d \n x %f y %f  z  %f\n Cam: %d %d (%ld)\n", Row[i].I(), Column[i].I(), X[i].D(), Y[i].D(), Z[i].D(), CamParam[6].I(),CamParam[7].I(), CamParam.Num());
    }
    gen_region_points(&Region, rowcollect.Int(), colcollect.Int());
  }


  //
  //threshold (ImageZ, Region, -0.3, 2.0)
  gray_histo(Region, ImageZ, &AbsoluteHisto, &RelativeHisto);
  min_max_gray(Region, ImageZ, 0, &Min1, &Max1, &Range);
  //printf("Min-max: %f - %f\n",Min1[0].D(), Max1[0].D());
  create_funct_1d_array(RelativeHisto, &Function);
  smooth_funct_1d_gauss(Function, 2, &SmoothedFunction);

  local_min_max_funct_1d(SmoothedFunction, "strict_min_max", "true", &Min, &Max);
  get_y_value_funct_1d(SmoothedFunction, Max, "constant", &Y1);
  (*table_height) = HTuple();
  HTuple max_y1;
  tuple_sort_index(Y1, &max_y1);
  (*table_height) = (Min1+((HTuple(Max[ max_y1[max_y1.Num() - 1].I() ])*(Max1-Min1))/255));

   if( Y1.Num() > 4 && (*table_height)[0].D() < 0.5)
      (*table_height) = (Min1+((HTuple(Max[max_y1[max_y1.Num() - 2].I() ])*(Max1-Min1))/255));

  printf("table height estimated at z=%f\n", (*table_height)[0].D());
  if((*table_height)[0].D() > 1.5)
  {
     printf("Ignored\n");
     return;
  }

  (*Mean_X) = HTuple();
  (*Mean_Y) = HTuple();
  (*Mean_Z) = HTuple();
  (*cov33) = HTuple();
  //printf("number of maxima: %ld\n", (*table_height).Num());
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
    start = HTuple((*table_height)[i])+table_noise;
    threshold(ImageZ, &Region1, start, thres);

    /** Extract table points*/
    Hobject RegionTable;
    HTuple rowt, colt, xt, yt, zt;
    threshold(ImageZ, &RegionTable,  HTuple((*table_height)[i])-table_noise, HTuple((*table_height)[i])+table_noise -0.001);
    get_region_points(RegionTable, &rowt, &colt);
    get_grayval(ImageX, rowt, colt, &xt);
    get_grayval(ImageY, rowt, colt, &yt);
    get_grayval(ImageZ, rowt, colt, &zt);
    table_pcd = triple_tuple_to_pcd(xt, yt, zt);

    intersection(Region1, Region, &RegionDifference1);
    closing_circle(RegionDifference1, &RegionClosing, 5);
    connection(RegionClosing, &ConnectedRegions);
    select_shape(ConnectedRegions, &SelectedRegions, "area", "and", X.Num() / 400, 99999);
    tuple_or(HTuple((*table_height)[i])>0.5, ((*table_height).Num())==1, &Or);
    if (0 != Or)
    {
      count_obj(SelectedRegions, &Number);
      //printf("\n\n%d regions to search for clusters\n\n", Number[0].I());
      for (index=1; index<=Number[0].I(); index+=1)
      {
        //printf("enter region %d\n", index[0].I());
        select_obj(SelectedRegions, &ObjectSelected, index);
        get_region_points(ObjectSelected, &Rows, &Columns);
        get_grayval(ImageY, Rows, Columns, &Yreg);
        if(Yreg.Num() < 2)
          continue;

        tuple_mean(Yreg, &MeanY);

        double cov[9];
        memset(cov, 0.0, sizeof(*cov)*9);

        HTuple max, histo, histo_abs, split_y, size_y;

        tuple_max(Yreg - MeanY, &max);
        if(max > 2*min_object_size_for_split)
        {
          //printf("Trying split in Y\n");
          splitDim(ImageY, ObjectSelected, MeanY, min_object_size_for_split, split_y, size_y);
        }
        else
          split_y = MeanY;\
        for(int ysplit = 0; ysplit < split_y.Num(); ysplit++)
        {
          HTuple RowY, ColY;
          double lower_bound = -1000.0;
          double upper_bound = 1000.0;

          if(ysplit != split_y.Num() -1)
          {
           upper_bound = (split_y[ysplit + 1] * size_y[ysplit + 1] +
                                  split_y[ysplit] * size_y[ysplit])
                                  / (size_y[ysplit + 1] + size_y[ysplit]);
          }
          if(ysplit != 0)
          {
            lower_bound =  (split_y[ysplit] * size_y[ysplit] +
                                                 split_y[ysplit - 1] * size_y[ysplit - 1])
                                              / (size_y[ysplit - 1] + size_y[ysplit]) ;
          }
          for (int rows = 0; rows < Rows.Num(); rows++)
          {
            if( lower_bound < Yreg[rows].D() &&  Yreg[rows].D() <  upper_bound && Yreg[rows].D() != 0.0)
            {
              RowY.Append(Rows[rows]);
              ColY.Append(Columns[rows]);
            }
          }
          //printf("ysplit %d left %ld / %ld\n", ysplit, RowY.Num(), Rows.Num());
          //printf("decision y: ");
          //printf( "%f < y < %f\n", lower_bound, upper_bound);
          HTuple split_x, size_x;
          HTuple x_off, RowXY, ColXY;
          if(RowY.Num() < 2)
          {
            continue;
          }
          get_grayval(ImageX, RowY, ColY, &Xreg);
          tuple_mean(Xreg, &MeanX);
          tuple_max(Xreg - MeanX, &max);

          if(max > 2*min_object_size_for_split)
          {
            //printf("Trying split in X\n");
            splitDim(ImageX, ObjectSelected, MeanX, min_object_size_for_split, split_x, size_x);
          }
          else
            split_x = MeanY;
          for(int xsplit = 0; xsplit < split_x.Num(); xsplit++)
          {
            RowXY = HTuple();
            ColXY = HTuple();
            double lower_bound_x = -1000.0;
            double upper_bound_x = 1000.0;
            if(xsplit != split_x.Num() -1)
            {
             upper_bound_x = (split_x[xsplit + 1] * size_x[xsplit + 1] +
                                    split_x[xsplit] * size_x[xsplit])
                                    / (size_x[xsplit + 1] + size_x[xsplit]);
            }
            if(xsplit != 0)
            {
              lower_bound_x =  (split_x[xsplit] * size_x[xsplit] +
                                                   split_x[xsplit - 1] * size_x[xsplit - 1])
                                                / (size_x[xsplit - 1] + size_x[xsplit]) ;
            }


            for (int rows = 0; rows < RowY.Num(); rows++)
            {
              if(lower_bound_x < Xreg[rows].D() && Xreg[rows].D() < upper_bound_x && Xreg[rows].D() != 0.0)
              {
                RowXY.Append(RowY[rows]);
                ColXY.Append(ColY[rows]);
              }
            }
            //printf("xsplit: %d () xsplit: %d size: %ld / %ld\n", xsplit, xsplit, RowXY.Num(),  RowY.Num());
            //printf("decision x: ");
            //printf( "%f < x < %f\n",lower_bound_x,upper_bound_x);

            HTuple YregInner, XregInner, z_off, x_off, y_off, mean_x_inner, mean_y_inner;

            if(RowXY.Num() < 2)
              continue;
            get_grayval(ImageX, RowXY, ColXY, &XregInner);
            get_grayval(ImageY, RowXY, ColXY, &YregInner);
            get_grayval(ImageZ, RowXY, ColXY, &Zreg);

            tuple_mean(XregInner, &mean_x_inner);
            tuple_mean(YregInner, &mean_y_inner);
            tuple_mean(Zreg, &MeanZ);

            z_off = Zreg - MeanZ;
            y_off = YregInner - mean_y_inner;
            x_off = XregInner - mean_x_inner;
            tuple_abs(x_off, &x_off);
            tuple_abs(y_off, &y_off);
            tuple_abs(z_off, &z_off);
            HTuple cov0,cov1, cov2,cov3,cov4,cov5,cov6,cov7,cov8,cov9, x_off_sorted, y_off_sorted, z_off_sorted, indices;
            tuple_sort_index(x_off, &x_off_sorted);
            tuple_select_range(x_off_sorted, HTuple(x_off_sorted.Num()*0.03).Int(), HTuple(x_off_sorted.Num()*0.97).Int(), &x_off_sorted);

            tuple_sort_index(y_off, &y_off_sorted);
            tuple_select_range(y_off_sorted, HTuple(y_off_sorted.Num()*0.03).Int(), HTuple(y_off_sorted.Num()*0.97).Int(), &y_off_sorted);

            tuple_sort_index(z_off, &z_off_sorted);
            tuple_select_range(z_off_sorted, HTuple(z_off_sorted.Num()*0.03).Int(), HTuple(z_off_sorted.Num()*0.97).Int(), &z_off_sorted);

            tuple_concat(x_off_sorted,y_off_sorted, &indices);
            tuple_concat(indices,z_off_sorted, &indices);

            tuple_select(x_off, indices, &x_off);
            tuple_select(y_off, indices, &y_off);
            tuple_select(z_off, indices, &z_off);

            //printf("Length before: %ld, length after: %ld\n", x_off_sorted.Num(), x_off.Num());

            tuple_sum(x_off*x_off, &cov0);
            tuple_sum(y_off*x_off, &cov1);
            tuple_sum(z_off*x_off, &cov2);
            tuple_sum(x_off*y_off, &cov3);
            tuple_sum(y_off*y_off, &cov4);
            tuple_sum(z_off*y_off, &cov5);
            tuple_sum(x_off*z_off, &cov6);
            tuple_sum(y_off*z_off, &cov7);
            tuple_sum(z_off*z_off, &cov8);
            /*compensate for the removed extreme points*/
            cov0 = sqrt(cov0[0].D() / x_off.Num() * 1.1);
            cov1 = sqrt(cov1[0].D() / x_off.Num() * 1.1);
            cov2 = sqrt(cov2[0].D() / x_off.Num() * 1.1);
            cov3 = sqrt(cov3[0].D() / x_off.Num() * 1.1);
            cov4 = sqrt(cov4[0].D() / x_off.Num() * 1.1);
            cov5 = sqrt(cov5[0].D() / x_off.Num() * 1.1);
            cov6 = sqrt(cov6[0].D() / x_off.Num() * 1.1);
            cov7 = sqrt(cov7[0].D() / x_off.Num() * 1.1);
            cov8 = sqrt(cov8[0].D() / x_off.Num() * 1.1);

            /** Test if there is already a similar cluster
                This can happen if the axis aligned split cut an object in half
            */
            bool rejected = false;
            for(int test_near = 0; test_near < (*Mean_X).Num(); test_near++)
            {

              double malha_dist = SymmMahalanobisDistance((*Mean_X)[test_near].D(), (*Mean_Y)[test_near].D() , (*Mean_Z)[test_near].D() , (*cov33)[test_near*9].D(),
                                                (*cov33)[test_near*9 + 1].D(), (*cov33)[test_near*9 + 2].D(), (*cov33)[test_near*9 + 3].D(), (*cov33)[test_near*9 + 4 ].D(), (*cov33)[test_near*9 + 5].D(),
                                                (*cov33)[test_near*9 + 6].D(), (*cov33)[test_near*9 + 7].D(), (*cov33)[test_near*9 + 8].D(),
                                                mean_x_inner[0].D(),  mean_y_inner[0].D(),  MeanZ[0].D(), cov0[0].D(), cov1[0].D(), cov2[0].D(), cov3[0].D(), cov4[0].D(),
                                                cov5[0].D(), cov6[0].D(), cov7[0].D(), cov8[0].D());

            double dist = NORM2(((*Mean_X)[test_near].D() - mean_x_inner[0].D()),
                       ((*Mean_Y)[test_near].D() - mean_y_inner[0].D()),
                      ((*Mean_Z)[test_near].D() - MeanZ[0].D()));
             /*printf("distance between two Cluster mahl_dist: %f dist: %f\n", malha_dist, dist);
             2*min_object_size_for_split
             */

             if( malha_dist < 0.01 || dist < 2*min_object_size_for_split)
              {
                SizeCluster[test_near] = SizeCluster[test_near].I() + x_off.Num();

                add_triple_tuple_to_pcd(XregInner, YregInner, Zreg, pcds[test_near]);
                double meanx, meany, meanz;
                update_cov_with_pcd(cov33, test_near, pcds[test_near], meanx, meany, meanz);
                (*Mean_X)[test_near] = meanx;
                (*Mean_Y)[test_near] = meany;
                (*Mean_Z)[test_near] = meanz;
                //printf("Fused two clusters, which were splitted before\n");
                rejected = true;
                break;
              }
              /*printf("Accept distance %f between %d and new mean (%f %f %f)\n", NORM2(((*Mean_X)[test_near].D() - mean_x_inner[0].D()),
                                   ((*Mean_Y)[test_near].D() - mean_y_inner[0].D()),
                                                       ((*Mean_Z)[test_near].D() - MeanZ[0].D())),
                                                       test_near, mean_x_inner[0].D(),
                                                       mean_y_inner[0].D(),  MeanZ[0].D());*/
            }



            if(rejected)
              continue;
            if(XregInner.Num() < X.Num() / 500)
            {
              //printf("Reject this region as too small (2nd split)\n");
              continue;
            }
            /** density in particle per cubic meter*/
            double dens_threshold = 5000000, density;


            density = XregInner.Num() / (cov0 * cov4 * cov8)[0].D();
            printf("density is %f (covs: %f %f %f., Num: %ld) (mean: %f %f %f)\n", density, cov0[0].D(), cov4[0].D(), cov8[0].D(), x_off.Num(), mean_x_inner[0].D(), mean_y_inner[0].D(), MeanZ[0].D());
            if(density < dens_threshold)
            {
              printf("Rejecting Cluster due  low density\n");
              continue;
            }
            cov33->Append(cov0);
            cov33->Append(cov1);
            cov33->Append(cov2);

            cov33->Append(cov3);
            cov33->Append(cov4);
            cov33->Append(cov5);

            cov33->Append(cov6);
            cov33->Append(cov7);
            cov33->Append(cov8);


            (*Mean_X).Append(mean_x_inner);
            (*Mean_Y).Append(mean_y_inner);
            (*Mean_Z).Append(MeanZ);
            pcds.push_back(triple_tuple_to_pcd(XregInner, YregInner, Zreg));
            SizeCluster.Append(x_off.Num());
          }
        }
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
    //printf("Read camparam: %s\n", camparam.c_str());
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
          //printf("Read HClusterDetector with %ld as camera position (%s)\n", pose->m_uniqueID, name.c_str());
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
      m_table_noise = tag->GetPropertyDouble(XML_ATTRIBUTE_TABLENOISE, 0.02);
      m_min_object_size_for_split = tag->GetPropertyDouble(XML_ATTRIBUTE_OBJECTSPLITSIZE, 0.05);
      m_min_table_percentage = tag->GetPropertyDouble(XML_ATTRIBUTE_TABLEPERCENTAGE, 0.05);
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
        results = Inner(*it, pose, proto, numOfObjects, qualityMeasure);
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



std::vector<RelPose*> HClusterDetector::Inner(Sensor* sens, RelPose* initial_pose_estimate, SegmentPrototype* obj_descr, int &numOfObjects, double& qualityMeasure)
{
  std::vector<RelPose*> results;
  //printf("Inner\n");
  double threshold = qualityMeasure;
  obj_descr->ClearPointClouds();
  if(threshold < 0.1 || threshold > 1.0)
    threshold  = 0.0;
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
    std::vector<sensor_msgs::PointCloud> segment_pcds;
    sensor_msgs::PointCloud table_pcd;
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

    //printf("Before extract cluster\n");
    extract_clusters(HomMat3d, X, Y, Z, CamParam, &table_height, &Mean_X, &Mean_Y, &Mean_Z, &cov33, segment_pcds,
            table_pcd,
            m_table_noise, m_min_object_size_for_split, m_min_table_percentage);
    //printf("After extract cluster\n");

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
      cov << (cov33[9*x+0].D()) << (cov33[9*x+1].D()) << (cov33[9*x+2].D()) << 0   << 0   << 0
          << (cov33[9*x+3].D()) << (cov33[9*x+4].D()) << (cov33[9*x+5].D()) <<  0  << 0   << 0
          << (cov33[9*x+6].D()) << (cov33[9*x+7].D()) << (cov33[9*x+8].D()) << 0   << 0   << 0
          <<0    << 0    << 0    <<  ((obj_descr == NULL) ? 0.2 : obj_descr->m_covRotX) << 0   << 0
          <<0    << 0    << 0    << 0   << ((obj_descr == NULL) ? 0.2 :obj_descr->m_covRotY) << 0
          <<0    << 0    << 0    << 0   << 0   << ((obj_descr == NULL) ? 0.8 :obj_descr->m_covRotZ);
     }
     else
     {
       cov << (cov33[9*x+3].D()) << (cov33[9*x+4].D()) << (cov33[9*x+5].D()) << 0  << 0   << 0
          << (cov33[9*x+0].D()) << (cov33[9*x+1].D()) << (cov33[9*x+2].D()) << 0  << 0   << 0
          << (cov33[9*x+6].D()) << (cov33[9*x+7].D()) << (cov33[9*x+8].D()) << 0   << 0   << 0
          <<0    << 0    << 0    << ((obj_descr == NULL) ? 0.2 :obj_descr->m_covRotY) << 0   << 0
          <<0    << 0    << 0    << 0   << ((obj_descr == NULL) ? 0.2 :obj_descr->m_covRotX) << 0
          <<0    << 0    << 0    << 0   << 0   << ((obj_descr == NULL) ? 0.8 :obj_descr->m_covRotZ);
      }

      cout << "Cov from pc: "<< endl <<  cov << endl;
      RelPose* pose_temp = RelPoseFactory::FRelPose(ptu, rotmat, cov);
      double temp_qual = min(1.0, max(0.0, fabs( 2*cov33[9*x+4].D()*2*cov33[9*x+0].D()*2*cov33[9*x+8].D())));
      if(x == 0)
       qualityMeasure =temp_qual;

      if(pose_temp == NULL)
        continue;
      pose_temp->m_qualityMeasure = temp_qual;
      double corr_prob = initial_pose_estimate->ProbabilityOfCorrespondence(pose_temp);
      if(corr_prob >= threshold || threshold == 0.0)
      {
        printf("Result: %f\n", corr_prob );
        results.push_back(pose_temp);
        obj_descr->SetPointCloud(pose_temp->m_uniqueID, segment_pcds[x]);
      }
      else
        printf("printf rejecting pose caused by threshold: %f < %f \n", corr_prob, threshold);
    }
    /**
      Add Table
    */
    if(results.size() > 0 && table_pcd.points.size() > 0)
    {
      Halcon::HTuple covtable;
      double  meanx, meany, meanz;
      update_cov_with_pcd(&covtable, 0, table_pcd, meanx, meany, meanz);
      Matrix rotmat(4,4);
      rotmat << 1 << 0<< 0  << meanx
             << 0 << 1 << 0 << meany
             << 0 << 0<< 1  << meanz
             << 0 << 0 << 0 << 1;

        cout <<  "Matrix from plane_clusters:" << endl << rotmat << endl;
        Matrix cov (6,6);
        cov << (covtable[0].D())  << (cov33[1].D()) << (cov33[2].D()) << 0   << 0   << 0
           << (covtable[3].D())   << (cov33[4].D()) << (cov33[5].D()) <<  0  << 0   << 0
           << (covtable[6].D())   << (cov33[7].D()) << (cov33[8].D()) << 0   << 0   << 0
           <<0    << 0    << 0    <<   0 << 0   << 0
           <<0    << 0    << 0    << 0   << 0   << 0
           <<0    << 0    << 0    << 0   << 0   << 0;
      RelPose* pose_temp = RelPoseFactory::FRelPose(ptu, rotmat, cov);
      obj_descr->SetTable(pose_temp->m_uniqueID, table_pcd);
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
    tag->AddProperty(XML_ATTRIBUTE_TABLENOISE, m_table_noise);
    tag->AddProperty(XML_ATTRIBUTE_OBJECTSPLITSIZE, m_min_object_size_for_split);
    tag->AddProperty(XML_ATTRIBUTE_TABLEPERCENTAGE, m_min_table_percentage);
    return tag;
}

