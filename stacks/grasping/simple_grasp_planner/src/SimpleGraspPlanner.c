/*
 * Copyright (c) 2009-2010, U. Klank
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "SimpleGraspPlanner.h"
#include <math.h>
#include <stdio.h>


typedef struct
{
  Point3D* obstacle_list;
  CovariancePoint* obstacle_covs;
  int num_obstacles;
  Point3D* hand_points;
  int num_hand_points;  double max_finger;
  double palm_width;
  double table_z;
}
Dataset;

Dataset g_data;
#define min(A,B) (((A) < (B)) ? (A) : (B))
#define max(A,B) (((A) > (B)) ? (A) : (B))

/**
  Initialize the hand configuration
*/
int InitSGP(Point3D* hand_points, int num, double table_z)
{
  int iter_hand_point;
  g_data.hand_points = hand_points;
  g_data.num_hand_points = num;
  g_data.max_finger = 0.0;
  g_data.palm_width = 0.0;

  printf("Hand Points:\n");
  for(iter_hand_point = 0; iter_hand_point < num; iter_hand_point++)
  {
    /** Get biggest hand z (first collision coming from top with a table)*/
    if(g_data.max_finger < hand_points[iter_hand_point].z)
      g_data.max_finger = hand_points[iter_hand_point].z;
    /** Get smallest hand y (first collision point with a table coming from the side)*/
    if(g_data.palm_width < fabs(hand_points[iter_hand_point].y))
      g_data.palm_width = fabs(hand_points[iter_hand_point].y);
    printf("%f %f %f\n", hand_points[iter_hand_point].x, hand_points[iter_hand_point].y, hand_points[iter_hand_point].z);
  }
  g_data.table_z = table_z;
  /*printf("Max Finger Length: %f Palm Width %f Table %f\n", g_data.max_finger, g_data.palm_width,  g_data.table_z);*/

  return num;
}

void setTableHeight(double table_z)
{
  g_data.table_z = table_z;
}


double getTableHeight()
{
  return (g_data.table_z);
}


/**
   Determinant
        a           b           c
   double sx; double sxy; double sxz;
      d            e             f
    double syx; double sy; double szy;
      g             h            i
      double szx; double syz; double sz;

*/
double det(CovariancePoint cov)
{
  return cov.sx * cov.sy * cov.sz -
         cov.sx * cov.szy * cov.syz +
         cov.sxy * cov.szy * cov.szx -
         cov.sxy * cov.syx * cov.sz +
         cov.sxz * cov.syx * cov.syz -
         cov.sxz * cov.sy * cov.szx;
}
/**
 Approximation of the probability density
 Implementation of this formula:

 /%4 (cov_sy cov_sz - cov_syz cov_szy)   %3 (-cov_szx cov_syz + cov_syx cov_sz)
|------------------------------------ - --------------------------------------
\                 %2                                      %2

       %1 (cov_szx cov_sy - cov_syx cov_szy)\      /  %4 (cov_sxy cov_sz - cov_sxz cov_szy)
     - -------------------------------------| %4 + |- -------------------------------------
                        %2     1.504695             /      \                   %2

       %3 (-cov_szx cov_sxz + cov_sx cov_sz)   %1 (cov_szx cov_sxy - cov_sx cov_szy)\      /
     + ------------------------------------- + -------------------------------------| %3 + |
                        %2                                      %2                  /      \

    %4 (cov_sxy cov_syz - cov_sxz cov_sy)   %3 (-cov_syx cov_sxz + cov_sx cov_syz)
    ------------------------------------- - --------------------------------------
                     %2                                       %2

       %1 (-cov_syx cov_sxy + cov_sx cov_sy)\
     + -------------------------------------| %1
                        %2                  /

1% = dz := point_test_z - mean_z

2% = cov_bla := cov_szx cov_sxy cov_syz - cov_szx cov_sxz cov_sy - cov_syx cov_sxy cov_sz

     + cov_syx cov_sxz cov_szy + cov_sx cov_sy cov_sz - cov_sx cov_syz cov_szy

3% = dy := point_test_y - mean_y

4% = dx := point_test_x - mean_x

*/
double f(Point3D point_test, Point3D mean, CovariancePoint cov)
{
  double c_temp, expo, det_temp;
  double dx =  mean.x - point_test.x;
  double dy =  mean.y - point_test.y;
  double dz =  mean.z - point_test.z;
  double cov_bla = cov.szx * cov.sxy * cov.syz - cov.szx * cov.sxz *  cov.sy - cov.syx * cov.sxy * cov.sz
         + cov.syx * cov.sxz * cov.szy + cov.sx * cov.sy * cov.sz - cov.sx * cov.syz * cov.szy;
  if(cov_bla == 0.0)
  {
    cov.sx = 0.0000001;
    cov.sy = 0.0000001;
    cov.sz = 0.0000001;
    cov_bla = cov.szx * cov.sxy * cov.syz - cov.szx * cov.sxz *  cov.sy - cov.syx * cov.sxy * cov.sz
         + cov.syx * cov.sxz * cov.szy + cov.sx * cov.sy * cov.sz - cov.sx * cov.syz * cov.szy;
  }
  double t1 = (( dx * (cov.sy   * cov.sz  - cov.syz * cov.szy)  / cov_bla)-
    ( dy * (-cov.szx * cov.syz + cov.syx * cov.sz)   / cov_bla) -
    ( dz * (cov.szx  * cov.sy  - cov.syx * cov.szy)  / cov_bla)) * dx;
  double t2 =  ((-dx * (cov.sxy  * cov.sz  - cov.sxz * cov.szy)) / cov_bla +
    ( dy * (-cov.szx * cov.sxz + cov.sx  * cov.sz)   / cov_bla) +
    ( dz * (cov.szx  * cov.sxy - cov.sx  * cov.szy)  / cov_bla)) * dy;
  double t3 = (( dx * (cov.sxy  * cov.syz - cov.sxz * cov.sy)   / cov_bla) -
    ( dy * (-cov.syx * cov.sxz + cov.sx  * cov.syz)  / cov_bla) +
    ( dz * (-cov.syx * cov.sxy + cov.sx  * cov.sy)   / cov_bla)) * dz;


  /*printf("cov_bla %f, dx %f dy %f dz %f\n",cov_bla,  dx, dy, dz);*/
  det_temp = det(cov);
  if(det_temp == 0)
    det_temp = 0.00000001;
  c_temp = 1.0 /( pow(2.0 * M_PI, 1.5) * sqrt(fabs(det_temp)));
  /*printf("t1 %f ,t2 %f, t3 %f, cov_bla = %f\n", t1, t2, t3, cov_bla);*/

  expo = (-1.0/2.0) * (t1 + t2 + t3);

  /*printf("expo = %f\n", expo);*/

  c_temp *= exp(expo);
  /*printf("ctemp = %f\n", c_temp);*/
  return c_temp;
}


Point3D TransformPoint(Point3D hand, Point3D target, double alpha, double beta, double delta)
{
  Point3D newp;
  /** new poitions:
      Point P in object coordinates depending on the parameters alpha, beta and delta
  */
  newp.x = target.x + (delta - hand.z) * cos(alpha) * sin(beta) - cos(beta) * hand.x - sin(alpha) * sin(beta) * hand.y;
  newp.y = target.y - (delta - hand.z) * cos(alpha) * cos(beta) - sin(beta) * hand.x + sin(alpha) * cos(beta) * hand.y;
  newp.z = target.z + (delta - hand.z) * sin(alpha) + cos(alpha) * hand.y;
  /*printf("Pnew %f %f %f\n",  newp.x, newp.y, newp.z);*/
  return newp;
}


#define MAX_HAND_POINTS 100
HandConfig GetGraspLM(Point3D* obstacle_list, CovariancePoint* obstacle_covs, int num,
                      double offset_rot_z_side, double offset_rot_z_top)
{
  HandConfig ret;
  Point3D hand_center;
  double delta_delta, delta_max, delta_min, delta_beta, beta, delta, min_score, score, alpha, delta_alpha, alpha_min, beta_max, beta_loop;
  double min_grasp_prop = 60.0;
  int iter_obstacle, iter_hand_point;
  Point3D hand_points_trans[MAX_HAND_POINTS];
  if(g_data.num_hand_points > MAX_HAND_POINTS)
  {
    printf("Error: too many hand points passed\n");
    g_data.num_hand_points = MAX_HAND_POINTS;
  }
  /** Initialize hand_center*/
  hand_center.x = 0.0;
  hand_center.y = 0.0;
  hand_center.z = 0.0;
  /** Initialize lists*/
  ret.alpha = 0.0;
  ret.beta = 0.0;
  ret.delta_max = 0.0;
  g_data.obstacle_list = obstacle_list;
  g_data.obstacle_covs = obstacle_covs;
  g_data.num_obstacles = num;


  printf("Obstacle List\n");

  for(iter_obstacle = 0; iter_obstacle < g_data.num_obstacles; iter_obstacle++)
  {
    Point3D obs = g_data.obstacle_list[iter_obstacle];
    CovariancePoint cov = g_data.obstacle_covs[iter_obstacle];
    printf("Obs: %05.2f,%05.2f,%05.2f\n", obs.x, obs.y, obs.z);
    printf(" Cov: %05.2f,%05.2f,%05.2f\n%05.2f,%05.2f,%05.2f\n%05.2f,%05.2f,%05.2f\n", cov.sx, cov.sxy, cov.sxz, cov.syx, cov.sy, cov.syz, cov.szx, cov.szy, cov.sz);
  }




  /** Initialize search space + sampling*/
  delta_max = (g_data.max_finger+ (fabs(g_data.obstacle_covs[0].sx) + fabs(g_data.obstacle_covs[0].sy) + fabs(g_data.obstacle_covs[0].sz)) /3.0);

  /**     Consider Table collision*/
  if(fabs(g_data.obstacle_list[0].z - g_data.table_z) <  g_data.palm_width )
  {
     printf("Table collision. Limiting alpha to pi/2\n");
     alpha_min = M_PI / 2.0;
     delta_min = g_data.max_finger - (g_data.obstacle_list[0].z - g_data.table_z);
     if(delta_max < delta_min)
      delta_max = delta_min + fabs(g_data.obstacle_covs[0].sz);
  }
  else
  {
    printf("table is safe : %f (=obj_z - table) >= %f (=palm_width)\n", fabs(g_data.obstacle_list[0].z - g_data.table_z),  g_data.palm_width);
    alpha_min = 0.0;
    delta_min = min(fabs(g_data.obstacle_covs[0].sx), min( fabs(g_data.obstacle_covs[0].sy) ,fabs(g_data.obstacle_covs[0].sz)))/2.0;
  }
  /*printf("Delta Max: %f\n", delta_max);*/
  delta_delta = (delta_max - delta_min) / 20.0;
  delta_beta = 0.0872 * M_PI;
  delta_alpha = M_PI/2.0;
  beta_max=M_PI/2.0;

  /** Initialize start score: careful for strange covariances, this value might be to low*/
  min_score = 100000.0;
  for(beta_loop = 0; beta_loop < beta_max; beta_loop += delta_beta)
  {
    for(alpha = alpha_min; alpha < M_PI / 2.0 +0.1; alpha += delta_alpha)
    {
      double score_dmax = 0.0;
      if(alpha == 0)
      {
        beta = beta_loop + offset_rot_z_side;
      }
      else
      {
        beta = beta_loop +  offset_rot_z_top;
      }
      for(iter_hand_point = 0; iter_hand_point < g_data.num_hand_points; iter_hand_point++)
      {
        hand_points_trans[iter_hand_point] = TransformPoint(g_data.hand_points[iter_hand_point], g_data.obstacle_list[0], alpha, beta, delta_max);
      }
      for(iter_obstacle = 0; iter_obstacle < g_data.num_obstacles; iter_obstacle++)
      {
        Point3D obs = g_data.obstacle_list[iter_obstacle];
        CovariancePoint cov = g_data.obstacle_covs[iter_obstacle];
        for(iter_hand_point = 0; iter_hand_point < g_data.num_hand_points; iter_hand_point++)
        {
          score_dmax += f( hand_points_trans[iter_hand_point], obs, cov);
        }
      }
      if(alpha <  M_PI / 4.0)
        delta_min = min(fabs(g_data.obstacle_covs[0].sx), fabs(g_data.obstacle_covs[0].sy))/2.0;
      else
      {
        delta_min = max(fabs(g_data.obstacle_covs[0].sz)/2.0, g_data.max_finger - (g_data.obstacle_list[0].z - g_data.table_z));
        printf("delta min including table: %f\n", delta_min);
      }
      delta_delta = (delta_max - delta_min) / 20.0;
      for(delta = delta_min; delta < delta_max; delta += delta_delta)
      {
        Point3D hand_center_trans = TransformPoint(hand_center, g_data.obstacle_list[0], alpha, beta, delta);
        score = score_dmax;

        /** generate configuration dependant hand points*/
        for(iter_hand_point = 0; iter_hand_point < g_data.num_hand_points; iter_hand_point++)
        {
          hand_points_trans[iter_hand_point] = TransformPoint(g_data.hand_points[iter_hand_point], g_data.obstacle_list[0], alpha, beta, delta);
        }
        for(iter_obstacle = 0; iter_obstacle < g_data.num_obstacles; iter_obstacle++)
        {
          Point3D obs = g_data.obstacle_list[iter_obstacle];
          CovariancePoint cov = g_data.obstacle_covs[iter_obstacle];
          for(iter_hand_point = 0; iter_hand_point < g_data.num_hand_points; iter_hand_point++)
          {
            score += f( hand_points_trans[iter_hand_point], obs, cov);
          }
        }

        score -= f( hand_center_trans , g_data.obstacle_list[0], g_data.obstacle_covs[0]) * min_grasp_prop;
        /*printf("alpha=%05.3f beta=%05.3f delta=%05.3f score=%f \n",alpha,beta,delta,score);*/


        if(score < min_score)
        {
          min_score = score;
          ret.delta_max = delta;
          ret.beta = beta;
          ret.alpha = alpha;
        }
      }
    }
  }
  return ret;
}


HandConfList GetGraspList(Point3D* obstacle_list, CovariancePoint* obstacle_covs, int num,
                      double offset_rot_z_side, double offset_rot_z_top)
{
  HandConfList ret;
  HandConfig max;
  ret.length = 0;
  ret.min_score_index = 0;

  Point3D hand_center;
  double delta_delta, delta_max, delta_min, delta_beta, beta, delta, min_score, score, alpha, delta_alpha, alpha_min, beta_max, beta_loop;
  double min_grasp_prop = 60.0;
  int iter_obstacle, iter_hand_point;
  Point3D hand_points_trans[MAX_HAND_POINTS];
  if(g_data.num_hand_points > MAX_HAND_POINTS)
  {
    printf("Error: too many hand points passed\n");
    g_data.num_hand_points = MAX_HAND_POINTS;
  }
  /** Initialize hand_center*/
  hand_center.x = 0.0;
  hand_center.y = 0.0;
  hand_center.z = 0.0;
  /** Initialize lists*/
  max.alpha = 0.0;
  max.beta = 0.0;
  max.delta_max = 0.0;
  g_data.obstacle_list = obstacle_list;
  g_data.obstacle_covs = obstacle_covs;
  g_data.num_obstacles = num;


  printf("Obstacle List\n");

  for(iter_obstacle = 0; iter_obstacle < g_data.num_obstacles; iter_obstacle++)
  {
    Point3D obs = g_data.obstacle_list[iter_obstacle];
    CovariancePoint cov = g_data.obstacle_covs[iter_obstacle];
    printf("Obs: %05.2f,%05.2f,%05.2f\n", obs.x, obs.y, obs.z);
    printf(" Cov: %05.2f,%05.2f,%05.2f\n%05.2f,%05.2f,%05.2f\n%05.2f,%05.2f,%05.2f\n", cov.sx, cov.sxy, cov.sxz, cov.syx, cov.sy, cov.syz, cov.szx, cov.szy, cov.sz);
  }




  /** Initialize search space + sampling*/
  delta_max = (g_data.max_finger+ (fabs(g_data.obstacle_covs[0].sx) + fabs(g_data.obstacle_covs[0].sy) + fabs(g_data.obstacle_covs[0].sz)) /3.0);

  /**     Consider Table collision*/
  if(fabs(g_data.obstacle_list[0].z - g_data.table_z) < g_data.palm_width )
  {
     printf("Table collision. Limiting alpha to pi/2\n");
     alpha_min = M_PI / 2.0;
     delta_min = g_data.max_finger - (g_data.obstacle_list[0].z - g_data.table_z);
     if(delta_max < delta_min)
      delta_max = delta_min + fabs(g_data.obstacle_covs[0].sz);
  }
  else
  {
    printf("table is safe : %f (=obj_z - table) >= %f (=palm_width)\n", fabs(g_data.obstacle_list[0].z - g_data.table_z),  g_data.palm_width);
    alpha_min = 0.0;
    delta_min = min(fabs(g_data.obstacle_covs[0].sx), min( fabs(g_data.obstacle_covs[0].sy) ,fabs(g_data.obstacle_covs[0].sz)))/2.0;
  }
  /*printf("Delta Max: %f\n", delta_max);*/
  delta_delta = (delta_max - delta_min) / 20.0;
  delta_beta = 0.0872 * M_PI;
  delta_alpha = M_PI/2.0;
  beta_max=M_PI/2.0;

  /** Initialize start score: careful for strange covariances, this value might be to low*/
  min_score = 100000.0;
  for(beta_loop = 0; beta_loop < beta_max; beta_loop += delta_beta)
  {
    for(alpha = alpha_min; alpha < M_PI / 2.0 +0.1; alpha += delta_alpha)
    {
      double score_dmax = 0.0;
      if(alpha == 0)
      {
        beta = beta_loop + offset_rot_z_side;
      }
      else
      {
        beta = beta_loop +  offset_rot_z_top;
      }
      for(iter_hand_point = 0; iter_hand_point < g_data.num_hand_points; iter_hand_point++)
      {
        hand_points_trans[iter_hand_point] = TransformPoint(g_data.hand_points[iter_hand_point], g_data.obstacle_list[0], alpha, beta, delta_max);
      }
      for(iter_obstacle = 0; iter_obstacle < g_data.num_obstacles; iter_obstacle++)
      {
        Point3D obs = g_data.obstacle_list[iter_obstacle];
        CovariancePoint cov = g_data.obstacle_covs[iter_obstacle];
        for(iter_hand_point = 0; iter_hand_point < g_data.num_hand_points; iter_hand_point++)
        {
          score_dmax += f( hand_points_trans[iter_hand_point], obs, cov);
        }
      }
      if(alpha <  M_PI / 4.0)
        delta_min = min(fabs(g_data.obstacle_covs[0].sx), fabs(g_data.obstacle_covs[0].sy))/2.0;
      else
      {
        delta_min = max(fabs(g_data.obstacle_covs[0].sz)/2.0, g_data.max_finger - (g_data.obstacle_list[0].z - g_data.table_z));
        printf("delta min including table: %f\n", delta_min);
      }
      delta_delta = (delta_max - delta_min) / 20.0;
      for(delta = delta_min; delta < delta_max; delta += delta_delta)
      {
        Point3D hand_center_trans = TransformPoint(hand_center, g_data.obstacle_list[0], alpha, beta, delta);
        score = score_dmax;

        /** generate configuration dependant hand points*/
        for(iter_hand_point = 0; iter_hand_point < g_data.num_hand_points; iter_hand_point++)
        {
          hand_points_trans[iter_hand_point] = TransformPoint(g_data.hand_points[iter_hand_point], g_data.obstacle_list[0], alpha, beta, delta);
        }
        for(iter_obstacle = 0; iter_obstacle < g_data.num_obstacles; iter_obstacle++)
        {
          Point3D obs = g_data.obstacle_list[iter_obstacle];
          CovariancePoint cov = g_data.obstacle_covs[iter_obstacle];
          for(iter_hand_point = 0; iter_hand_point < g_data.num_hand_points; iter_hand_point++)
          {
            score += f( hand_points_trans[iter_hand_point], obs, cov);
          }
        }

        score -= f( hand_center_trans , g_data.obstacle_list[0], g_data.obstacle_covs[0]) * min_grasp_prop;
        /*printf("alpha=%05.3f beta=%05.3f delta=%05.3f score=%f \n",alpha,beta,delta,score);*/

        ret.configs[ret.length].delta_max = delta;
        ret.configs[ret.length].beta = beta;
        ret.configs[ret.length].alpha = alpha;        
        ret.scores[ret.length]  = score;     
        ret.length++;
        if(ret.length > MAX_TESTED_POSES)
        {
          printf("To many poses tried!!! Increase MAX_TESTED_POSES\n");
          return ret;
        }
        if(score < min_score)
        {
          ret.min_score_index = ret.length - 1;
          min_score = score;
          max.delta_max = delta;
          max.beta = beta;
          max.alpha = alpha;
        }
      }
    }
  }
  return ret;
}







