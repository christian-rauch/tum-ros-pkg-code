/*
 * Copyright (c) 2008 Nico Blodow <blodow -=- cs.tum.edu>
 *
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
 * $Id:$
 *
 */

/** \author Nico Blodow, Radu Bogdan Rusu */

#include <stdlib.h>
// basic file operations
#include <iostream>
#include <fstream>

#include <sac_model_rotational.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/distances.h>
#include <point_cloud_mapping/geometry/nearest.h>
#include <point_cloud_mapping/geometry/transforms.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <mapping_msgs/PolygonalMap.h>
#include <tf/tf.h>
#include <Eigen/LU>
 
#include <lm.h>
#include <cminpack.h>

using namespace mapping_msgs;
using namespace sensor_msgs;
using namespace geometry_msgs;

/* Subroutine */ int lmdif2(minpack_func_mn fcn, void *p, int m, int n, double *x, 
	double *fvec, double tol, int *iwa, 
	double *wa, int lwa)
{
    /* Initialized data */

    const double factor = 100.;

    int mp5n, mode, nfev;
    double ftol, gtol, xtol;
    double epsfcn;
    int maxfev, nprint;
    int info;

/*     ********** */

/*     subroutine lmdif1 */

/*     the purpose of lmdif1 is to minimize the sum of the squares of */
/*     m nonlinear functions in n variables by a modification of the */
/*     levenberg-marquardt algorithm. this is done by using the more */
/*     general least-squares solver lmdif. the user must provide a */
/*     subroutine which calculates the functions. the jacobian is */
/*     then calculated by a forward-difference approximation. */

/*     the subroutine statement is */

/*       subroutine lmdif1(fcn,m,n,x,fvec,tol,info,iwa,wa,lwa) */

/*     where */

/*       fcn is the name of the user-supplied subroutine which */
/*         calculates the functions. fcn must be declared */
/*         in an external statement in the user calling */
/*         program, and should be written as follows. */

/*         subroutine fcn(m,n,x,fvec,iflag) */
/*         integer m,n,iflag */
/*         double precision x(n),fvec(m) */
/*         ---------- */
/*         calculate the functions at x and */
/*         return this vector in fvec. */
/*         ---------- */
/*         return */
/*         end */

/*         the value of iflag should not be changed by fcn unless */
/*         the user wants to terminate execution of lmdif1. */
/*         in this case set iflag to a negative integer. */

/*       m is a positive integer input variable set to the number */
/*         of functions. */

/*       n is a positive integer input variable set to the number */
/*         of variables. n must not exceed m. */

/*       x is an array of length n. on input x must contain */
/*         an initial estimate of the solution vector. on output x */
/*         contains the final estimate of the solution vector. */

/*       fvec is an output array of length m which contains */
/*         the functions evaluated at the output x. */

/*       tol is a nonnegative input variable. termination occurs */
/*         when the algorithm estimates either that the relative */
/*         error in the sum of squares is at most tol or that */
/*         the relative error between x and the solution is at */
/*         most tol. */

/*       info is an integer output variable. if the user has */
/*         terminated execution, info is set to the (negative) */
/*         value of iflag. see description of fcn. otherwise, */
/*         info is set as follows. */

/*         info = 0  improper input parameters. */

/*         info = 1  algorithm estimates that the relative error */
/*                   in the sum of squares is at most tol. */

/*         info = 2  algorithm estimates that the relative error */
/*                   between x and the solution is at most tol. */

/*         info = 3  conditions for info = 1 and info = 2 both hold. */

/*         info = 4  fvec is orthogonal to the columns of the */
/*                   jacobian to machine precision. */

/*         info = 5  number of calls to fcn has reached or */
/*                   exceeded 200*(n+1). */

/*         info = 6  tol is too small. no further reduction in */
/*                   the sum of squares is possible. */

/*         info = 7  tol is too small. no further improvement in */
/*                   the approximate solution x is possible. */

/*       iwa is an integer work array of length n. */

/*       wa is a work array of length lwa. */

/*       lwa is a positive integer input variable not less than */
/*         m*n+5*n+m. */

/*     subprograms called */

/*       user-supplied ...... fcn */

/*       minpack-supplied ... lmdif */

/*     argonne national laboratory. minpack project. march 1980. */
/*     burton s. garbow, kenneth e. hillstrom, jorge j. more */

/*     ********** */
    /* Parameter adjustments */
    --fvec;
    --iwa;
    --x;
    --wa;

    /* Function Body */
    info = 0;

/*     check the input parameters for errors. */

    if (n <= 0 || m < n || tol < 0. || lwa < m * n + n * 5 + m) {
	/* goto L10; */
        return info;
    }

/*     call lmdif. */

    maxfev = (n + 1) * 200;
    ftol = tol;
    xtol = tol;
    gtol = 0.;
    epsfcn = 0.;
    mode = 1;
    nprint = 0;
    mp5n = m + n * 5;
    info = lmdif(fcn, p, m, n, &x[1], &fvec[1], ftol, xtol, gtol, maxfev,
	    epsfcn, &wa[1], mode, factor, nprint, &nfev, &wa[mp5n + 
	    1], m, &iwa[1], &wa[n + 1], &wa[(n << 1) + 1], &wa[n * 3 + 1], 
	    &wa[(n << 2) + 1], &wa[n * 5 + 1]);
    if (info == 8) {
	info = 4;
    }
/* L10: */
    return info;

/*     last card of subroutine lmdif1. */

} /* lmdif1_ */


namespace sample_consensus
{
std::pair<double,double> getMinMaxK (PointCloud &cloud, std::vector<double> model_coefficients, std::vector<int> inliers)
{
  double minimum = FLT_MAX;
  double maximum = -FLT_MAX;

  geometry_msgs::Point32 axis;   // axis direction vector
  geometry_msgs::Point32 point0; // point on axis
  axis.x = model_coefficients[3] - model_coefficients[0];
  axis.y = model_coefficients[4] - model_coefficients[1];
  axis.z = model_coefficients[5] - model_coefficients[2];

  point0.x = model_coefficients[0];
  point0.y = model_coefficients[1];
  point0.z = model_coefficients[2];


  //double norm = sqrt(axis.x*axis.x + axis.y*axis.y + axis.z*axis.z);

//   double k0 = (0.0
//              - cloud_geometry::dot (point0, axis)) 
//              / cloud_geometry::dot (axis, axis);
  for (unsigned int i = 0; i < inliers.size(); i++)
    //   for (unsigned int i = 0; i < cloud.pts.size(); i++)
  {
    // "k" is the position (x-value?) of the projection of current point on the rot. axis
    double k = (cloud_geometry::dot (cloud.points[inliers[i]], axis) 
              - cloud_geometry::dot (point0, axis)) 
              / cloud_geometry::dot (axis, axis);
//         k = (k - k0);
    if (k < minimum)
      minimum = k;
    if (k > maximum)
      maximum = k;
  }

  // Cleanup
  return std::pair<double,double> (minimum, maximum);
}
  bool
    SACModelRotational::freespace (Point32 p)
    {
      int x,y,z;
      x = (p.x - occupancy_min.x) / occupancy_leaf_width.x;
      y = (p.y - occupancy_min.y) / occupancy_leaf_width.y;
      z = (p.z - occupancy_min.z) / occupancy_leaf_width.z;
      if (x < 0 || y < 0 || z < 0 || x >= occupancy_ndivs.x || y >= occupancy_ndivs.y || z >= occupancy_ndivs.z)
        return false;
      return (occupancy_lookup [x][y][z]); 
    }

  std::vector<double>
    LineToLineSegment (std::vector<double> line_a, std::vector<double> line_b, double eps)
    {
      std::vector<double> segment (6);
      Point32 u;
      Point32 v;
      Point32 w;

      // a = x2 - x1 = line_a[1] - line_a[0]
      u.x = line_a[3] - line_a[0];
      u.y = line_a[4] - line_a[1];
      u.z = line_a[5] - line_a[2];
      // b = x4 - x3 = line_b[1] - line_b[0]
      v.x = line_b[3] - line_b[0];
      v.y = line_b[4] - line_b[1];
      v.z = line_b[5] - line_b[2];
      // c = x2 - x3 = line_a[1] - line_b[0]
      w.x = line_a[3] - line_b[0];
      w.y = line_a[4] - line_b[1];
      w.z = line_a[5] - line_b[2];

      double a = cloud_geometry::dot (u, u);
      double b = cloud_geometry::dot (u, v);
      double c = cloud_geometry::dot (v, v);
      double d = cloud_geometry::dot (u, w);
      double e = cloud_geometry::dot (v, w);
      double denominator = a*c - b*b;
      double sc, tc;
      // Compute the line parameters of the two closest points
      if (denominator < eps)          // The lines are almost parallel
      {
        sc = 0.0;
        tc = (b > c ? d / b : e / c);  // Use the largest denominator
      }
      else
      {
        sc = (b*e - c*d) / denominator;
        tc = (a*e - b*d) / denominator;
      }
      // Get the closest points
      segment[0] = line_a[3] + (sc * u.x);
      segment[1] = line_a[4] + (sc * u.y);
      segment[2] = line_a[5] + (sc * u.z);

      segment[3] = line_b[0] + (tc * v.x);
      segment[4] = line_b[1] + (tc * v.y);
      segment[5] = line_b[2] + (tc * v.z);

      return segment;
    }

  double
    LineToLineDistance (std::vector<double> line_a, std::vector<double> line_b, double eps)
    {
      std::vector<double> segment = LineToLineSegment (line_a, line_b, eps);
      std::vector<double> dP(3);
      // Get the difference of the two closest points
      dP[0] = segment[3] - segment[0];
      dP[1] = segment[4] - segment[1];
      dP[2] = segment[5] - segment[2];
      // Get the closest distance
      double distance = sqrt (dP[0]*dP[0] + dP[1]*dP[1] + dP[2]*dP[2]);
      return distance;
    }

  PointCloud
    SACModelRotational::samplePointsOnRotational  
    (const std::vector<double> modelCoefficients, std::pair<double,double> minmaxK, std::vector<int> inliers)
  {
    static int count = 0;
    count++;
    PointCloud ret;

    // create 100 points from min_k to max_k on the outline of the rotational volume

    Point32 axis;
    axis.x = modelCoefficients[3] - modelCoefficients[0];
    axis.y = modelCoefficients[4] - modelCoefficients[1];
    axis.z = modelCoefficients[5] - modelCoefficients[2];
    Point32 point0;
    point0.x = modelCoefficients[0];
    point0.y = modelCoefficients[1];
    point0.z = modelCoefficients[2];
#define SQR(x) ((x)*(x))
#define SQR_VECT_LENGTH(x) (SQR(x[0])+SQR(x[1])+SQR(x[2]))
#define SQR_NORM(x) SQR_VECT_LENGTH(x)
    
    double norm = sqrt(axis.x*axis.x + axis.y*axis.y + axis.z*axis.z);
    
    //cross product with (0,0,1) is cheap:
    Point32 rotationaxis;
    rotationaxis.x = axis.y/norm;
    rotationaxis.y = -axis.x/norm;
    rotationaxis.z = 0.0;
#define RAD2DEG(r) (double)((r) * 180.0 / M_PI)
//     double rotationangle = - ( acos (axis.z / norm) );  // dot product of a with (0,0,1) is a[2]

#define DOT_PROD_3D(x,y) ((x[0])*(y[0])+(x[1])*(y[1])+(x[2])*(y[2]))
    double k0 = (0.0 - cloud_geometry::dot (point0, axis)) / 
                       cloud_geometry::dot (axis, axis);                                    
    // resolution around axis = 60...
    
    double res_axial = 50.0;
    double res_radial = 30.0;
    for (int i = 0; i < res_axial; i++)
    {
      double X = (((double)i)*((minmaxK.second - minmaxK.first)/res_axial) + minmaxK.first);
      double Y = 0.0;
      X = X - k0;
      
      // evaluate polynomial at position X
      for (int w = 0; w < polynomial_order+1; w++)
        Y += modelCoefficients[6+w] * pow(X,(double)w);
      Point32 p;
      for (int j = 0; j < res_radial; j++)
      {
        p.x = Y;
        p.y = 0.0;
        p.z = ((double)i/res_axial)*norm*(minmaxK.second - minmaxK.first);

        Eigen::Matrix3d rotation1, rotation2;
        Point32 z_axis_vec;
        z_axis_vec.x = 0;
        z_axis_vec.y = 0;
        z_axis_vec.z = 1;
        std::vector<double> norm_rot_axis(3);
        norm_rot_axis[0] = axis.x/norm;
        norm_rot_axis[1] = axis.y/norm;
        norm_rot_axis[2] = axis.z/norm;
        std::vector<double> z_axis(3);
        z_axis[0] = 0;
        z_axis[1] = 0;
        z_axis[2] = 1;
        cloud_geometry::transforms::convertAxisAngleToRotationMatrix (z_axis_vec, i+M_PI*2.0*((double)j)/res_radial, rotation2);
        Eigen::Matrix4d transformation;
        cloud_geometry::transforms::getPlaneToPlaneTransformation (z_axis, norm_rot_axis,
            modelCoefficients[0] + minmaxK.first * axis.x,
            modelCoefficients[1] + minmaxK.first * axis.y,
            modelCoefficients[2] + minmaxK.first * axis.z, transformation);
        
        
//         cloud_geometry::transforms::convertAxisAngleToRotationMatrix (rotationaxis, rotationangle, rotation1);
        
        
        Eigen::Vector3d p_0 (p.x, p.y, p.z);
        p_0 = rotation2 * p_0;
        Eigen::Vector4d p_1 (p_0[0], p_0[1], p_0[2], 1.0);
        Eigen::Vector4d q_0 = transformation * p_1;


//         rotateUp (p, PI*(j/60.0)) 
        //RotateWXYZ (p, rotationangle, rotationaxis);
//         p.x = q_0[0] + modelCoefficients[0] + minmaxK.first * axis.x;
//         p.y = q_0[1] + modelCoefficients[1] + minmaxK.first * axis.y;
//         p.z = q_0[2] + modelCoefficients[2] + minmaxK.first * axis.z;
        p.x = q_0[0];
        p.y = q_0[1];
        p.z = q_0[2];
        
        ret.points.push_back (p);
      }
    }
      Point32 p1;
      Point32 p2;
      geometry_msgs::Polygon polygon;
      p1.x = modelCoefficients[0];
      p1.y = modelCoefficients[1];
      p1.z = modelCoefficients[2];
      p2.x = modelCoefficients[3];
      p2.y = modelCoefficients[4];
      p2.z = modelCoefficients[5];
      polygon.points.push_back (p1);
      polygon.points.push_back (p2);
      pmap.polygons.push_back (polygon);
      pmap.chan[0].values.push_back (0);
      pmap.chan[1].values.push_back (0);
      pmap.chan[2].values.push_back (0);
  return ret;
  }


  double SACModelRotational::computeScore 
      (const std::vector<double> &modelCoefficients, std::pair<double,double> minmaxK, std::vector<int> inliers, PointCloud &cloud_synth, double threshold)
  {
      
//     ROS_ERROR ("inliers: %i\n", inliers.size());
//       std::cerr << "SCORE FOR THE FOLLOWING COEFFS: ";
//       for (unsigned int i = 0; i < modelCoefficients.size(); i++)
//         std::cerr << modelCoefficients[i] << " "; 
//       std::cerr << std::endl;
    ROS_INFO ("coefficients: %g %g %g %g %g %g %g %g %g %g \n",
               modelCoefficients.at (0), modelCoefficients.at (1), modelCoefficients.at (2), modelCoefficients.at (3),
               modelCoefficients.at (4), modelCoefficients.at (5), modelCoefficients.at (6), 
               modelCoefficients.at (7), modelCoefficients.at (8), modelCoefficients.at (9)); 
    static int count = 0;
//     std::cerr << "before sampling" << std::endl;
    PointCloud synth_points = samplePointsOnRotational (modelCoefficients, minmaxK, inliers);
//     std::cerr << "after sampling, kdtree for " << synth_points.pts.size() << std::endl;
    std::vector <int> marks (synth_points.points.size());

    // create kd-tree from synth_points
      
     ROS_INFO ("Creating kd-tree with %i points.", synth_points.points.size());
     cloud_kdtree::KdTreeANN *kd_tree = new cloud_kdtree::KdTreeANN (synth_points);
     ROS_INFO ("Created kd-tree.");
     // go through inliers, mark synth_points that are close as "cat.1"
     
     for (unsigned int i = 0; i < inliers.size (); i++)
     {
       std::vector<int> k_indices;
       std::vector<float> k_distances;
       kd_tree->radiusSearch (cloud_->points[inliers[i]] , threshold*2, k_indices, k_distances);
       for (unsigned int j = 0; j < k_indices.size(); j++)
       {
         marks[k_indices[j]] = 1;
       }
     }
    int mark_count_1 = 0;
    int mark_count_2 = 0;
    for (unsigned int j = 0; j < marks.size(); j++)
    {
      if (marks[j] == 1)
        mark_count_1++;
    }
//     std::cerr << "after marking" << std::endl;

    // go through unmarked synth_points, mark as "cat.2" if not in free space
    for (unsigned int j = 0; j < marks.size(); j++)
    {
      if (marks[j] == 0)
        if (!freespace (synth_points.points[j]))
          mark_count_2++;
    }

    // return (#cat1 + #cat2) / synth_points.pts.size();
    
    //    int cur = pmap.polygons.size ();
    //    pmap.polygons.resize(pmap.polygons.size()+1);
//     for (unsigned int i = 0; i < inliers.size (); i++)
//     {
//       cloud_synth.chan[0].vals.push_back (count);
//       cloud_synth.pts.push_back (cloud_->pts[inliers[i]]);
//     }
    double score = double(mark_count_1+mark_count_2) / double (synth_points.points.size ());
    for (unsigned int i = 0; i < synth_points.points.size (); i++)
    {
      cloud_synth.channels[0].values.push_back (score);
      cloud_synth.points.push_back (synth_points.points[i]);
    }
//      pmap.polygons[cur].points.push_back (synth_points.points[i]);
    count++;
    ROS_ERROR ("SAMPLED %i POINTS! score is : %f", cloud_synth.points.size(), score);
    return double(mark_count_1+mark_count_2) / double (synth_points.points.size ());
  }

  double 
    SACModelRotational::pointToRotationalDistance (const geometry_msgs::Point32 &p, const std::vector<double> &model_coefficients, const int &polynomial_order)
  {
    geometry_msgs::Point32 axis;   // axis direction vector
    geometry_msgs::Point32 point0; // point on axis
    axis.x = model_coefficients[3] - model_coefficients[0];
    axis.y = model_coefficients[4] - model_coefficients[1];
    axis.z = model_coefficients[5] - model_coefficients[2];

    point0.x = model_coefficients[0];
    point0.y = model_coefficients[1];
    point0.z = model_coefficients[2];
    
    double f  = cloud_geometry::distances::pointToLineDistance (p, point0, axis);
    double k0 = (0.0 - cloud_geometry::dot (point0, axis)) / 
                       cloud_geometry::dot (axis, axis);                                    

    double k = (cloud_geometry::dot (p, axis) 
              - cloud_geometry::dot (point0, axis)) 
              / cloud_geometry::dot (axis, axis);

    double r = 0.0; 
    for (int w = 0; w < polynomial_order + 1; w++)
      r += model_coefficients[6+w] * pow (k-k0, (double)w);

    return fabs(f - fabs(r));
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Get 6 random points as data samples and return them as point indices.
    * \param iterations the internal number of iterations used by SAC methods
    * \param samples the resultant model samples
    * \note assumes unique points and requires normals!
    */
  void
    SACModelRotational::getSamples (int &iterations, std::vector<int> &samples)
  {
    samples.resize (6);

    // Get random indices
    for (int i = 0; i < 6; i++)
      samples[i] = (int)(indices_.size () * (rand () / (RAND_MAX + 1.0)));

    return;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Select all the points which respect the given model coefficients as inliers.
    * \param model_coefficients the coefficients of a cylinder model that we need to compute distances to
    * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
    * \param inliers the resultant model inliers
    * \note: To get the refined inliers of a model, use:
    * ANNpoint refined_coeff = refitModel (...); selectWithinDistance (refined_coeff, threshold);
    */
  void
    SACModelRotational::selectWithinDistance (const std::vector<double> &model_coefficients, double threshold, std::vector<int> &inliers)
  {
    int nr_p = 0;
    inliers.resize (indices_.size ());

    // Model coefficients: [point_on_axis axis_direction c_0 c_1 ...]
    // where c_i are the polynomial coefficients
    // Iterate through the 3d points and calculate the distances from them to the cylinder
    for (unsigned int i = 0; i < indices_.size (); i++)
    {
      // Aproximate the distance from the point to the rotational object 
      // perpendicular to the axis of rotation 
      // (not orthogonal to the surface)
      if (fabs (/*cloud_geometry::distances::*/
                pointToRotationalDistance (cloud_->points.at (indices_[i]), model_coefficients, polynomial_order)
               ) < threshold)
      {
        // Returns the indices of the points whose distances are smaller than the threshold
        inliers[nr_p] = indices_[i];
        nr_p++;
      }
    }
    inliers.resize (nr_p);
    return;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Compute all distances from the cloud data to a given cylinder model.
    * \param model_coefficients the coefficients of a cylinder model that we need to compute distances to
    * \param distances the resultant estimated distances
    */
  void
    SACModelRotational::getDistancesToModel (const std::vector<double> &model_coefficients, std::vector<double> &distances)
  {
    distances.resize (indices_.size ());

    // Iterate through the 3d points and calculate the distances from them to the cylinder
    for (unsigned int i = 0; i < indices_.size (); i++)
      // Aproximate the distance from the point to the cylinder as the difference between
      //dist(point,cylinder_axis) and cylinder radius
      // NOTE: need to revise this.
      distances[i] = fabs (/*cloud_geometry::distances::*/
                 pointToRotationalDistance (cloud_->points.at (indices_[i]), model_coefficients, polynomial_order));
    return;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Create a new point cloud with inliers projected onto the cylinder model.
    * \param inliers the data inliers that we want to project on the cylinder model
    * \param model_coefficients the coefficients of a cylinder model
    * \param projected_points the resultant projected points
    * \todo implement this.
    */
  void
    SACModelRotational::projectPoints (const std::vector<int> &inliers, const std::vector<double> &model_coefficients,
                                     PointCloud &projected_points)
  {
    std::cerr << "[SACModelRotational::projecPoints] Not implemented yet." << std::endl;
    projected_points = *cloud_;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Project inliers (in place) onto the given cylinder model.
    * \param inliers the data inliers that we want to project on the cylinder model
    * \param model_coefficients the coefficients of a cylinder model
    * \todo implement this.
    */
  void
    SACModelRotational::projectPointsInPlace (const std::vector<int> &inliers, const std::vector<double> &model_coefficients)
  {
    std::cerr << "[SACModelRotational::projecPointsInPlace] Not implemented yet." << std::endl;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Check whether the given index samples can form a valid cylinder model, compute the model coefficients from
    * these samples and store them internally in model_coefficients_. The cylinder coefficients are: x, y, z, R.
    * \param samples the point indices found as possible good candidates for creating a valid model
    */
  bool
    SACModelRotational::computeModelCoefficients (const std::vector<int> &samples)
  {
    model_coefficients_.resize (6+1+polynomial_order);

    // Save the nx/ny/nz channel indices the first time we run this
    if (nx_idx_ == -1)
    {
      nx_idx_ = cloud_geometry::getChannelIndex (*cloud_, "nx");
      if (nx_idx_ == -1) return (false);
    }
    if (ny_idx_ == -1)
    {
      ny_idx_ = cloud_geometry::getChannelIndex (*cloud_, "ny");
      if (ny_idx_ == -1) return (false);
    }
    if (nz_idx_ == -1)
    {
      nz_idx_ = cloud_geometry::getChannelIndex (*cloud_, "nz");
      if (nz_idx_ == -1) return (false);
    }
    
    geometry_msgs::Point32 centroid;
    cloud_geometry::nearest::computeCentroid (*cloud_, samples, centroid);

    // use the centroid as an initial guess for a point on the axis,
    // and the y axis as a rotation axis
    // this will get changed by the first optimization step.
    model_coefficients_[0] = centroid.x;
    model_coefficients_[1] = centroid.y;
    model_coefficients_[2] = centroid.z;
    model_coefficients_[3] = centroid.x + 0.0;
    model_coefficients_[4] = centroid.y + 0.0;
    model_coefficients_[5] = centroid.z + 1.0;

      Point32 p1;
      Point32 p2;
      geometry_msgs::Polygon polygon;
//     for (unsigned int i = 0; i < samples.size(); i++)
//     {
//     polygon.points.clear();
//       polygon.points.push_back ( cloud_->points[samples[i]] );
//       p1.x = 0.2*cloud_->channels[nx_idx_].values[samples[i]] + cloud_->points[samples[i]].x;
//       p1.y = 0.2*cloud_->channels[ny_idx_].values[samples[i]] + cloud_->points[samples[i]].y;
//       p1.z = 0.2*cloud_->channels[nz_idx_].values[samples[i]] + cloud_->points[samples[i]].z;
//       polygon.points.push_back ( p1 );
//       pmap.polygons.push_back (polygon);
//       pmap.chan[0].values.push_back (0);
//       pmap.chan[1].values.push_back (0);
//       pmap.chan[2].values.push_back (0);
//     }
//
//     p1.x = model_coefficients_[0];
//     p1.y = model_coefficients_[1];
//     p1.z = model_coefficients_[2];
//     p2.x = model_coefficients_[3];
//     p2.y = model_coefficients_[4];
//     p2.z = model_coefficients_[5];
//     polygon.points.push_back (p1);
//     polygon.points.push_back (p2);
//     pmap.polygons.push_back (polygon);
//     pmap.chan[0].values.push_back (0);
//     pmap.chan[1].values.push_back (0);
//     pmap.chan[2].values.push_back (0);

     if (!refitAxis (samples, model_coefficients_))
       return false;
//      if (!refitModelNoAxis (samples, model_coefficients_))
//        return false;
//----------------------- visualize the axis
       p1.x = model_coefficients_[0];
       p1.y = model_coefficients_[1];
       p1.z = model_coefficients_[2];
       p2.x = model_coefficients_[3];
       p2.y = model_coefficients_[4];
       p2.z = model_coefficients_[5];
       polygon.points.clear();
       polygon.points.push_back (p1);
       polygon.points.push_back (p2);
       pmap.polygons.push_back (polygon);
       pmap.chan[0].values.push_back (1);
       pmap.chan[1].values.push_back (1);
       pmap.chan[2].values.push_back (1);
//-----------------------
//    pmap.polygons.resize (pmap.polygons.size()+1);
//     pmap.polygons[(pmap.polygons.size()-1)].push_back (p1);
//     pmap.polygons[(pmap.polygons.size()-1)].push_back (p2);
//    firstOptimization ();
      return true;
      Eigen::MatrixXf A = Eigen::MatrixXf (polynomial_order + 1, polynomial_order + 1);
      Eigen::VectorXf b = Eigen::VectorXf (polynomial_order + 1);
      Eigen::VectorXf xvec;

      geometry_msgs::Point32 axis;   // axis direction vector
      geometry_msgs::Point32 point0; // point on axis
      axis.x = model_coefficients_[3] - model_coefficients_[0];
      axis.y = model_coefficients_[4] - model_coefficients_[1];
      axis.z = model_coefficients_[5] - model_coefficients_[2];

      point0.x = model_coefficients_[0];
      point0.y = model_coefficients_[1];
      point0.z = model_coefficients_[2];
      
      double k0 = (0.0 - cloud_geometry::dot (point0, axis)) / 
                         cloud_geometry::dot (axis, axis);                                    

      for (int d1 = 0; d1 < polynomial_order + 1; d1++)
      {
        double x = (cloud_geometry::dot (cloud_->points.at (samples.at (d1)), axis) 
                  - cloud_geometry::dot (point0, axis)) 
                  / cloud_geometry::dot (axis, axis);
        x = x - k0;
        b[d1] = cloud_geometry::distances::pointToLineDistance (cloud_->points.at (samples.at (d1)), point0, axis);
        for (int d2 = 0; d2 < polynomial_order + 1; d2++)
          A(d1,d2) = pow (x, (double) d2);
      }

      if (!A.lu().solve(b, &xvec))
      {
        return (false);
      }
     for (int i = 0; i < polynomial_order + 1; i++)
       model_coefficients_[6+i] = xvec[i];
    for (int i = 0; i < 4; i++)
      std::cerr << pointToRotationalDistance (cloud_->points.at(samples[i]), model_coefficients_, polynomial_order) << " - ";
    std::cerr << std::endl;
    return (true);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Recompute the cylinder coefficients using the given inlier set and return them to the user.
    * @note: these are the coefficients of the cylinder model after refinement (eg. after SVD)
    * \param inliers the data inliers found as supporting the model
    * \param refit_coefficients the resultant recomputed coefficients after non-linear optimization
    */
  void
    SACModelRotational::refitModel (const std::vector<int> &inliers, std::vector<double> &refit_coefficients)
  {
    if (inliers.size () == 0)
    {
       ROS_ERROR ("[SACModelRotational::RefitModel] Cannot re-fit 0 inliers!");
      refit_coefficients = model_coefficients_;
      return;
    }
    if (model_coefficients_.size () == 0)
    {
      ROS_WARN ("[SACModelRotational::RefitModel] Initial model coefficients have not been estimated yet - proceeding without an initial solution!");
      best_inliers_ = indices_;
    }

    tmp_inliers_ = &inliers;

    int m = inliers.size ();

    double *fvec = new double[m];

    int n = 10;      // 7 unknowns
    int iwa[n];

    int lwa = m * n + 5 * n + m;
    double *wa = new double[lwa];

    // Set the initial solution
    double x[10] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if ((int)model_coefficients_.size () >= n)
      for (int d = 0; d < n; d++)
        x[d] = model_coefficients_.at (d);

    // Set tol to the square root of the machine. Unless high solutions are required, these are the recommended settings.
    double tol = sqrt (dpmpar (2));

    // Optimize using forward-difference approximation LM
    int info = lmdif2 (&sample_consensus::SACModelRotational::functionToOptimize, this, m, n, x, fvec, tol, iwa, wa, lwa);

    // Compute the L2 norm of the residuals
     ROS_INFO ("LM solver finished with exit code %i, having a residual norm of %g. \nInitial solution: %g %g %g %g %g %g %g \nFinal solution: %g %g %g %g %g %g %g",
                info, enorm (m, fvec), model_coefficients_.at (0), model_coefficients_.at (1), model_coefficients_.at (2), model_coefficients_.at (3),
                model_coefficients_.at (4), model_coefficients_.at (5), model_coefficients_.at (6), x[0], x[1], x[2], x[3], x[4], x[5], x[6]);

//    refit_coefficients.resize (n);
    for (int d = 0; d < n; d++)
      refit_coefficients[d] = x[d];

    free (wa); free (fvec);
  }
  bool
    SACModelRotational::refitModelNoAxis (const std::vector<int> &inliers, std::vector<double> &refit_coefficients)
  {
    if (inliers.size () == 0)
    {
       ROS_ERROR ("[SACModelRotational::RefitModel] Cannot re-fit 0 inliers!");
      refit_coefficients = model_coefficients_;
      return false;
    }
    if (model_coefficients_.size () == 0)
    {
      ROS_WARN ("[SACModelRotational::RefitModel] Initial model coefficients have not been estimated yet - proceeding without an initial solution!");
      best_inliers_ = indices_;
    }

    tmp_inliers_ = &inliers;

    int m = inliers.size ();

    double *fvec = new double[m];

    int n = 4;      // 7 unknowns
    int iwa[n];

    int lwa = m * n + 5 * n + m;
    double *wa = new double[lwa];

    // Set the initial solution
    double x[4] = {0.0, 0.0, 0.0, 0.0};
//    if ((int)model_coefficients_.size () >= n)
      for (int d = 0; d < n; d++)
        x[d] = model_coefficients_.at (d+6);

    // Set tol to the square root of the machine. Unless high solutions are required, these are the recommended settings.
    double tol = sqrt (dpmpar (2));

    // Optimize using forward-difference approximation LM
    int info = lmdif2 (&sample_consensus::SACModelRotational::functionToOptimize, this, m, n, x, fvec, tol, iwa, wa, lwa);

    // Compute the L2 norm of the residuals
     ROS_INFO ("LM solver finished with exit code %i, having a residual norm of %g. \nInitial solution: %g %g %g %g \nFinal solution: %g %g %g %g",
                info, enorm (m, fvec), model_coefficients_.at (0), model_coefficients_.at (1), model_coefficients_.at (2), model_coefficients_.at (3),
                x[0], x[1], x[2], x[3]);

//    refit_coefficients.resize (n);
    for (int d = 0; d < n; d++)
      refit_coefficients[d+6] = x[d];

    free (wa); free (fvec);
    return true;
  }

void
  rot_axis_func (double *p, double *X, int m, int n, void *data)
{
  struct LMStrucData *d = (struct LMStrucData *) data;
//      ROS_INFO ("rot_axis_func %g %g %g %g %g %g", 
//                 p[0], p[1], p[2], p[3], p[4], p[5]);
  
//      Point32 p1;
//      Point32 p2;
//      geometry_msgs::Polygon polygon;
//      p1.x = p[0];
//      p1.y = p[1];
//      p1.z = p[2];
//      p2.x = p[3];
//      p2.y = p[4];
//      p2.z = p[5];
//      polygon.points.clear();
//      polygon.points.push_back (p1);
//      polygon.points.push_back (p2);
//      d->model->pmap.polygons.push_back (polygon);
//      d->model->pmap.chan[0].values.push_back (1);
//      d->model->pmap.chan[1].values.push_back (1);
//      d->model->pmap.chan[2].values.push_back (1);

    std::vector<double> rot_coeff (6);
    for (int d = 0; d < 6; d++)
      rot_coeff[d] = p[d];

    std::vector<double> point_normal_line (6);

    for (int i = 0; i < m; i++)
    {
      point_normal_line[0] = d->cloud->points[d->samples.at(i)].x;
      point_normal_line[1] = d->cloud->points[d->samples.at(i)].y;
      point_normal_line[2] = d->cloud->points[d->samples.at(i)].z;
      
      point_normal_line[3] = d->cloud->points[d->samples.at(i)].x + d->cloud->channels[d->nx_idx_].values[d->samples.at(i)];
      point_normal_line[4] = d->cloud->points[d->samples.at(i)].y + d->cloud->channels[d->ny_idx_].values[d->samples.at(i)];
      point_normal_line[5] = d->cloud->points[d->samples.at(i)].z + d->cloud->channels[d->nz_idx_].values[d->samples.at(i)];
      
      double ll = LineToLineDistance (rot_coeff, point_normal_line, 1e-5); 
      X[i] = ll;// * ll;
    }
}

  bool
    SACModelRotational::refitAxis (const std::vector<int> &inliers, std::vector<double> &refit_coefficients)
  {
    std::cerr << "REFIT AXIS COEFFS BEFOR: ";
    for (unsigned int i = 0; i < refit_coefficients.size(); i++)
      std::cerr << refit_coefficients[i] << " "; 
    std::cerr << std::endl;
    ROS_INFO ("optimizing axis from %i points", inliers.size());
    
    if (inliers.size () == 0)
    {
      ROS_ERROR ("[SACModelRotational::RefitModel] Cannot re-fit 0 inliers!");
      refit_coefficients = model_coefficients_;
      return false;
    }
    if (model_coefficients_.size () == 0)
    {
      ROS_WARN ("[SACModelRotational::RefitModel] Initial model coefficients have not been estimated yet - proceeding without an initial solution!");
      best_inliers_ = indices_;
    }

     tmp_inliers_ = &inliers;

     int m = inliers.size ();

     double *fvec = new double[m];

     int n = 6;      // 6 unknowns
     int iwa[n];

     int lwa = m * n + 5 * n + m;
     double *wa = new double[lwa];

     // Set the initial solution
     double x[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//      if ((int)model_coefficients_.size () >= n)
     for (int d = 0; d < n; d++)
       x[d] = refit_coefficients.at (d);

     // Set tol to the square root of the machine. Unless high solutions are required, these are the recommended settings.
     double tol = sqrt (dpmpar (2));

     // Optimize using forward-difference approximation LM
     int info = lmdif1 (&sample_consensus::SACModelRotational::functionToOptimizeAxis, this, m, n, x, fvec, tol, iwa, wa, lwa);
      info=info;
     // Compute the L2 norm of the residuals
      ROS_INFO ("LM solver finished with exit code %i, having a residual norm of %g. \nInitial solution: %g %g %g %g %g %g \nFinal solution: %g %g %g %g %g %g ",
                 info, enorm (m, fvec), 
                 refit_coefficients.at (0), refit_coefficients.at (1), refit_coefficients.at (2), refit_coefficients.at (3),
                 refit_coefficients.at (4), refit_coefficients.at (5), x[0], x[1], x[2], x[3], x[4], x[5]);

     //refit_coefficients.resize (n);
     for (int d = 0; d < n; d++)
       refit_coefficients[d] = x[d];

    // renormalize coeffs
    std::pair<double,double> minmax = getMinMaxK (*cloud_ , refit_coefficients, inliers);
    
    geometry_msgs::Point32 axis;   // axis direction vector
    geometry_msgs::Point32 p1;
    geometry_msgs::Point32 p2;
    axis.x = refit_coefficients[3] - refit_coefficients[0];
    axis.y = refit_coefficients[4] - refit_coefficients[1];
    axis.z = refit_coefficients[5] - refit_coefficients[2];
    
    p1.x = refit_coefficients[0] + minmax.first  * axis.x;
    p2.x = refit_coefficients[0] + minmax.second * axis.x;
    p1.y = refit_coefficients[1] + minmax.first  * axis.y;
    p2.y = refit_coefficients[1] + minmax.second * axis.y;
    p1.z = refit_coefficients[2] + minmax.first  * axis.z;
    p2.z = refit_coefficients[2] + minmax.second * axis.z;

    refit_coefficients[0] = p1.x;
    refit_coefficients[1] = p1.y;
    refit_coefficients[2] = p1.z;
    refit_coefficients[3] = p2.x;
    refit_coefficients[4] = p2.y;
    refit_coefficients[5] = p2.z;

    // refit polynomial
    Eigen::MatrixXf A = Eigen::MatrixXf (inliers.size(), polynomial_order + 1);
    Eigen::VectorXf b = Eigen::VectorXf (inliers.size());
    Eigen::VectorXf xvec;

    axis.x = p2.x - p1.x;
    axis.y = p2.y - p1.y;
    axis.z = p2.z - p1.z;
    
    double k0 = (0.0 - cloud_geometry::dot (p1, axis)) / 
                       cloud_geometry::dot (axis, axis);                                    

    for (unsigned int d1 = 0; d1 < inliers.size(); d1++)
    {
      double x = (cloud_geometry::dot (cloud_->points.at (inliers.at (d1)), axis) 
                - cloud_geometry::dot (p1, axis)) 
                / cloud_geometry::dot (axis, axis);
      x = x - k0;
      b[d1] = cloud_geometry::distances::pointToLineDistance (cloud_->points.at (inliers.at (d1)), p1, axis);
      for (int d2 = 0; d2 < polynomial_order + 1; d2++)
        A(d1,d2) = pow (x, (double) d2);
    }
  
    Eigen::MatrixXf A_t1 = A.transpose (); 
    Eigen::MatrixXf A_t2 = A.transpose (); 
    Eigen::MatrixXf A_temp = A_t1 * A;
    xvec = A_temp.inverse () * A_t2 * b;
    bool isnanorinf = false;
    for (int i = 0; i < polynomial_order+1; i++)
      if (std::isinf (xvec[i]) || std::isnan (xvec[i]))
      {
        ROS_ERROR ("NAN OR INF: %f", xvec[i]);
        isnanorinf = true;
      }

    if (isnanorinf)
    {
// write affected points to file
//       static int filecounter = 0;
//       std::ofstream myfile;
//       std::stringstream filename;
//       filename << "nan" << filecounter++ << ".pcd";
//       myfile.open (filename.str().c_str());
//       myfile << "COLUMNS x y z i\nPOINTS " << inliers.size()+2 << "\nDATA ascii\n";
//       myfile << p1.x 
//              << " "<< p1.y
//              << " "<< p1.z << " " << filecounter << std::endl;
//       myfile << p2.x 
//              << " "<< p2.y
//              << " "<< p2.z << " " << filecounter << std::endl;

//       for (unsigned int d1 = 0; d1 < inliers.size(); d1++)
//         myfile << cloud_->points.at (inliers.at (d1)).x
//                << " "<< cloud_->points.at (inliers.at (d1)).y
//                << " "<< cloud_->points.at (inliers.at (d1)).z << " " << filecounter << std::endl;
//       myfile.close();

      return false;

    }
//     if (!A.lu().solve(b, &xvec))
//     {
//       return (false);
//     }
    for (int i = 0; i < polynomial_order + 1; i++)
      refit_coefficients[6+i] = xvec[i];
       std::cerr << "REFIT AXIS COEFFS AFTER: ";
       for (unsigned int i = 0; i < refit_coefficients.size(); i++)
         std::cerr << refit_coefficients[i] << " "; 
       std::cerr << std::endl;
    return true; 
//     free (wa); free (fvec);
  }

  void
    SACModelRotational::refitAxisGoodLevmar (const std::vector<int> &samples, std::vector<double> &refit_coefficients)
  {
//     ROS_INFO ("optimizing axis from %i points... normal indices: %i, %i, %i", samples.size(), nx_idx_, ny_idx_, nz_idx_);
    if (samples.size () == 0)
    {
      ROS_ERROR ("[SACModelRotational::RefitModel] Cannot re-fit 0 inliers!");
      refit_coefficients = model_coefficients_;
      return;
    }
    if (model_coefficients_.size () == 0)
    {
      ROS_WARN ("[SACModelRotational::RefitModel] Initial model coefficients have not been estimated yet - proceeding without an initial solution!");
      best_inliers_ = indices_;
    }
            LMStrucData data;
            data.model  = this;
            data.cloud  = cloud_;
            data.samples = samples;
            data.nx_idx_ = nx_idx_;
            data.ny_idx_ = ny_idx_;
            data.nz_idx_ = nz_idx_;

            double *x (new double[samples.size ()]);
            for (unsigned int i = 0; i < samples.size (); i++)
              x[i] = 0;

            // I: minim. options [\tau, \epsilon1, \epsilon2, \epsilon3]. Respectively the scale factor for initial \mu,
            // stopping thresholds for ||J^T e||_inf, ||Dp||_2 and ||e||_2. Set to NULL for defaults to be used
            double opts[LM_OPTS_SZ], info[LM_INFO_SZ];
            opts[0] = LM_INIT_MU;
            opts[1] = 1E-15;
            opts[2] = 1E-15;
            opts[3] = 1E-20;
            opts[4] = LM_DIFF_DELTA;

            int m = 6;  // parameter vector dimension (i.e. #unknowns)
            int n = samples.size (); // I: measurement vector dimension
            int itmax = 5000;        // I: maximum number of iterations

            // I/O: initial parameter estimates. On output contains the estimated solution
            double p[m];
            for (int d = 0; d < m; d++)
              p[d] = refit_coefficients[d];

//            print_info (stderr, "[SACModelRotational::RefitModel] Refitting the rotational axis: ");
//            for (int i = 0; i < 6; i++)
//              fprintf (stderr, "%.5g ", p[i]);
//            fprintf (stderr, "\n");

            //int ret = dlevmar_der (rotational_func, rotational_jac, p, x, m, n, itmax, opts, info, NULL, NULL, (void *) &data);
            int ret = dlevmar_dif (rot_axis_func, p, x, m, n, itmax, opts, info, NULL, NULL, (void *) &data);
            ret=ret;
//      ROS_INFO ("LM solver finished with exit code %i\nInitial solution: %g %g %g %g %g %g \nFinal solution: %g %g %g %g %g %g ",
//                 ret,  
//                 refit_coefficients.at (0), refit_coefficients.at (1), refit_coefficients.at (2), refit_coefficients.at (3),
//                 refit_coefficients.at (4), refit_coefficients.at (5), p[0], p[1], p[2], p[3], p[4], p[5]);
            delete [] x;

            ANNpoint newcoeff = annAllocPt (6);
            newcoeff[0] = p[0]; newcoeff[1] = p[1]; newcoeff[2] = p[2];
            newcoeff[3] = p[3]; newcoeff[4] = p[4]; newcoeff[5] = p[5];
            for (int d = 0; d < m; d++)
              newcoeff[d] = p[d];

            // recompute the line coefficients so that they store the first and last points of the Rotational
            // Compute the line direction (P2 - P1)
            ANNcoord p21[3];
            p21[0] = p[3] - p[0];
            p21[1] = p[4] - p[1];
            p21[2] = p[5] - p[2];

            double k_min = FLT_MAX;
            double k_max = -FLT_MAX;
            double k0 = (0.0 - DOT_PROD_3D (p, p21)) / DOT_PROD_3D (p21, p21);
            for (unsigned int i = 0; i < samples.size (); i++)
            {
              double point[3] = 
              { cloud_->points[samples[i]].x,
                cloud_->points[samples[i]].y,
                cloud_->points[samples[i]].z};
              double k = (DOT_PROD_3D (point, p21) - DOT_PROD_3D (p, p21)) / DOT_PROD_3D (p21, p21);
              k = k - k0;
              if (k < k_min)
                k_min = k;
              if (k > k_max)
                k_max = k;
            }
            for (int d = 0; d <3; d++)
            {
              newcoeff[d+0] = p[d] + k_min * p21[d];
              newcoeff[d+3] = p[d] + k_max * p21[d];
            }
//            print_info (stderr, "[SACModelRotational::RefitModel] Rot. Axis got %04d / %04g iterations, reason %g, solution: ", ret, info[5], info[6]);
//            for (int i = 0; i < 6; i++)
//              fprintf (stderr, "%.5g ", newcoeff[i]);
//            fprintf (stderr, "\n");
//            cerr << "k is in the range [" << k_min << " : " << k_max << "]" << endl; 

            for (int d = 0; d < 6; d++)
              refit_coefficients[d] = newcoeff[d];





    std::pair<double,double> minmax = getMinMaxK (*cloud_ , refit_coefficients, samples);
    
    std::vector<double> axis(3);
    std::vector<double> p1(3);
    std::vector<double> p2(3);
    axis[0] = refit_coefficients[3] - refit_coefficients[0];
    axis[1] = refit_coefficients[4] - refit_coefficients[1];
    axis[2] = refit_coefficients[5] - refit_coefficients[2];
    
    p1[0] = refit_coefficients[0] + minmax.first * axis[0];
    p2[0] = refit_coefficients[0] + minmax.second * axis[0];
    p1[1] = refit_coefficients[1] + minmax.first * axis[1];
    p2[1] = refit_coefficients[1] + minmax.second * axis[1];
    p1[2] = refit_coefficients[2] + minmax.first * axis[2];
    p2[2] = refit_coefficients[2] + minmax.second * axis[2];

    refit_coefficients[0] = p1[0];
    refit_coefficients[1] = p1[1];
    refit_coefficients[2] = p1[2];
    refit_coefficients[3] = p2[0];
    refit_coefficients[4] = p2[1];
    refit_coefficients[5] = p2[2];

//     free (wa); free (fvec);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////(
  /** \brief Cost function to be minimized
    * \param p a pointer to our data structure array
    * \param m the number of functions
    * \param n the number of variables
    * \param x a pointer to the variables array
    * \param fvec a pointer to the resultant functions evaluations
    * \param iflag set to -1 inside the function to terminate execution
    */
  int
    SACModelRotational::functionToOptimizeNoAxis (void *p, int m, int n, const double *x, double *fvec, int iflag)
  {
    SACModelRotational *model = (SACModelRotational*)p;
    
    std::vector<double> rot_coeff (6+1+model->polynomial_order);
    
    for (int d = 0; d < 6; d++)
      rot_coeff[d] = model->model_coefficients_[d];
    for (int d = 6; d < 6+1+model->polynomial_order; d++)
      rot_coeff[d] = x[6-d];

    for (int i = 0; i < m; i++)
      // dist = f - r
      fvec[i] = 100*pointToRotationalDistance (model->cloud_->points[model->tmp_inliers_->at (i)], rot_coeff, model->polynomial_order);

    return (0);
  }
  int
    SACModelRotational::functionToOptimize (void *p, int m, int n, const double *x, double *fvec, int iflag)
  {
    SACModelRotational *model = (SACModelRotational*)p;
    
    std::vector<double> rot_coeff (6+1+model->polynomial_order);
    for (int d = 0; d < 6+1+model->polynomial_order; d++)
      rot_coeff[d] = x[d];

    for (int i = 0; i < m; i++)
      // dist = f - r
      fvec[i] = 100*pointToRotationalDistance (model->cloud_->points[model->tmp_inliers_->at (i)], rot_coeff, model->polynomial_order);

    return (0);
  }
  
  int
    SACModelRotational::functionToOptimizeAxis (void *p, int m, int n, const double *x, double *fvec, int iflag)
  {
     // Compute the L2 norm of the residuals
    SACModelRotational *model = (SACModelRotational*)p;
   
    std::vector<double> rot_coeff (6);
    for (int d = 0; d < 6; d++)
      rot_coeff[d] = x[d];

    std::vector<double> point_normal_line (6);

//     Point32 p1;
//     Point32 p2;
//     robot_msgs::Polygon3D polygon;
//     p1.x = x[0];
//     p1.y = x[1];
//     p1.z = x[2];
//     p2.x = x[3];
//     p2.y = x[4];
//     p2.z = x[5];
//     polygon.points.clear();
//     polygon.points.push_back (p1);
//     polygon.points.push_back (p2);
//     model->pmap.polygons.push_back (polygon);
//     model->pmap.chan[0].vals.push_back (1);
//     model->pmap.chan[1].vals.push_back (1);
//     model->pmap.chan[2].vals.push_back (1);
    
    for (int i = 0; i < m; i++)
    {
      point_normal_line[0] = model->cloud_->points[model->tmp_inliers_->at(i)].x;
      point_normal_line[1] = model->cloud_->points[model->tmp_inliers_->at(i)].y;
      point_normal_line[2] = model->cloud_->points[model->tmp_inliers_->at(i)].z;
      
      point_normal_line[3] = model->cloud_->points[model->tmp_inliers_->at(i)].x + model->cloud_->channels[model->nx_idx_].values[model->tmp_inliers_->at(i)];
      point_normal_line[4] = model->cloud_->points[model->tmp_inliers_->at(i)].y + model->cloud_->channels[model->ny_idx_].values[model->tmp_inliers_->at(i)];
      point_normal_line[5] = model->cloud_->points[model->tmp_inliers_->at(i)].z + model->cloud_->channels[model->nz_idx_].values[model->tmp_inliers_->at(i)];
      
      double ll = LineToLineDistance (rot_coeff, point_normal_line, 1e-5); 
      // dist = f - r
      fvec[i] = ll * ll;
//       fvec[i] = pointToRotationalDistance (model->cloud_->points[model->tmp_inliers_->at (i)], rot_coeff, model->polynomial_order) - x[6];
    }

    return (0);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Verify whether a subset of indices verifies the internal cylinder model coefficients.
    * \param indices the data indices that need to be tested against the cylinder model
    * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
    */
  bool
    SACModelRotational::doSamplesVerifyModel (const std::set<int> &indices, double threshold)
  {
    for (std::set<int>::iterator it = indices.begin (); it != indices.end (); ++it)
      // Aproximate the distance from the point to the cylinder as the difference between
      //dist(point,cylinder_axis) and cylinder radius
      // NOTE: need to revise this.
      if (fabs (
                cloud_geometry::distances::pointToLineDistance (cloud_->points.at (*it), model_coefficients_) - model_coefficients_[6]
               ) > threshold)
        return (false);

    return (true);
  }
}

