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
#include <sac_model_rotational.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/distances.h>
#include <point_cloud_mapping/geometry/nearest.h>
#include <Eigen/LU>

#include <cminpack.h>

namespace sample_consensus
{
  double 
    SACModelRotational::pointToRotationalDistance (const robot_msgs::Point32 &p, const std::vector<double> &model_coefficients, const int &polynomial_order)
  {
    robot_msgs::Point32 axis;   // axis direction vector
    robot_msgs::Point32 point0; // point on axis
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
                pointToRotationalDistance (cloud_->pts.at (indices_[i]), model_coefficients, polynomial_order)
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
                 pointToRotationalDistance (cloud_->pts.at (indices_[i]), model_coefficients, polynomial_order));
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
                                     robot_msgs::PointCloud &projected_points)
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
    
    robot_msgs::Point32 centroid;
    cloud_geometry::nearest::computeCentroid (*cloud_, samples, centroid);

    // use the centroid as an initial guess for a point on the axis,
    // and the y axis as a rotation axis
    // this will get changed by the first optimization step.
    model_coefficients_[0] = centroid.x;
    model_coefficients_[1] = centroid.y;
    model_coefficients_[2] = centroid.z;
    model_coefficients_[3] = centroid.x + 0.0;
    model_coefficients_[4] = centroid.y + 1.0;
    model_coefficients_[5] = centroid.z + 0.0;

//    firstOptimization ();
    Eigen::MatrixXf A = Eigen::MatrixXf (polynomial_order + 1, polynomial_order + 1);
    Eigen::VectorXf b = Eigen::VectorXf (polynomial_order + 1);
    Eigen::VectorXf x;

    robot_msgs::Point32 axis;   // axis direction vector
    robot_msgs::Point32 point0; // point on axis
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
      double x = (cloud_geometry::dot (cloud_->pts.at (samples.at (d1)), axis) 
                - cloud_geometry::dot (point0, axis)) 
                / cloud_geometry::dot (axis, axis);
      x = x - k0;
      b[d1] = cloud_geometry::distances::pointToLineDistance (cloud_->pts.at (samples.at (d1)), point0, axis);
      for (int d2 = 0; d2 < polynomial_order + 1; d2++)
        A(d2,d1) = pow (x, (double) d2);
    }

    if (!A.lu().solve(b, &x))
      return (false);

    for (int i = 0; i < polynomial_order + 1; i++)
      model_coefficients_[6+i] = x[i];

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

    int n = 7;      // 7 unknowns
    int iwa[n];

    int lwa = m * n + 5 * n + m;
    double *wa = new double[lwa];

    // Set the initial solution
    double x[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if ((int)model_coefficients_.size () == n)
      for (int d = 0; d < n; d++)
        x[d] = model_coefficients_.at (d);

    // Set tol to the square root of the machine. Unless high solutions are required, these are the recommended settings.
    double tol = sqrt (dpmpar (1));

    // Optimize using forward-difference approximation LM
    int info = lmdif1 (&sample_consensus::SACModelRotational::functionToOptimize, this, m, n, x, fvec, tol, iwa, wa, lwa);

    // Compute the L2 norm of the residuals
    ROS_DEBUG ("LM solver finished with exit code %i, having a residual norm of %g. \nInitial solution: %g %g %g %g %g %g %g \nFinal solution: %g %g %g %g %g %g %g",
               info, enorm (m, fvec), model_coefficients_.at (0), model_coefficients_.at (1), model_coefficients_.at (2), model_coefficients_.at (3),
               model_coefficients_.at (4), model_coefficients_.at (5), model_coefficients_.at (6), x[0], x[1], x[2], x[3], x[4], x[5], x[6]);

    refit_coefficients.resize (n);
    for (int d = 0; d < n; d++)
      refit_coefficients[d] = x[d];

    free (wa); free (fvec);
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
    SACModelRotational::functionToOptimize (void *p, int m, int n, const double *x, double *fvec, int iflag)
  {
    SACModelRotational *model = (SACModelRotational*)p;

    std::vector<double> line_coefficients (6);
    for (unsigned int d = 0; d < 6; d++)
      line_coefficients[d] = x[d];

    for (int i = 0; i < m; i++)
      // dist = f - r
      fvec[i] = cloud_geometry::distances::pointToLineDistance (model->cloud_->pts[model->tmp_inliers_->at (i)], line_coefficients) - x[6];

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
                cloud_geometry::distances::pointToLineDistance (cloud_->pts.at (*it), model_coefficients_) - model_coefficients_[6]
               ) > threshold)
        return (false);

    return (true);
  }
}

