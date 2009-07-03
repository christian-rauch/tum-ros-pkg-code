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
#include <point_cloud_mapping/geometry/transforms.h>
#include <robot_msgs/PointCloud.h>
#include <mapping_msgs/PolygonalMap.h>
#include <tf/tf.h>
#include <Eigen/LU>

#include <cminpack.h>

using namespace mapping_msgs;
using namespace robot_msgs;

namespace sample_consensus
{
std::pair<double,double> getMinMaxK (PointCloud &cloud, std::vector<double> model_coefficients, std::vector<int> inliers)
{
  double minimum = FLT_MAX;
  double maximum = -FLT_MAX;
    
  robot_msgs::Point32 axis;   // axis direction vector
  robot_msgs::Point32 point0; // point on axis
  axis.x = model_coefficients[3] - model_coefficients[0];
  axis.y = model_coefficients[4] - model_coefficients[1];
  axis.z = model_coefficients[5] - model_coefficients[2];

  point0.x = model_coefficients[0];
  point0.y = model_coefficients[1];
  point0.z = model_coefficients[2];


  //double norm = sqrt(axis.x*axis.x + axis.y*axis.y + axis.z*axis.z);

  //  ANNcoord k0 = (0.0 - DOT_PROD_3D (p, p21)) / DOT_PROD_3D (p21, p21);
  for (unsigned int i = 0; i < inliers.size(); i++)
    //   for (unsigned int i = 0; i < cloud.pts.size(); i++)
  {
    // "k" is the position (x-value?) of the projection of current point on the rot. axis
#define DOT_PROD_3D(x,y) ((x[0])*(y[0])+(x[1])*(y[1])+(x[2])*(y[2]))
    double k = (cloud_geometry::dot (cloud.pts[inliers[i]], axis) 
        - cloud_geometry::dot (point0, axis)) 
      / cloud_geometry::dot (axis, axis);
    //    k = (k - k0);
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
    PointCloud ret;

    // create 100 points from min_k to max_k on the outline of the rotational volume

    double p21[3];
    p21[0] = modelCoefficients[3] - modelCoefficients[0];
    p21[1] = modelCoefficients[4] - modelCoefficients[1];
    p21[2] = modelCoefficients[5] - modelCoefficients[2];
#define SQR(x) ((x)*(x))
#define SQR_VECT_LENGTH(x) (SQR(x[0])+SQR(x[1])+SQR(x[2]))
#define SQR_NORM(x) SQR_VECT_LENGTH(x)
    
    double norm = sqrt(SQR_NORM (p21)); 
    
    //cross product with (0,0,1) is cheap:
    Point32 rotationaxis;
    rotationaxis.x = p21[1]/norm;
    rotationaxis.y = -p21[0]/norm;
    rotationaxis.z = 0.0;
#define RAD2DEG(r) (double)((r) * 180.0 / M_PI)
    double rotationangle = - RAD2DEG ( acos (p21[2] / norm) );  // dot product of a with (0,0,1) is a[2]

#define DOT_PROD_3D(x,y) ((x[0])*(y[0])+(x[1])*(y[1])+(x[2])*(y[2]))
    double k0 = (0.0 - DOT_PROD_3D (modelCoefficients, p21)) / DOT_PROD_3D (p21, p21);
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
ROS_WARN ("%f %f", X, Y);
      Point32 p;
      for (int j = 0; j < res_radial; j++)
      {
        p.x = Y;
        p.y = 0.0;
        p.z = ((double)i/res_axial)*norm*(minmaxK.second - minmaxK.first);

        Eigen::Matrix3d rotation1, rotation2;
        Point32 z_axis;
        z_axis.x = 0;
        z_axis.y = 0;
        z_axis.z = 1;
        cloud_geometry::transforms::convertAxisAngleToRotationMatrix (z_axis, M_PI*2*j/res_radial, rotation2);
        cloud_geometry::transforms::convertAxisAngleToRotationMatrix (rotationaxis, rotationangle, rotation1);
        Eigen::Vector3d p_0 (p.x, p.y, p.z);
        Eigen::Vector3d q_0 = rotation1 * rotation2 * p_0;

//         rotateUp (p, PI*(j/60.0)) 
        //RotateWXYZ (p, rotationangle, rotationaxis);
        p.x = q_0[0] + modelCoefficients[0] + minmaxK.first * p21[0];
        p.y = q_0[1] + modelCoefficients[1] + minmaxK.first * p21[1];
        p.z = q_0[2] + modelCoefficients[2] + minmaxK.first * p21[2];
        
        ret.pts.push_back (p);
      }
    }
    return ret;
  }


  double SACModelRotational::computeScore 
      (const std::vector<double> &modelCoefficients, std::pair<double,double> minmaxK, std::vector<int> inliers, PointCloud &cloud_synth, double threshold)
  {
      
    ROS_ERROR ("inliers: %i\n", inliers.size());
      std::cerr << "SCORE FOR THE FOLLOWING COEFFS: ";
      for (unsigned int i = 0; i < modelCoefficients.size(); i++)
        std::cerr << modelCoefficients[i] << " "; 
      std::cerr << std::endl;
    ROS_ERROR ("coefficients: %g %g %g %g %g %g %g %g %g %g \n",
               modelCoefficients.at (0), modelCoefficients.at (1), modelCoefficients.at (2), modelCoefficients.at (3),
               modelCoefficients.at (4), modelCoefficients.at (5), modelCoefficients.at (6), 
               modelCoefficients.at (7), modelCoefficients.at (8), modelCoefficients.at (9)); 
    static int count = 0;
    std::cerr << "before sampling" << std::endl;
    PointCloud synth_points = samplePointsOnRotational (modelCoefficients, minmaxK, inliers);
    std::cerr << "after sampling, kdtree for " << synth_points.pts.size() << std::endl;
    std::vector <int> marks (synth_points.pts.size());

    // create kd-tree from synth_points
    
//     cloud_kdtree::KdTreeANN *kd_tree = new cloud_kdtree::KdTreeANN (synth_points);
//     // go through inliers, mark synth_points that are close as "cat.1"
//     
//     for (unsigned int i = 0; i < inliers.size (); i++)
//     {
//       std::vector<int> k_indices;
//       std::vector<float> k_distances;
//       kd_tree->radiusSearch (cloud_->pts[inliers[i]] , threshold*2, k_indices, k_distances);
//       for (unsigned int j = 0; j < k_indices.size(); j++)
//       {
//         marks[k_indices[j]] = 1;
//       }
//     }
    int mark_count_1 = 0;
    int mark_count_2 = 0;
    for (unsigned int j = 0; j < marks.size(); j++)
    {
      if (marks[j] == 1)
        mark_count_1++;
    }
    std::cerr << "after marking" << std::endl;

    // go through unmarked synth_points, mark as "cat.2" if not in free space
    for (unsigned int j = 0; j < marks.size(); j++)
    {
      if (marks[j] == 0)
        if (!freespace (synth_points.pts[j]))
          mark_count_2++;
    }

    // return (#cat1 + #cat2) / synth_points.pts.size();
    
    //    int cur = pmap.polygons.size ();
    //    pmap.polygons.resize(pmap.polygons.size()+1);
    for (unsigned int i = 0; i < inliers.size (); i++)
    {
      cloud_synth.chan[0].vals.push_back (count);
      cloud_synth.pts.push_back (cloud_->pts[inliers[i]]);
    }
    for (unsigned int i = 0; i < synth_points.pts.size (); i++)
    {
      cloud_synth.chan[0].vals.push_back (count);
      cloud_synth.pts.push_back (synth_points.pts[i]);
    }
//      pmap.polygons[cur].points.push_back (synth_points.pts[i]);
    count++;
    ROS_ERROR ("SAMPLED %i POINTS!!!!!", cloud_synth.pts.size());
    return double(mark_count_1+mark_count_2) / double (synth_points.pts.size ());
  }

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
    
    robot_msgs::Point32 centroid;
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
    robot_msgs::Polygon3D polygon;
    for (unsigned int i = 0; i < samples.size(); i++)
    {
    polygon.points.clear();
      polygon.points.push_back ( cloud_->pts[samples[i]] );
      p1.x = 0.2*cloud_->chan[nx_idx_].vals[samples[i]] + cloud_->pts[samples[i]].x;
      p1.y = 0.2*cloud_->chan[ny_idx_].vals[samples[i]] + cloud_->pts[samples[i]].y;
      p1.z = 0.2*cloud_->chan[nz_idx_].vals[samples[i]] + cloud_->pts[samples[i]].z;
      polygon.points.push_back ( p1 );
      pmap.polygons.push_back (polygon);
      pmap.chan[0].vals.push_back (0);
      pmap.chan[1].vals.push_back (0);
      pmap.chan[2].vals.push_back (0);
    }

//     p1.x = model_coefficients_[0];
//     p1.y = model_coefficients_[1];
//     p1.z = model_coefficients_[2];
//     p2.x = model_coefficients_[3];
//     p2.y = model_coefficients_[4];
//     p2.z = model_coefficients_[5];
//     polygon.points.push_back (p1);
//     polygon.points.push_back (p2);
//     pmap.polygons.push_back (polygon);
//     pmap.chan[0].vals.push_back (0);
//     pmap.chan[1].vals.push_back (0);
//     pmap.chan[2].vals.push_back (0);

    refitAxis (samples, model_coefficients_);
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
    pmap.chan[0].vals.push_back (1);
    pmap.chan[1].vals.push_back (1);
    pmap.chan[2].vals.push_back (1);
//    pmap.polygons.resize (pmap.polygons.size()+1);
//     pmap.polygons[(pmap.polygons.size()-1)].push_back (p1);
//     pmap.polygons[(pmap.polygons.size()-1)].push_back (p2);
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

void
  rot_axis_func (double *p, double *X, int m, int n, void *data)
{
  struct LMStrucData *d = (struct LMStrucData *) data;
  
    std::vector<double> rot_coeff (6);
    for (int d = 0; d < 6; d++)
      rot_coeff[d] = p[d];

    std::vector<double> point_normal_line (6);

    for (int i = 0; i < m; i++)
    {
      point_normal_line[0] = d->cloud->pts[d->samples.at(i)].x;
      point_normal_line[1] = d->cloud->pts[d->samples.at(i)].y;
      point_normal_line[2] = d->cloud->pts[d->samples.at(i)].z;
      
      point_normal_line[3] = d->cloud->chan[d->nx_idx_].vals[d->samples.at(i)];
      point_normal_line[4] = d->cloud->chan[d->ny_idx_].vals[d->samples.at(i)];
      point_normal_line[5] = d->cloud->chan[d->nz_idx_].vals[d->samples.at(i)];
      
      double ll = LineToLineDistance (rot_coeff, point_normal_line, 1e-5); 
      X[i] = ll * ll;
    }
}

  void
    SACModelRotational::refitAxis (const std::vector<int> &inliers, std::vector<double> &refit_coefficients)
  {
    ROS_INFO ("optimizing axis from %i points", inliers.size());
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

     // Compute the L2 norm of the residuals
     ROS_INFO ("LM solver finished with exit code %i, having a residual norm of %g. \nInitial solution: %g %g %g %g %g %g \nFinal solution: %g %g %g %g %g %g ",
                info, enorm (m, fvec), 
                refit_coefficients.at (0), refit_coefficients.at (1), refit_coefficients.at (2), refit_coefficients.at (3),
                refit_coefficients.at (4), refit_coefficients.at (5), x[0], x[1], x[2], x[3], x[4], x[5]);

     //refit_coefficients.resize (n);
     for (int d = 0; d < n; d++)
       refit_coefficients[d] = x[d];

    std::pair<double,double> minmax = getMinMaxK (*cloud_ , refit_coefficients, inliers);
    
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
    SACModelRotational::functionToOptimize (void *p, int m, int n, const double *x, double *fvec, int iflag)
  {
    SACModelRotational *model = (SACModelRotational*)p;
    
    std::vector<double> rot_coeff (6+1+model->polynomial_order);
    for (int d = 0; d < 6+1+model->polynomial_order; d++)
      rot_coeff[d] = x[d];

    for (int i = 0; i < m; i++)
      // dist = f - r
      fvec[i] = pointToRotationalDistance (model->cloud_->pts[model->tmp_inliers_->at (i)], rot_coeff, model->polynomial_order) - x[6];

    return (0);
  }
  
  int
    SACModelRotational::functionToOptimizeAxis (void *p, int m, int n, const double *x, double *fvec, int iflag)
  {
    SACModelRotational *model = (SACModelRotational*)p;
    
    std::vector<double> rot_coeff (6);
    for (int d = 0; d < 6; d++)
      rot_coeff[d] = x[d];

    std::vector<double> point_normal_line (6);

    for (int i = 0; i < m; i++)
    {
      point_normal_line[0] = model->cloud_->pts[model->tmp_inliers_->at(i)].x;
      point_normal_line[1] = model->cloud_->pts[model->tmp_inliers_->at(i)].y;
      point_normal_line[2] = model->cloud_->pts[model->tmp_inliers_->at(i)].z;
      
      point_normal_line[3] = model->cloud_->chan[model->nx_idx_].vals[model->tmp_inliers_->at(i)];
      point_normal_line[4] = model->cloud_->chan[model->ny_idx_].vals[model->tmp_inliers_->at(i)];
      point_normal_line[5] = model->cloud_->chan[model->nz_idx_].vals[model->tmp_inliers_->at(i)];
      
      double ll = 100.0*LineToLineDistance (rot_coeff, point_normal_line, 1e-5); 
      // dist = f - r
      fvec[i] = ll * ll;
//       fvec[i] = pointToRotationalDistance (model->cloud_->pts[model->tmp_inliers_->at (i)], rot_coeff, model->polynomial_order) - x[6];
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
                cloud_geometry::distances::pointToLineDistance (cloud_->pts.at (*it), model_coefficients_) - model_coefficients_[6]
               ) > threshold)
        return (false);

    return (true);
  }
}

