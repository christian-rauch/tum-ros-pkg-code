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

#ifndef _SAMPLE_CONSENSUS_SACMODELCYLINDER_H_
#define _SAMPLE_CONSENSUS_SACMODELCYLINDER_H_
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <mapping_msgs/PolygonalMap.h>

#include <point_cloud_mapping/kdtree/kdtree_ann.h>
#include <point_cloud_mapping/sample_consensus/sac_model.h>
#include <point_cloud_mapping/sample_consensus/model_types.h>

/** \brief Define the maximum number of iterations for collinearity checks */
#define MAX_ITERATIONS_COLLINEAR 1000

namespace sample_consensus
{
  /// General datastructure for passing points and samples to the LM optimizer
  struct LMStrucData
  {
    class SACModelRotational* model;
    sensor_msgs::PointCloud *cloud;
    std::vector<int> samples;
    int nx_idx_;
    int ny_idx_;
    int nz_idx_;
  };
  std::pair<double,double> getMinMaxK (sensor_msgs::PointCloud &cloud, std::vector<double> model_coefficients, std::vector<int> inliers);
  /** \brief A Sample Consensus Model class for cylinder segmentation.
    */
  class SACModelRotational : public SACModel
  {
    public:
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Constructor for base SACModelRotational. */
      SACModelRotational (std::vector<std::vector<std::vector<bool> > > &free_voxels, geometry_msgs::Point32 min, geometry_msgs::Point32 ndivs, geometry_msgs::Point32 leaf_width, mapping_msgs::PolygonalMap &polymap) 
        : pmap (polymap)
        , occupancy_min(min)
        , occupancy_ndivs(ndivs)
        , occupancy_leaf_width(leaf_width)
        , occupancy_lookup (free_voxels)
        { nx_idx_ = ny_idx_ = nz_idx_ = -1; polynomial_order = 3; }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Destructor for base SACModelRotational. */
      virtual ~SACModelRotational () { }

      virtual void getSamples (int &iterations, std::vector<int> &samples);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Test whether the given model coefficients are valid given the input point cloud data.
        * \param model_coefficients the model coefficients that need to be tested
        * \todo implement this
        */
      bool testModelCoefficients (const std::vector<double> &model_coefficients) { return true; }
      
      /** \brief Compute the distance of a point to a rotational object
       *  \param p point in question
       *  \param model_coefficients model coefficients of the rot. object
       *  \param polynomial_order order of polynomial of contour
       */ 
      static double pointToRotationalDistance (const geometry_msgs::Point32 &p, const std::vector<double> &model_coefficients, const int &polynomial_order);

      virtual bool computeModelCoefficients (const std::vector<int> &samples);

      virtual void refitModel (const std::vector<int> &inliers, std::vector<double> &refit_coefficients);
      virtual bool refitModelNoAxis (const std::vector<int> &inliers, std::vector<double> &refit_coefficients);
      virtual bool refitAxis (const std::vector<int> &inliers, std::vector<double> &refit_coefficients);
      virtual void refitAxisGoodLevmar (const std::vector<int> &inliers, std::vector<double> &refit_coefficients);
      virtual void getDistancesToModel (const std::vector<double> &model_coefficients, std::vector<double> &distances);
      virtual void selectWithinDistance (const std::vector<double> &model_coefficients, double threshold, std::vector<int> &inliers);

      virtual void projectPoints (const std::vector<int> &inliers, const std::vector<double> &model_coefficients, sensor_msgs::PointCloud &projected_points);

      virtual void projectPointsInPlace (const std::vector<int> &inliers, const std::vector<double> &model_coefficients);
      virtual bool doSamplesVerifyModel (const std::set<int> &indices, double threshold);

      static int functionToOptimizeNoAxis (void *p, int m, int n, const double *x, double *fvec, int iflag);
      static int functionToOptimize (void *p, int m, int n, const double *x, double *fvec, int iflag);
      static int functionToOptimizeAxis (void *p, int m, int n, const double *x, double *fvec, int iflag);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Return an unique id for this model (SACMODEL_CYLINDER). */
      virtual int getModelType () { return (SACMODEL_CYLINDER); }
      sensor_msgs::PointCloud samplePointsOnRotational (const std::vector<double> modelCoefficients, std::pair<double,double> minmaxK, std::vector<int> inliers);
      bool freespace (geometry_msgs::Point32 p);
      double computeScore (const std::vector<double> &modelCoefficients, std::pair<double,double> minmaxK, std::vector<int> inliers, sensor_msgs::PointCloud &cloud_synth, double threshold);

      mapping_msgs::PolygonalMap &pmap;
    private:
      /** \brief The order of the polynomial to be fitted */
      int polynomial_order;
      /** \brief The coordinates of point normals in the dataset. */
      int nx_idx_, ny_idx_, nz_idx_;
      /** \brief temporary pointer to a list of given indices for refitModel () */
      const std::vector<int> *tmp_inliers_;
      geometry_msgs::Point32 occupancy_min;
      geometry_msgs::Point32 occupancy_ndivs;
      geometry_msgs::Point32 occupancy_leaf_width;
      std::vector<std::vector<std::vector<bool> > > &occupancy_lookup;
  };
}
#endif

