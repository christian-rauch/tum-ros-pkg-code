/* 
 * Copyright (c) 2010, Nico Blodow <blodow@cs.tum.edu>
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
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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
 */

#ifndef IAS_SAMPLE_CONSENSUS_SACMODELROTATIONAL_H_
#define IAS_SAMPLE_CONSENSUS_SACMODELROTATIONAL_H_

#include <point_cloud_mapping/sample_consensus/sac_model.h>
#include <triangle_mesh/TriangleMesh.h>
#include <mapping_msgs/PolygonalMap.h>

namespace ias_sample_consensus
{
  /** \brief A Sample Consensus Model class for 3D plane segmentation.
    */
  class SACModelRotational : public sample_consensus::SACModel
  {
    public:
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Constructor for SACModelRotational. */
      SACModelRotational (boost::shared_ptr<mapping_msgs::PolygonalMap> pmap) : pmap_(pmap) 
      { 
        nx_idx_ = ny_idx_ = nz_idx_ = -1;
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Destructor for SACModelRotational. */
      virtual ~SACModelRotational () { }

      virtual void getSamples (int &iterations, std::vector<int> &samples);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Test whether the given model coefficients are valid given the input point cloud data.
        * \param model_coefficients the model coefficients that need to be tested
        * \todo implement this
        */
      bool testModelCoefficients (const std::vector<double> &model_coefficients) { return true; }

      virtual bool computeModelCoefficients (const std::vector<int> &samples);

      virtual void refitModel (const std::vector<int> &inliers, std::vector<double> &refit_coefficients);
      virtual void getDistancesToModel (const std::vector<double> &model_coefficients, std::vector<double> &distances);
      virtual void selectWithinDistance (const std::vector<double> &model_coefficients, double threshold, std::vector<int> &inliers);

      virtual void projectPoints (const std::vector<int> &inliers, const std::vector<double> &model_coefficients, sensor_msgs::PointCloud &projected_points);

      virtual void projectPointsInPlace (const std::vector<int> &inliers, const std::vector<double> &model_coefficients);
      virtual bool doSamplesVerifyModel (const std::set<int> &indices, double threshold);

      static int functionToOptimize (void *p, int m, int n, const double *x, double *fvec, int iflag);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Return an unique id for this model (666 for now). */
      virtual int getModelType () { return (666); }
      
      void samplePointsOnRotational (const std::vector<double> modelCoefficients, std::vector<int> inliers, boost::shared_ptr<triangle_mesh::TriangleMesh>);
      static int functionToOptimizeAxis (void *p, int m, int n, const double *x, double *fvec, int iflag);
      bool MinimizeAxisDistancesToSamples (const std::vector<int> samples, std::vector<double> &model_coefficients, double &err);
      double PointToRotationalDistance (const std::vector<double> &model_coefficients, const geometry_msgs::Point32 &p);
      bool EstimateAxisFromSamples (const std::vector<int> samples, std::vector<double> &model_coefficients);
      bool EstimateContourFromSamples (const std::vector<int> samples, std::vector<double> &model_coefficients);
      bool RefitAxis (const std::vector<int> &inliers, std::vector<double> &refit_coefficients);
      void RefitContour (const std::vector<int> &inliers, std::vector<double> &refit_coefficients);
      boost::shared_ptr<mapping_msgs::PolygonalMap> pmap_;
      int nx_idx_;
      int ny_idx_;
      int nz_idx_;
      const std::vector<int> *tmp_inliers_;
  };
}

#endif
