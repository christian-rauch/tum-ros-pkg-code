#ifndef _SHAPE_TYPES_H
#define	_SHAPE_TYPES_H

#include <set>
#include <vector>
#include <ias_table_msgs/TableObjectReconstructed.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/angles.h>
#include <sensor_msgs/PointCloud.h>

struct ShapeType
{
    bool CheckShape (const sensor_msgs::PointCloudConstPtr points,
                     std::vector<int> samples,
                     int idx_normal, double angle_thresh,
                     std::vector<double> &coeffs)
    {
      return false;
    }

//  static RefitToInliers ();
    
    static std::set<int> GetInliers (const sensor_msgs::PointCloudConstPtr points,
                                     std::vector<int> indices,
                                     std::vector<double> shape_coeffs, 
                                     double epsilon, int idx_normal)
    {
      return std::set<int>();
    }
};

struct ShapeTypePlane : ShapeType
{
    static bool CheckShape (sensor_msgs::PointCloudConstPtr points,
                            std::vector<int> samples,
                            int idx_normal, double angle_thresh,
                            std::vector<double> &coeffs)
    {
      geometry_msgs::Point32 v1;
      geometry_msgs::Point32 v2;
    //  if (Are3DPointsCollinear (dataset->points[samples[0]], dataset->points[samples[1]], dataset->points[samples[2]]))
    //    return false;

      // calc normal for plane through the 3 points
      v1.x = points->points[samples[1]].x - points->points[samples[0]].x;
      v2.x = points->points[samples[2]].x - points->points[samples[0]].x;
      v1.y = points->points[samples[1]].y - points->points[samples[0]].y;
      v2.y = points->points[samples[2]].y - points->points[samples[0]].y;
      v1.z = points->points[samples[1]].z - points->points[samples[0]].z;
      v2.z = points->points[samples[2]].z - points->points[samples[0]].z;
      
      cloud_geometry::normalizePoint (v1);
      cloud_geometry::normalizePoint (v2);
      
      geometry_msgs::Point32 plane_normal = cloud_geometry::cross (v1, v2);  ///this can result in a division by zero error
    //   norm_b = sqrt (plane_normal.x*plane_normal.x + plane_normal.y*plane_normal.y + plane_normal.z*plane_normal.z);
    //   // normalize it
    //   if (norm_b == 0) // this means the 3 points were collinear
    //     return false;

    //   for (int set_idx = 0; set_idx < 3; set_idx++)
    //   {
    //     angle = (acos (
    //         (plane_normal[0] * dataset->points[samples[set_idx]][idx_normal + 0]) +
    //         (plane_normal[1] * dataset->points[samples[set_idx]][idx_normal + 1]) +
    //         (plane_normal[2] * dataset->points[samples[set_idx]][idx_normal + 2]) ));
    //     if (angle > angle_thresh && (M_PI - angle) > angle_thresh)
    //       return false;
    //   }
      coeffs.resize(4);
      coeffs[0] = plane_normal.x;
      coeffs[1] = plane_normal.y;
      coeffs[2] = plane_normal.z;
      coeffs[3] = - ( (plane_normal.x * points->points[samples[0]].x) +
                      (plane_normal.y * points->points[samples[0]].y) +
                      (plane_normal.z * points->points[samples[0]].z) );
      return true;
    }
    
    static std::set<int> GetInliers (sensor_msgs::PointCloudConstPtr points,
                                     std::vector<int> indices,
                                     std::vector<double> shape_coeffs, 
                                     double epsilon, int idx_normal)
    {
      std::set<int> res;
      for (unsigned int i = 0; i < indices.size(); i++)
      {
        if (fabs (shape_coeffs[0] * points->points[indices[i]].x +
                  shape_coeffs[1] * points->points[indices[i]].y +
                  shape_coeffs[2] * points->points[indices[i]].z + shape_coeffs[3]) < epsilon)
            res.insert (i);
      }
      return res;
    }
};

struct ShapeTypeSphere : ShapeType
{
    #define THRESH_RADIUS_SPHERE 0.5
    static bool CheckShape (sensor_msgs::PointCloudConstPtr points,
                            std::vector<int> samples,
                            int idx_normal, double angle_thresh,
                            std::vector<double> &coeffs)
    {
      // See http://local.wasp.uwa.edu.au/~pbourke/geometry/spherefrom4/ for more details
      Eigen::Matrix4d temp;
      // Find determinant M11
      for (int i = 0; i < 4; i++)
      {
        temp (i, 0) = points->points[samples[i]].x;
        temp (i, 1) = points->points[samples[i]].y;
        temp (i, 2) = points->points[samples[i]].z;
        temp (i, 3) = 1;
      }
      double m11 = temp.determinant ();
  
      if (m11 == 0)
        return false;
      
      #define SQR(x) ((x)*(x))
      // Find determinant M12
      for (int i = 0; i < 4; i++)
        temp (i, 0) = SQR (points->points[samples[i]].x) + 
                      SQR (points->points[samples[i]].y) + 
                      SQR (points->points[samples[i]].z);
      double m12 = temp.determinant ();
      
      // Find determinant M13
      for (int i = 0; i < 4; i++)
      {
        temp (i, 0) = points->points[samples[i]].x;
        temp (i, 1) = SQR (points->points[samples[i]].x) + 
                      SQR (points->points[samples[i]].y) + 
                      SQR (points->points[samples[i]].z);
      }
      double m13 = temp.determinant ();
      
      // Find determinant M14
      for (int i = 0; i < 4; i++)
      {
        temp (i, 1) = points->points[samples[i]].x;
        temp (i, 2) = SQR (points->points[samples[i]].x) + 
                      SQR (points->points[samples[i]].y) + 
                      SQR (points->points[samples[i]].z);
      }
      double m14 = temp.determinant ();
      
      // Find determinant M15
      for (int i = 0; i < 4; i++)
      {
        temp (i, 0) = SQR (points->points[samples[i]].x) + 
                      SQR (points->points[samples[i]].y) + 
                      SQR (points->points[samples[i]].z);
        temp (i, 1) = points->points[samples[i]].x;
        temp (i, 2) = points->points[samples[i]].y;
        temp (i, 3) = points->points[samples[i]].z;
      }
      double m15 = temp.determinant ();
      
      // Center (x , y, z)
      coeffs.resize(4);
      coeffs[0] = 0.5 * m12 / m11;
      coeffs[1] = 0.5 * m13 / m11;
      coeffs[2] = 0.5 * m14 / m11;
      // Radius
      coeffs[3] = sqrt (SQR (coeffs[0]) + 
                        SQR (coeffs[1]) + 
                        SQR (coeffs[2]) - m15 / m11);
  
      if (coeffs[3] > THRESH_RADIUS_SPHERE)
        return false;
      else
      {
        geometry_msgs::Point32 normal;
        geometry_msgs::Point32 spherenormal;

        // if normals deviate, don't accept
        for (int i = 0; i < 4 ; i++)
        {
          spherenormal.x = points->points[samples[i]].x - coeffs[0];
          spherenormal.y = points->points[samples[i]].y - coeffs[1];
          spherenormal.z = points->points[samples[i]].z - coeffs[2];
          
          normal.x = points->channels[idx_normal + 0].values[samples[i]];
          normal.y = points->channels[idx_normal + 1].values[samples[i]];
          normal.z = points->channels[idx_normal + 2].values[samples[i]];

          double angle = cloud_geometry::angles::getAngleBetweenPlanes (spherenormal, normal);
          if (angle > angle_thresh)
          {
            return false;
          }
        }
      }
      return true;
    }
    
    static std::set<int> GetInliers (sensor_msgs::PointCloudConstPtr points,
                                     std::vector<int> indices,
                                     std::vector<double> shape_coeffs, 
                                     double epsilon, int idx_normal)
    {
      std::set<int> res;
      for (unsigned int i = 0; i < indices.size(); i++)
      {
        geometry_msgs::Point32 d;
        d.x = points->points[indices[i]].x - shape_coeffs[0];
        d.y = points->points[indices[i]].y - shape_coeffs[1];
        d.z = points->points[indices[i]].z - shape_coeffs[2];
        if (fabs (sqrt (d.x * d.x +
                        d.y * d.y +
                        d.z * d.z - shape_coeffs[3])) < epsilon)
            res.insert (i);
      }
      return res;
    }
};

struct ShapeTypeRotational : ShapeType
{
    #define polynomial_order 3
    static bool refitAxis (const sensor_msgs::PointCloudConstPtr points,
                           std::vector<int> inliers,
                           std::vector<double> &refit_coefficients)
    {
      std::vector<double> orig_coefficients = refit_coefficients;
      if (inliers.size () == 0)
      {
        return false;
      }
      if (refit_coefficients.size () == 0)
      {
        return false;
      }

      int m = inliers.size ();
      double *fvec = new double[m];
      int n = 6;      // 6 unknowns
      int iwa[n];
      int lwa = m * n + 5 * n + m;
      double *wa = new double[lwa];
      // Set the initial solution
      double x[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      for (int d = 0; d < n; d++)
        x[d] = refit_coefficients.at (d);
      double tol = sqrt (dpmpar (2));
      int info = lmdif1 (&sample_consensus::SACModelRotational::functionToOptimizeAxis, 
                         this, m, n, x, fvec, tol, iwa, wa, lwa);
      info=info;
      for (int d = 0; d < n; d++)
        refit_coefficients[d] = x[d];

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
        b[d1] = cloud_geometry::distances::pointToLineDistance 
                   (cloud_->points.at (inliers.at (d1)), p1, axis);
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

      return false;
    }
    
    static bool CheckShape (sensor_msgs::PointCloudConstPtr points,
                            std::vector<int> samples,
                            int idx_normal, double angle_thresh,
                            std::vector<double> &coeffs)
    {
      coeffs.resize (6+1+polynomial_order);

      geometry_msgs::Point32 centroid;
      cloud_geometry::nearest::computeCentroid (points, samples, centroid);

      // use the centroid as an initial guess for a point on the axis,
      // and the z axis as a rotation axis
      // this axis will get optimized by the first optimization.
      coeffs[0] = centroid.x;
      coeffs[1] = centroid.y;
      coeffs[2] = centroid.z;
      coeffs[3] = centroid.x + 0.0;
      coeffs[4] = centroid.y + 0.0;
      coeffs[5] = centroid.z + 1.0;

      Point32 p1;
      Point32 p2;
      geometry_msgs::Polygon polygon;
     
      if (!refitAxis (points, samples, coeffs))
        return false;
      
      return true;
    }
    
    static std::set<int> GetInliers (sensor_msgs::PointCloudConstPtr points,
                                     std::vector<int> indices,
                                     std::vector<double> shape_coeffs, 
                                     double epsilon, int idx_normal)
    {
      std::set<int> res;
      for (unsigned int i = 0; i < indices.size(); i++)
      {
        geometry_msgs::Point32 p;
        p.x = points->points[indices[i]].x;
        p.y = points->points[indices[i]].y;
        p.z = points->points[indices[i]].z;
        if (2*epsilon < epsilon)
            res.insert (i);
      }
      return res;
    }
};
#endif

