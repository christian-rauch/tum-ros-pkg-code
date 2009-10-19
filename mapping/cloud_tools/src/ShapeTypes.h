#ifndef _SHAPE_TYPES_H
#define	_SHAPE_TYPES_H

#include <set>
#include <vector>
#include <ias_table_msgs/TableObject.h>
#include <point_cloud_mapping/geometry/point.h>
#include <sensor_msgs/PointCloud.h>
class ShapeType
{
  public:
    bool CheckShape (
        sensor_msgs::PointCloudConstPtr points,
        std::vector<int> samples,
        int idx_normal,
        std::vector<double> &coeffs)
    {
      return false;
    }

//    std::RefitToInliers ();
    static std::set<int> GetInliers (sensor_msgs::PointCloudConstPtr points,
                                     std::vector<double> shape_coeffs, 
                                     double epsilon, int idx_normal)
    {
      return std::set<int>();
    }
};

class ShapeTypePlane : public ShapeType
{
  public:
    static bool CheckShape (sensor_msgs::PointCloudConstPtr points,
                            std::vector<int> samples,
                            int idx_normal,
                            std::vector<double> coeffs)
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
                                     std::vector<double> shape_coeffs, 
                                     double epsilon, int idx_normal)
    {
      std::set<int> res;
      for (unsigned int i = 0; i < points->points.size(); i++)
      {
        if (fabs (shape_coeffs[0] * points->points[i].x +
                  shape_coeffs[1] * points->points[i].y +
                  shape_coeffs[2] * points->points[i].z + shape_coeffs[3]) < epsilon)
            res.insert (i);
      }
      return res;
    }
};

#endif

