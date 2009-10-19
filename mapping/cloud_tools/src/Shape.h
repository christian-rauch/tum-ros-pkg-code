#ifndef _SHAPE_H
#define	_SHAPE_H

#include <set>
#include <vector>
#include <sensor_msgs/PointCloud.h>

#include "param.h"

class ShapeType
{
  public:
    virtual bool CheckShape (
        sensor_msgs::PointCloudConstPtr points,
        std::vector<double> coeffs,
        int idx_normal,
        std::vector<int> samples,
        std::set<int> inliers,
        int type,
        unsigned int score) = 0;

//    std::RefitToInliers ();
    virtual std::set<int> GetInliers () = 0;

  
};


enum {
  SHAPE_TYPE_FIRST = 0,
  SHAPE_TYPE_PLANE = 0,
  SHAPE_TYPE_SPHERE,
  SHAPE_TYPE_CYLINDER,
  SHAPE_TYPE_CONE,
  SHAPE_TYPE_TORUS,
  SHAPE_TYPE_LAST = 2 // TODO: Increase when implementing more shape types
};

class Shape 
{
public:
  Shape (sensor_msgs::PointCloudConstPtr pts, std::vector<int> sample, int shape_type, int idx_normal);
  ~Shape ();

  friend bool operator< (Shape &s1, Shape &s2);
  friend bool operator> (Shape &s1, Shape &s2);
  
  int RemovePoints (std::set<int> p);
  unsigned int GetScore ();
  int GetType ();
  std::vector<double> GetCoeffs ();
  std::set<int> GetInliers ();
  std::set<int> GetPlaneInliers (std::vector<double> shape_coeffs, 
                                 double epsilon, int idx_normal);
  bool CheckShapePlane();
  bool CheckShape();
  void RefitToInliers ();
  void PrintCoeffs();

  //private:

  sensor_msgs::PointCloudConstPtr points;
  int idx_normal;

  std::vector<double> coeffs;
  std::vector<int> samples;
  std::set<int> inliers;
  
  int type;
  unsigned int score;
};

#endif	/* _SHAPE_H */

