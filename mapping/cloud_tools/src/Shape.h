#ifndef _SHAPE_H
#define	_SHAPE_H

#include "ShapeTypes.h"
#include "param.h"

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

