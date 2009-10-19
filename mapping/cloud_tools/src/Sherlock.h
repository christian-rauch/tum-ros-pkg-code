#ifndef _SHERLOCK_H
#define	_SHERLOCK_H

#include <stdlib.h>
#include <vector>
#include <set>
#include <algorithm>

#include <sensor_msgs/PointCloud.h>

#include "Shape.h"

class Sherlock
{
public:
    Sherlock ();
    ~Sherlock ();
    void SetData (const sensor_msgs::PointCloudConstPtr points);
    void SetShapeTypes (unsigned int types);
    int DetectShapes ();
    void FindSuspects ();
    Shape* GetPrimeSuspect ();
    void EliminatePrimeSuspect ();
    void AddSuspect (Shape *shape);
    double pFindNoBetter (int n);
    bool pFindNoBetterFalse (int n);
    bool pFindNoBetterTrue (int n);

    sensor_msgs::PointCloudConstPtr points;
    Shape* best;
    int best_index;
    int nr_trials;
    int nr_points_left;
    std::set<int> points_used;
    std::vector<Shape*> shapes;
    std::vector<Shape*> suspects;
    std::vector<ShapeType> shapetypes;
};

#endif	/* _SHERLOCK_H */

