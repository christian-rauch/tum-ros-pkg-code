#include <set>
#include <iterator>
#include <vector>

#include <angles/angles.h>

#include "Shape.h"
// #include "LMmodels.h"
// #include "lm.h"

Shape::Shape(sensor_msgs::PointCloudConstPtr points, std::vector<int> sample, 
             int shape_type, int idx_normal)
{
  this->idx_normal = idx_normal;
  this->points = points;
  this->samples = sample;
  this->type = shape_type;
}

Shape::~Shape()
{
}

void Shape::RefitToInliers ()
{
  return;
// //  std::cerr << endl;
// //  std::cerr << "\tRefitting Model to Inliers using Levenberg-Marquardt minimization." << std::endl;
// //  std::cerr << "    before: " << inliers.size() << " inliers." << std::endl;
// //  
// //  std::cerr << "    after : " << inliers.size() << " inliers." << std::endl;
//   if (type == SHAPE_TYPE_PLANE)
//   {
//     std::cerr << endl;
//     std::cerr << "\tRefitting Plane: " << inliers.size() ;
// //      Model to Inliers using Least-Squares minimization." << std::endl;
// //    std::cerr << "    before: " << inliers.size() << " inliers." << std::endl;
// //    printf ("Using the following initial \nEstimates: %g, %g, %g, %g\n", 
// //          coeffs[0], coeffs[1], coeffs[2], coeffs[3]);

// ////// blablabla
//     //
//     double centroid[] = {0.0,0.0,0.0};
//     for (std::set<int>::iterator it = inliers.begin(); it != inliers.end(); it++)
//       for (int d = 0; d < 3; d++)
//         centroid[d] += dataset->points[*it][d];
//     for (int d = 0; d < 3; d++)
//       centroid[d] /= inliers.size();

//     // Initialize matrix
//     ANNpointArray cov_matrix = annAllocPts (3, 3, 0.0);

//     // Sum of outer products
//     for (int k = 0; k < 3; k++)
//       for (int i = 0; i < 3; i++)
//         for (std::set<int>::iterator it = inliers.begin(); it != inliers.end(); it++)
//         //for (int j = 0; j < dataset->header.nr_points; j++)
//           // cov_matrix[k][i] += points_ct[k][j] * points_c[j][i];
//           cov_matrix[k][i] += (dataset->points[*it][k] - centroid[k]) * (dataset->points[*it][i] - centroid[k]);

//     ANNpoint eVal = NULL;
//     ANNpointArray eVec = NULL;

//     // Eigenanalysis (compute the eigenvalues and eigenvectors)
//     eig_sym (cov_matrix, eVal, eVec, 3);

//     
//     // The normalized normal of the plane corresponds to the eigenvector of the least eigenvalue
//     ANNpoint nc = annAllocPt (5);
//     //double nlen = sqrt (eVec[0][0]*eVec[0][0] + eVec[0][1]*eVec[0][1] + eVec[0][2]*eVec[0][2]);
//     double nlen = sqrt (SQR_VECT_LENGTH (eVec[0]));
//     nc[0] = eVec[0][0] / nlen;
//     nc[1] = eVec[0][1] / nlen;
//     nc[2] = eVec[0][2] / nlen;

//     // Hessian form (D = nc . p_plane (centroid here) + p)
//     nc[3] = -1 * (nc[0] * centroid[0] + nc[1] * centroid[1] + nc[2] * centroid[2]);

//     // Compute the curvature estimate
//     nc[4] = eVal[0] / (eVal[0] + eVal[1] + eVal[2]);
//     ///nc[4] = eVal[0] / (eVal[1] * eVal[2]);

//     annDeallocPts (eVec);
//     annDeallocPt (eVal);
// //    annDeallocPt (centroid);
//     annDeallocPts (cov_matrix);

// /////
//     ANNpoint p = nc;//cANN::computeNormalCurvature (dataset->points, inliers.size (), 3, std::vector<int>(inliers.begin(),inliers.end()));
//     for (int i = 0; i < 4; ++i)
//       coeffs[i] = p[i];
//     annDeallocPt(p);

// //    printf ("Least Sqaures done, \nSolution:  ");
// //    for (int i = 0; i < 4; i++)
// //      printf ("%.7g ", coeffs[i]);
// //      printf ("\n ");
//     
//     OctreeIntersectionTest *test;
//     test = new OctreeIntersectionTestPlane (dataset->points);

//     ANNpoint BoxOrigin = annAllocPt(3);
//     for (int d = 0; d < 3; d++)
//       BoxOrigin[d] = dataset->header.minPD[d];
//     test->init(coeffs, BoxOrigin, epsilon);
//     std::set<int> v = dataset->octree->IntersectWithShape (test, dataset->header.minPD, dataset->header.maxPD, (gui_mode >= 4) );

//     inliers = test->GetInliers (v, coeffs, epsilon, angle_thresh, idx_normal);

//     std::cerr << " --> " << inliers.size() << " inliers." << std::endl;
// //    std::cerr << "    after : " << inliers.size() << " inliers." << std::endl;
//   }
//   else if(type == SHAPE_TYPE_SPHERE)
//   {
//     std::cerr << endl;
//     std::cerr << "\tRefitting Sphere : " << inliers.size();
// //      Model to Inliers using Levenberg-Marquardt minimization." << std::endl;
// //    std::cerr << "    before: " << inliers.size() << " inliers." << std::endl;
//     
//     // set up data structure for Levenberg Marquardt fitting
//     LMStrucData data;
//     data.points  = dataset->points;
//     data.samples = std::vector<int>(inliers.begin(), inliers.end());

//     // compute better coeffs for the given inliers and initial guess
//     RefitLevMarSphere (&data, coeffs);
//     
//     // recompute inliers
//     OctreeIntersectionTest *test;
//     test = new OctreeIntersectionTestSphere (dataset->points);

//     ANNpoint BoxOrigin = annAllocPt(3);
//     for (int d = 0; d < 3; d++)
//       BoxOrigin[d] = dataset->header.minPD[d];
//     test->init(coeffs, BoxOrigin, epsilon);
//     std::set<int> v = dataset->octree->IntersectWithShape (test, dataset->header.minPD, dataset->header.maxPD, (gui_mode >= 4) );

//     inliers = test->GetInliers (v, coeffs, epsilon, angle_thresh, idx_normal);

//     std::cerr << " --> " << inliers.size() << " inliers." << std::endl;
// //    std::cerr << "    after : " << inliers.size() << " inliers." << std::endl;
//   }
//   else if(type == SHAPE_TYPE_CYLINDER)
//   {
//     std::cerr << endl;
//     std::cerr << "\tRefitting Cylinder : " << inliers.size() ;
// //      Model to Inliers using Levenberg-Marquardt minimization." << std::endl;
// //    std::cerr << "    before: " << inliers.size() << " inliers." << std::endl;
//     
//     // set up data structure for Levenberg Marquardt fitting
//     LMStrucData data;
//     data.points  = dataset->points;
//     data.samples = std::vector<int>(inliers.begin(), inliers.end());

//     // compute better coeffs for the given inliers and initial guess
//     RefitLevMarCylinder (&data, coeffs);
//     
//     // recompute inliers
//     OctreeIntersectionTest *test;
//     test = new OctreeIntersectionTestCylinder (dataset->points);

//     ANNpoint BoxOrigin = annAllocPt(3);
//     for (int d = 0; d < 3; d++)
//       BoxOrigin[d] = dataset->header.minPD[d];
//     test->init(coeffs, BoxOrigin, epsilon);
//     std::set<int> v = dataset->octree->IntersectWithShape (test, dataset->header.minPD, dataset->header.maxPD, (gui_mode >= 4) );

//     inliers = test->GetInliers (v, coeffs, epsilon, angle_thresh, idx_normal);

//     std::cerr << " --> " << inliers.size() << " inliers." << std::endl;
// //    std::cerr << "    after : " << inliers.size() << " inliers." << std::endl;
//   }
//   else if(type == SHAPE_TYPE_CONE)
//   {
//     std::cerr << endl;
//     std::cerr << "\tRefitting Cone : " << inliers.size();
// //      Model to Inliers using Levenberg-Marquardt minimization." << std::endl;
// //    std::cerr << "    before: " << inliers.size() << " inliers." << std::endl;
//     
//     // set up data structure for Levenberg Marquardt fitting
//     LMStrucData data;
//     data.points  = dataset->points;
//     data.samples = std::vector<int>(inliers.begin(), inliers.end());

//     // compute better coeffs for the given inliers and initial guess
//     RefitLevMarCone (&data, coeffs);
//     
//     // recompute inliers
//     OctreeIntersectionTest *test;
//     test = new OctreeIntersectionTestCone (dataset->points);

//     ANNpoint BoxOrigin = annAllocPt(3);
//     for (int d = 0; d < 3; d++)
//       BoxOrigin[d] = dataset->header.minPD[d];
//     test->init(coeffs, BoxOrigin, epsilon);
//     std::set<int> v = dataset->octree->IntersectWithShape (test, dataset->header.minPD, dataset->header.maxPD, (gui_mode >= 4) );

//     inliers = test->GetInliers (v, coeffs, epsilon, angle_thresh, idx_normal);

//     std::cerr << " --> " << inliers.size() << " inliers." << std::endl;
// //    std::cerr << "    after : " << inliers.size() << " inliers." << std::endl;
//   }

//   // make sure the inliers are all connected to the first sample point
//   ConnectedComponents ();
//   score = inliers.size();
}


bool Shape::CheckShape()
{
  #define CheckAndGetInliers(TYPE_CONSTANT,SHAPE_TYPE) \
    if ((unsigned int)type == ias_table_msgs::TableObject::TYPE_CONSTANT)\
    { \
      if (!SHAPE_TYPE::CheckShape (points, samples, idx_normal, angle_thresh, coeffs)) \
        return false; \
      inliers = SHAPE_TYPE::GetInliers (points, indices, coeffs, epsilon, idx_normal); \
      score = inliers.size (); \
    }
  
  double angle_thresh = angles::from_degrees (15);
  std::vector<int> indices;

  CheckAndGetInliers (PLANE,      ShapeTypePlane);
  CheckAndGetInliers (SPHERE,     ShapeTypePlane);
  CheckAndGetInliers (CYLINDER,   ShapeTypePlane);
  CheckAndGetInliers (ROTATIONAL, ShapeTypePlane);
  CheckAndGetInliers (BOX,        ShapeTypePlane);
  
  //OctreeListType v;
//   OctreeIntersectionTest *test;
//   if (type == SHAPE_TYPE_CYLINDER)
//     test = new OctreeIntersectionTestCylinder (dataset->points);
//   else if (type == SHAPE_TYPE_SPHERE)
//     test = new OctreeIntersectionTestSphere (dataset->points);
//   else if (type == SHAPE_TYPE_PLANE)
//     test = new OctreeIntersectionTestPlane (dataset->points);
//   else if (type == SHAPE_TYPE_CONE)
//     test = new OctreeIntersectionTestCone (dataset->points);
//   else if (type == SHAPE_TYPE_TORUS)
//     test = new OctreeIntersectionTestTorus (dataset->points);
//   else
//     std::cerr << "Invalid shape type: " << type << ". Shape.cc line 34" << std::endl;
//   
//   ANNpoint BoxOrigin = annAllocPt(3);
//   for (int d = 0; d < 3; d++)
//     BoxOrigin[d] = dataset->header.minPD[d];

//   test->init(coeffs, BoxOrigin, epsilon);
//   if (verbosity >= 2)
//     std::cerr << "     OctreeIntersectionTest initialised." << std::endl;
//   if (gui_mode >= 4)
//   {
//     if (verbosity >= 2)
//       std::cerr << "     Rendering Shape and Sample points." << std::endl;
//     if (verbosity >= 2 && type == SHAPE_TYPE_CYLINDER)
//       std::cerr << "     coeffs[2] < coeffs[5]" << (coeffs[2]<coeffs[5]) << std::endl;

//     RenderShape(ren);
//     RenderSamples(ren);
//     ren->RemoveAllViewProps();
//   }
//   
//   std::set<int> v = dataset->octree->IntersectWithShape (test, dataset->header.minPD, dataset->header.maxPD, (gui_mode >= 4) );
//   if (verbosity >= 2)
//     std::cerr << "     OctreeIntersectionTest done. possible nr_inliers = " << v.size() << std::endl;
// //  if (type == SHAPE_TYPE_SPHERE)
// //    cerr << "found otree cubes with " << v.size() << " possible inliers" << endl;

//   if (gui_mode >= 4 && dataset->octree->allBBs->GetNumberOfInputPorts() != 0)
//   {
//     if (verbosity >= 2)
//       std::cerr << "     Rendering Octree cubes containing possible inliers." << std::endl;
//     vtkSmartPointer<vtkActor> cubeActor = createActorFromDataSet (dataset->octree->allBBs->GetOutput (), 0.0, 1.0, 0.0, false);
//     cubeActor->GetProperty ()->SetRepresentationToWireframe ();
//     cubeActor->GetProperty ()->SetOpacity (0.1);
//     cubeActor->GetProperty ()->SetLineWidth (2);
//     ren->AddActor (cubeActor);
//     ren->ResetCamera ();
//   
//     ren->Modified ();
//     ren->Render ();
//     ren->GetRenderWindow ()->Render ();
//     ren->GetRenderWindow ()->GetInteractor()->Start ();
//     
//     ren->RemoveActor(cubeActor);
//   }

//   inliers = test->GetInliers (v, coeffs, epsilon, angle_thresh, idx_normal);
//   if (inliers.size() > min_nr_points_in_shape)
//     ConnectedComponents ();
// //  if (type == SHAPE_TYPE_SPHERE)
// //    cerr << "out of those, found " << inliers.size() << " real inliers" << endl;
//   if (verbosity >= 2)
//     std::cerr << "     Computed Inliers. nr_inliers = " << inliers.size() << std::endl;
//   score = inliers.size();
//   delete test;
//   annDeallocPt (BoxOrigin);
  return true;
}

// inline
// bool Shape::CheckShapeCylinder ()
// {
//     coeffs = annAllocPt(7);
//     ANNpoint u = annAllocPt (3);
//     ANNpoint v = annAllocPt (3);
//     ANNpoint w = annAllocPt (3);

//     for (int d = 0; d < 3; d++)
//     {
//       u[d] = dataset->points[samples[0]][idx_normal+d];
//       v[d] = dataset->points[samples[1]][idx_normal+d];
//       w[d] = (dataset->points[samples[0]][idx_normal+d] + dataset->points[samples[0]][d]) - dataset->points[samples[1]][d];
//     }

//     double a = _dot (u, u);
//     double b = _dot (u, v);
//     double c = _dot (v, v);
//     double d = _dot (u, w);
//     double e = _dot (v, w);
//     double denominator = a*c - b*b;
//     double sc, tc;
//     // Compute the line parameters of the two closest points
//     if (denominator < 1e-8)          // The lines are almost parallel
//     {
//       sc = 0.0;
//       tc = (b > c ? d / b : e / c);  // Use the largest denominator
//     }
//     else
//     {
//       sc = (b*e - c*d) / denominator;
//       tc = (a*e - b*d) / denominator;
//     }
//     // Get the closest points
//     coeffs[0] = dataset->points[samples[0]][0] + dataset->points[samples[0]][idx_normal + 0] + (sc * u[0]);
//     coeffs[1] = dataset->points[samples[0]][1] + dataset->points[samples[0]][idx_normal + 1] + (sc * u[1]);
//     coeffs[2] = dataset->points[samples[0]][2] + dataset->points[samples[0]][idx_normal + 2] + (sc * u[2]);
//     
//     coeffs[3] = dataset->points[samples[1]][0] + (tc * v[0]);
//     coeffs[4] = dataset->points[samples[1]][1] + (tc * v[1]);
//     coeffs[5] = dataset->points[samples[1]][2] + (tc * v[2]);

//     coeffs[6] = PointToLineDistance (dataset->points[samples[0]], coeffs);
//     annDeallocPt (u);
//     annDeallocPt (v);
//     annDeallocPt (w);

//     if (coeffs[6] > THRESH_RADIUS_CYLINDER)
//     {
//       annDeallocPt (coeffs);
//       return false;
//     }
//     for (int i = 0; i < 4 ; i++)
//     {
//       if (abs (PointToLineDistance (dataset->points[samples[i]], coeffs) - coeffs[6]) > epsilon)
//       {
//         annDeallocPt (coeffs);
//         return false;
//       }
//       double angle = (acos (
//           ((coeffs[0]-coeffs[3]) * dataset->points[samples[i]][idx_normal + 0]) +
//           ((coeffs[1]-coeffs[4]) * dataset->points[samples[i]][idx_normal + 1]) +
//           ((coeffs[2]-coeffs[5]) * dataset->points[samples[i]][idx_normal + 2])));
//       if ( fabs(fabs(angle) - DEG2RAD(90)) > angle_thresh)
//       {
//         annDeallocPt (coeffs);
//         return false;
//       }
//     }
//   
//   return true;
// }

// inline
//   bool Shape::CheckShapeSphere ()
// {
//     double m11, m12, m13, m14, m15;
//     // See http://local.wasp.uwa.edu.au/~pbourke/geometry/spherefrom4/ for more details
//     ANNpointArray temp = annAllocPts (4, 4);
//     //ANNpointArray temp = new ANNpoint[3];
//     // Find determinant M11
//     for (int i = 0; i < 4; i++)
//     {
//       temp[i][0] = dataset->points[samples[i]][0];
//       temp[i][1] = dataset->points[samples[i]][1];
//       temp[i][2] = dataset->points[samples[i]][2];
//       temp[i][3] = 1;
//     }
//     m11 = cANN::computeMatrix4x4Det (temp);

//     if (m11 == 0)
//     {
//       annDeallocPts (temp);
//       return false;
//     }
//     
//     // Find determinant M12
//     for (int i = 0; i < 4; i++)
//     {
//       temp[i][0] = SQR (dataset->points[samples[i]][0]) + SQR (dataset->points[samples[i]][1]) + SQR (dataset->points[samples[i]][2]);
//       //temp[i][1] = dataset->points[samples[i]][1];
//       //temp[i][2] = dataset->points[samples[i]][2];
//       //temp[i][3] = 1;
//     }
//     m12 = cANN::computeMatrix4x4Det (temp);
//     
//     // Find determinant M13
//     for (int i = 0; i < 4; i++)
//     {
//       temp[i][0] = dataset->points[samples[i]][0];
//       temp[i][1] = SQR (dataset->points[samples[i]][0]) + SQR (dataset->points[samples[i]][1]) + SQR (dataset->points[samples[i]][2]);
//       //temp[i][2] = dataset->points[samples[i]][2];
//       //temp[i][3] = 1;
//     }
//     m13 = cANN::computeMatrix4x4Det (temp);
//     
//     // Find determinant M14
//     for (int i = 0; i < 4; i++)
//     {
//       //temp[i][0] = dataset->points[samples[i]][0];
//       temp[i][1] = dataset->points[samples[i]][1];
//       temp[i][2] = SQR (dataset->points[samples[i]][0]) + SQR (dataset->points[samples[i]][1]) + SQR (dataset->points[samples[i]][2]);
//       //temp[i][3] = 1;
//     }
//     m14 = cANN::computeMatrix4x4Det (temp);
//     
//     // Find determinant M15
//     for (int i = 0; i < 4; i++)
//     {
//       temp[i][0] = SQR (dataset->points[samples[i]][0]) + SQR (dataset->points[samples[i]][1]) + SQR (dataset->points[samples[i]][2]);
//       temp[i][1] = dataset->points[samples[i]][0];
//       temp[i][2] = dataset->points[samples[i]][1];
//       temp[i][3] = dataset->points[samples[i]][2];
//     }
//     m15 = cANN::computeMatrix4x4Det (temp);
//     
//     // Center (x , y, z)
//     coeffs = annAllocPt (4);
//     coeffs[0] = 0.5 * m12 / m11;
//     coeffs[1] = 0.5 * m13 / m11;
//     coeffs[2] = 0.5 * m14 / m11;
//     // Radius
//     coeffs[3] = sqrt (SQR (coeffs[0]) + SQR (coeffs[1]) + SQR (coeffs[2]) - m15 / m11);

//     if (coeffs[3] > THRESH_RADIUS_SPHERE)
//     {
//       annDeallocPts (temp);
//       return false;
//     }
//     else
//       // if normals deviate, don't accept
//       for (int i = 0; i < 4 ; i++)
//       {
//         for (int d = 0; d < 3; d++)
//           temp[0][d] = dataset->points[samples[i]][d] - coeffs[d];
//         double angle = (acos (
//             (temp[0][0] * dataset->points[samples[i]][idx_normal + 0]) +
//             (temp[0][1] * dataset->points[samples[i]][idx_normal + 1]) +
//             (temp[0][2] * dataset->points[samples[i]][idx_normal + 2])));
//         if (angle > angle_thresh)
//         {
//           //annDeallocPts (temp);
//           //return false;
//         }
//       }

//     annDeallocPts (temp);
//   return true;
// }

inline
bool Shape::CheckShapePlane ()
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

// inline
// bool Shape::CheckShapeCone ()
// {
//   ANNpoint n1 = annAllocPt (3);
//   ANNpoint n2 = annAllocPt (3);
//   ANNpoint n3 = annAllocPt (3);
//   for (int d = 0; d < 3; d++)
//   {
//     n1[d] = dataset->points[samples[0]][idx_normal + d];
//     n2[d] = dataset->points[samples[1]][idx_normal + d];
//     n3[d] = dataset->points[samples[2]][idx_normal + d];
//   }
//   double d1 =   ((n1[0] * dataset->points[samples[0]][0]) +
//                  (n1[1] * dataset->points[samples[0]][1]) +
//                  (n1[2] * dataset->points[samples[0]][2]));
//   double d2 =   ((n2[0] * dataset->points[samples[1]][0]) +
//                  (n2[1] * dataset->points[samples[1]][1]) +
//                  (n2[2] * dataset->points[samples[1]][2]));
//   double d3 =   ((n3[0] * dataset->points[samples[2]][0]) +
//                  (n3[1] * dataset->points[samples[2]][1]) +
//                  (n3[2] * dataset->points[samples[2]][2]));
//     
//   // Planes N1*p = d1;
// //            |  d1 ( N2 * N3 ) + d2 ( N3 * N1 ) + d3 ( N1 * N2 )
// //   apex=   <     /
// //            |  N1 . ( N2 * N3 )
//           
//   ANNpoint n2xn3 = cross (n2, n3);
//   ANNpoint n3xn1 = cross (n3, n1);
//   ANNpoint n1xn2 = cross (n1, n2);
//   if (NORM(n2xn3) == 0 || NORM(n3xn1) == 0 || NORM(n1xn2) == 0)
//   {
//     annDeallocPt(n1);
//     annDeallocPt(n2);
//     annDeallocPt(n3);
//     annDeallocPt(n2xn3);
//     annDeallocPt(n3xn1);
//     annDeallocPt(n1xn2);
//     return false;
//   }
// //  LineCoefficientsNormalize(n2xn3);
// //  LineCoefficientsNormalize(n3xn1);
// //  LineCoefficientsNormalize(n1xn2);
//   double temp  = _dot (n1, n2xn3);
//   if (temp == 0)
//   {
//     annDeallocPt(n1);
//     annDeallocPt(n2);
//     annDeallocPt(n3);
//     annDeallocPt(n2xn3);
//     annDeallocPt(n3xn1);
//     annDeallocPt(n1xn2);
//     return false;
//   }
//   
//   ANNpoint apex = annAllocPt(3);
//   for (int d = 0; d < 3; d++)
//     apex[d] = (d1*n2xn3[d] + d2*n3xn1[d] + d3*n1xn2[d]) / temp;
//   
//   ANNpoint p1_apex = annAllocPt (3);
//   ANNpoint p2_apex = annAllocPt (3);
//   ANNpoint p3_apex = annAllocPt (3);
//   for (int d = 0; d < 3; d++)
//   {
//     p1_apex[d] = dataset->points[samples[0]][d] - apex[d];
//     p2_apex[d] = dataset->points[samples[1]][d] - apex[d];
//     p3_apex[d] = dataset->points[samples[2]][d] - apex[d];
//   }
//   double ax1_norm = 0.0;
//   double ax2_norm = 0.0;
//   ANNpoint ax1 = annAllocPt(3);
//   ANNpoint ax2 = annAllocPt(3);
//   for (int d = 0; d < 3; d++)
//   { // n1, n2, n3 are considered points now :) 
//     n1[d] = apex[d] + p1_apex[d]/sqrt(NORM(p1_apex));
//     n2[d] = apex[d] + p2_apex[d]/sqrt(NORM(p2_apex));
//     n3[d] = apex[d] + p3_apex[d]/sqrt(NORM(p3_apex));
//     ax1[d] = n2[d] - n1[d];
//     ax1_norm += ax1[d] * ax1[d];
//     ax2[d] = n3[d] - n1[d];
//     ax2_norm += ax2[d] * ax2[d];
//   }
//   
//   if (ax1_norm == 0 || ax2_norm == 0)
//   {
//     annDeallocPt(n1);
//     annDeallocPt(n2);
//     annDeallocPt(n3);
//     annDeallocPt(n2xn3);
//     annDeallocPt(n3xn1);
//     annDeallocPt(n1xn2);

//     annDeallocPt(apex);
//     annDeallocPt(p1_apex);
//     annDeallocPt(p2_apex);
//     annDeallocPt(p3_apex);
//     annDeallocPt(ax1);
//     annDeallocPt(ax2);

//     return false;
//   }
//   
//   for (int d = 0; d < 3; d++) 
//   {
//     ax1[d] /= sqrt(ax1_norm);
//     ax2[d] /= sqrt(ax2_norm);
//   }  
//   ANNpoint axis = cross(ax1, ax2);
//   if (NORM(axis) == 0)
//   {
//     annDeallocPt(n1);
//     annDeallocPt(n2);
//     annDeallocPt(n3);
//     annDeallocPt(n2xn3);
//     annDeallocPt(n3xn1);
//     annDeallocPt(n1xn2);

//     annDeallocPt(apex);
//     annDeallocPt(p1_apex);
//     annDeallocPt(p2_apex);
//     annDeallocPt(p3_apex);
//     annDeallocPt(ax1);
//     annDeallocPt(ax2);
//     annDeallocPt(axis);
//     return false;
//   }
//   LineCoefficientsNormalize(axis);

//   double d = -(  (axis[0]*n1[0]) + 
//                  (axis[1]*n1[1]) + 
//                  (axis[2]*n1[2]));
//   if ((_dot (axis, apex) + d) > 0)
//     for (int d = 0; d < 3; d++)
//       axis[d] *= -1;
//   
//   double opening_angle = (acos(_dot(p1_apex, axis) / sqrt(NORM(p1_apex)))
//                         + acos(_dot(p2_apex, axis) / sqrt(NORM(p2_apex)))
//                         + acos(_dot(p3_apex, axis) / sqrt(NORM(p3_apex)))) / 3.0;
//  
//   coeffs = annAllocPt(7);
//   coeffs[0] = apex[0];
//   coeffs[1] = apex[1];
//   coeffs[2] = apex[2];
//   coeffs[3] = axis[0];
//   coeffs[4] = axis[1];
//   coeffs[5] = axis[2];
//   coeffs[6] = opening_angle;

// //  std::cerr << "    - Apex  = [ " << coeffs[0] << ", " << coeffs[1] << ", " << coeffs[2] << " ]" << std::endl;
// //  std::cerr << "    - Axis  = [ " << coeffs[3] << ", " << coeffs[4] << ", " << coeffs[5] << " ]" << std::endl;  
// //  std::cerr << "    - Angle = [ " << RAD2DEG(coeffs[6]) << " ]" << std::endl;  

//   annDeallocPt(n1);
//   annDeallocPt(n2);
//   annDeallocPt(n3); 
//   annDeallocPt(n2xn3);
//   annDeallocPt(n3xn1);
//   annDeallocPt(n1xn2);

//   annDeallocPt(apex);
//   annDeallocPt(p1_apex);
//   annDeallocPt(p2_apex);
//   annDeallocPt(p3_apex);
//   annDeallocPt(ax1);
//   annDeallocPt(ax2);
//   annDeallocPt(axis);
//   if (opening_angle > DEG2RAD (64) || opening_angle < DEG2RAD(5))
//   {
//     return false;
//   }
//   return true;
// }

// inline
// bool Shape::CheckShapeTorus ()
// {
//   return false;
// }

unsigned int Shape::GetScore ()
{
  assert (score == inliers.size());
  return score;
}

int Shape::GetType ()
{
  return type;
}

std::vector<double> Shape::GetCoeffs ()
{
  return coeffs;
}

int Shape::RemovePoints (std::set<int> p)
{
  std::set<int> diff;
  set_difference(inliers.begin(), inliers.end(), p.begin(), p.end(), 
          std::insert_iterator<std::set<int> > (diff, diff.begin()));
  score = diff.size();
  inliers = diff;
  return score;
}

std::set<int> Shape::GetInliers()
{
  return inliers;
}

bool operator< (Shape &s1, Shape &s2)  
{  
  return (s1.score < s2.score);  
}

bool operator> (Shape &s1, Shape &s2)  
{  
  return (s1.score > s2.score);  
}

void Shape::PrintCoeffs()
{
  int n = 0;
  switch (type)
  {
    case ias_table_msgs::TableObject::SPHERE:
    case ias_table_msgs::TableObject::PLANE:
      n = 4;
      break;
    case ias_table_msgs::TableObject::CYLINDER:
      n = 7;
      break;
    default:
      std::cerr << "not implemented: type = " << std::endl;
      break;
  }
  std::cerr << "  type = " << type;
  for (int i = 0; i < n; i++)
    std::cerr << "  " << coeffs[i];
  std::cerr << std::endl;
}
  
std::set<int> Shape::GetPlaneInliers (std::vector<double> shape_coeffs, 
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

