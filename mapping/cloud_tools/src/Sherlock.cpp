#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/areas.h>
#include <point_cloud_mapping/geometry/statistics.h>
#include <point_cloud_mapping/geometry/nearest.h>
#include <point_cloud_mapping/geometry/angles.h>
#include <ias_table_msgs/TableObject.h>
#include "Sherlock.h"

Sherlock::Sherlock()
{
  nr_trials = 0;
}

void Sherlock::SetData (const sensor_msgs::PointCloudConstPtr pts)
{
  this->points = pts; 
  nr_points_left = pts->points.size ();
  points_used.clear ();
}

void Sherlock::SetShapeTypes (unsigned int types)
{
  if (types & ias_table_msgs::TableObject::PLANE)
    shapetypes.push_back (ias_table_msgs::TableObject::PLANE);
  if (types & ias_table_msgs::TableObject::SPHERE)
    shapetypes.push_back (ias_table_msgs::TableObject::SPHERE);
  if (types & ias_table_msgs::TableObject::CYLINDER)
    shapetypes.push_back (ias_table_msgs::TableObject::CYLINDER);
  if (types & ias_table_msgs::TableObject::ROTATIONAL)
    shapetypes.push_back (ias_table_msgs::TableObject::ROTATIONAL);
  if (types & ias_table_msgs::TableObject::BOX)
    shapetypes.push_back (ias_table_msgs::TableObject::BOX);
}

Sherlock::~Sherlock()
{
  for (std::vector<Shape*>::iterator it = suspects.begin(); it != suspects.end(); it++)
    delete (*it);
  for (std::vector<Shape*>::iterator it = shapes.begin(); it != shapes.end(); it++)
    delete (*it);
}

void Sherlock::AddSuspect(Shape* shape)
{
  if (shape->score < min_nr_points_in_shape)
  {
    delete shape;
    return;
  }
  suspects.push_back(shape);
  if (suspects.size() == 1 || (*shape) > (*best))
  {
    best = shape;
    best_index = suspects.size()-1;
  }
}

void Sherlock::FindSuspects()
{
  std::vector<int> sample(4);
  int idx_normal = cloud_geometry::getChannelIndex (points, "nx");
  // sample first point randomly
  sample[0] = (int) (( rand() / (RAND_MAX + 1.0) ) * (points->points.size()));

  // sample 3 more points, making sure that they are different
  for(int i = 1; i < 4; i++)
  {
    bool same;
    do
    {
      sample[i] = (int) ( (rand() / (RAND_MAX + 1.0)) * (points->points.size()));
      same = false;
      if (points_used.find(sample[i]) != points_used.end())
      {
        same = true;
        continue;
      }

      for(int j = 0; j < i; j++)
        if (sample[j] == sample[i])
          same = true;
    } while (same);
  }

  // If the samples can be explained by one of the shape models, it's a Suspect
  for (unsigned int shape_type = 0; shape_type < shapetypes.size (); shape_type++)
  {
    nr_trials ++;
    Shape *S = new Shape(points, sample, shapetypes[shape_type], idx_normal);
    if (S->CheckShape())
      AddSuspect(S);
    else
      delete S;
  }
}

Shape *Sherlock::GetPrimeSuspect()
{
  assert (suspects.size() != 0);
  return best;
}

void Sherlock::EliminatePrimeSuspect()
{
  best->RefitToInliers ();
  
  std::set<int> inliers = best->GetInliers();
  
  int best_value = 0;
  Shape* best_temp;
  int bla =0 ;
  // Pretend as if inliers never existed...
  int counter = 0;
  for (std::vector<Shape*>::iterator it = suspects.begin(); it != suspects.end(); it++)
  {
    if ((*it) == best)
      continue;
    bla = (*it)->RemovePoints (inliers);

    counter++;
    // Keep track of who's best
    if (bla > best_value)
    {
      best_value = bla;
      best_temp = (*it);
    }
    // Remove Suspect if nr_inliers gets too low
    if (bla < 4)
    {
      delete (*it);
      it = suspects.erase(it);
      it--;
//      if (it == suspects.end())
//        break;
    }
  }
  
  if (((signed int)nr_points_left) - ((signed int)inliers.size()) < 0)
  {
    std::cerr << "diff: " << (((signed int)nr_points_left) - ((signed int)inliers.size())) << std::endl;
    nr_points_left = 0;
  }
  else
    nr_points_left -= inliers.size();
  points_used.insert (inliers.begin (), inliers.end ()); 
  std::cerr << "pts left: " 
            << nr_points_left
            << ", pts used: "
            << points_used.size() 
            << std::endl;
  suspects.erase(std::find(suspects.begin(), suspects.end(), best));
  best = best_temp;
}

double Sherlock::pFindNoBetter (int n)
{
  if (nr_points_left == 0)
    return 1.0;
  double P_n = pow (( (double) n * pow (8.0,3.0) / (double) (nr_points_left) ) , 3.0 );
  double retval = 1.0 - pow ( ( 1.0 - P_n ), (double)nr_trials); //suspects_->NumberOfSuspects ());
  return retval; 
}

bool Sherlock::pFindNoBetterFalse (int n)
{
  long long int bla1 = nr_trials;
  double P_n = ((double)n/((double)(nr_points_left))) ;
//  if (!use_class_info)
  P_n /= 2*2*2;
  long long int bla2 = -log(1.0-prob_thresh) / P_n;
  // -(n/N) should be -(n/N) - (n/N)^2/2 - (n/N)^3/3 etc...

  return bla1 < bla2;
}

bool Sherlock::pFindNoBetterTrue (int n)
{
  long long int bla1 = nr_trials;
  double P_n = ((double)n/((double)(nr_points_left))) ;
//  if (!use_class_info)
  P_n /= 2*2*2;
  long long int bla2 = -log(1.0-prob_thresh) / P_n;
//  long long int bla2 = -log(1.0-prob_thresh)/pow(((double)n/(double)nr_points_left),2.0);
  
  return bla1 > bla2;
}

int Sherlock::DetectShapes()
{
  Shape *m;
  do
  {
    FindSuspects();
    
    if (suspects.size() < 1)
      continue;
    m = GetPrimeSuspect();
    
    if (pFindNoBetterTrue (m->GetScore()))
    {
      EliminatePrimeSuspect();
      
      std::cerr << "\tFound shape of type " << m->GetType() 
                << " with " << m->GetInliers().size() << " inliers." << std::endl;
      shapes.push_back(m);
    }
  } while (pFindNoBetterFalse (min_nr_points_in_shape)
       &&  (nr_points_left >= (signed int)min_nr_points_in_shape));
//   if (suspects.size()!=0)
//   {
//     std::cerr << "\nExtracting remaining shapes..." << std::endl;
//     do
//     {
//       m = GetPrimeSuspect();
//       if (m->GetScore() >= min_nr_points_in_shape)
//       {
//         EliminatePrimeSuspect();
//         shapes.push_back(m);
//       }
//     }
//     while (m->GetScore() >= min_nr_points_in_shape);
//   }
  return shapes.size ();
}


