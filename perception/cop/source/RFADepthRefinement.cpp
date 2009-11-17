/*
 * Copyright (C) 2009 by Ulrich Friedrich Klank <klank@in.tum.de>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

 
#include "RFADepthRefinement.h"
#include "ShapeModel.h"

#ifndef USE_YARP_COMM
#include <ros/ros.h>
#include <robot_msgs/PointCloud.h>
#endif


RFADepthRefinement::RFADepthRefinement()
{
}


RFADepthRefinement::RFADepthRefinement(XMLTag* tag)
{
}

RFADepthRefinement::~RFADepthRefinement(void)
{
}

bool filled = false;
#ifndef USE_YARP_COMM
std::vector<robot_msgs::Point32> _pts;

void PointCloudCallback(const boost::shared_ptr<const robot_msgs::PointCloud> pc)
{
   _pts = pc->pts;
   filled = true;
}

double dist(Polygon_t poly, robot_msgs::Point32 pt, double thres = 0.3)
{
  double dist = 1000000.0;
  double dist_x_y = thres + 0.1;
  for(int n = 0; n < poly.size(); n++)
  {
    double dist_xy_tmp = + (poly[n].x - pt.x)*(poly[n].x - pt.x) + (poly[n].y - pt.y)*(poly[n].y - pt.y);
    if(dist_xy_tmp < dist_x_y)
      dist_x_y == dist_xy_tmp;
    double dist_tmp = dist_xy_tmp + (poly[n].z - pt.z)*(poly[n].z - pt.z);
    if(dist_tmp < dist)
      dist = dist_tmp;
  }
  if(dist_x_y >= thres) /*Out of region of interest*/
    dist = 0.0;
  return dist;
}

RelPose* DepthRefinement(RelPose* pose, ShapeModel* model)
{
  RelPose* cur = pose;
  double dist_min = 100000000000.0;
  double cur_scale = 1.0;
  for(double scale = 0.8; scale < 1.2; scale += 0.05)  /*TODO: Gradient descent*/
  {
    double dist_sum = 0;
    for(std::vector<robot_msgs::Point32>::const_iterator pt = _pts.begin(); pt != _pts.end(); pt++)
    {
      Mesh_t mesh = model->GetMesh(pose, scale);
      for(Mesh_t::const_iterator poly = mesh.begin(); poly != mesh.end(); poly++)
      {
         dist_sum += dist((*poly).first, (*pt));
      }
    }
    if(dist_sum < dist_min)
    {
      dist_min = dist_sum;
      cur_scale = scale;
    }
  }
  printf("scale: %f\n", cur_scale);
  /*Scale!*/

  return cur;
}

#endif

Descriptor* RFADepthRefinement::Perform(std::vector<Camera*> cam, RelPose* pose, Signature& sig, int &numOfObjects, double& qualityMeasure)
{
  Descriptor *ret = NULL;
  Image *img = NULL;
  if(numOfObjects == 1)
  {
    Camera* cam3d = NULL;
    for(unsigned int i = 0; i < (unsigned int)cam.size(); i++)
    {
        if(cam[i]->CanSee(*pose) &&  cam[i]->GetName().compare("CameraDriver") == 0)
        {
          img = cam[i]->GetImage(-1);
          if(img->GetType() == GRAY_DISPARITY_IMAGE || img->GetType() == YUV_DISPARITY_IMAGE)
          {
            cam3d = cam[i];
            break;
          }
          img->Free();
        }
    }
    if(cam3d != NULL && img != NULL)
    {
        /**Intersect the Pose Induced line of sight with the 3D structures by selecting the visible faces*/
        ShapeModel* model = (ShapeModel*)sig.GetElement(0, DESCRIPTOR_SHAPE);
        if(model != NULL)
        {
          ret = NULL;
        }


#ifndef USE_YARP_COMM
        filled = false;
        ros::NodeHandle n;
        if(filled = false)
        {
          ros::Subscriber s = n.subscribe<robot_msgs::PointCloud>("swissranger_test_node", 1, &PointCloudCallback);
          ros::Rate r(1);
          while (!filled)
          {
            r.sleep();
            continue;
          }
          RelPose* pose_out = DepthRefinement(pose, model);
        }

#endif


    }
  }
  return ret;
}

double RFADepthRefinement::CheckSignature(Signature& sig)
{
  double ret = 0.0;
  ShapeModel* model = (ShapeModel*)sig.GetElement(0, DESCRIPTOR_SHAPE);
  if(model != NULL)
  {
    ret = 1.0;
  }
  return ret;
}

XMLTag* RFADepthRefinement::Save()
{
  return new XMLTag(this->GetName());
}

