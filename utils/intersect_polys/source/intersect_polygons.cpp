/* 
 * Copyright (c) 2009, U.
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





#include "cpp/HalconCpp.h"
#include "ros/ros.h"
#include "vision_srvs/clip_polygon.h"


// Procedure declarations 
void intersect_poly (Halcon::HTuple X1, Halcon::HTuple Y1, Halcon::HTuple X2, Halcon::HTuple Y2, 
    Halcon::HTuple *XOut, Halcon::HTuple *YOut);
void merge_polys (Halcon::HTuple X1, Halcon::HTuple Y1, Halcon::HTuple X2, Halcon::HTuple Y2, 
    Halcon::HTuple *XOut, Halcon::HTuple *YOut);
// Procedures 
void intersect_poly (Halcon::HTuple X1, Halcon::HTuple Y1, Halcon::HTuple X2, Halcon::HTuple Y2, 
    Halcon::HTuple *XOut, Halcon::HTuple *YOut)
{
  using namespace Halcon;

  // Local iconic variables 
  Hobject  Contour, Contour2, ContoursIntersection;

  gen_contour_polygon_xld(&Contour, X1, Y1);
  gen_contour_polygon_xld(&Contour2, X2, Y2);
  intersection_closed_contours_xld(Contour, Contour2, &ContoursIntersection);
  get_contour_xld(ContoursIntersection, &(*XOut), &(*YOut));
  return;
}

void merge_polys (Halcon::HTuple X1, Halcon::HTuple Y1, Halcon::HTuple X2, Halcon::HTuple Y2, 
    Halcon::HTuple *XOut, Halcon::HTuple *YOut)
{
  using namespace Halcon;

  // Local iconic variables 
  Hobject  Contour, Contour2, ContoursIntersection;

  gen_contour_polygon_xld(&Contour, X1, Y1);
  gen_contour_polygon_xld(&Contour2, X2, Y2);
  union2_closed_contours_xld(Contour, Contour2, &ContoursIntersection);
  get_contour_xld(ContoursIntersection, &(*XOut), &(*YOut));
  return;
}

bool srvcallback(vision_srvs::clip_polygon::Request& req, vision_srvs::clip_polygon::Response& resp)
{
   Halcon::HTuple x1,x2,y1, y2, xout, yout;
   for(int i = 0; i < req.poly1.points.size(); i++)
   {
     x1 = x1.Append(req.poly1.points[i].x);
     y1 = y1.Append(req.poly1.points[i].y);
   }
   for(int j = 0; j < req.poly2.points.size(); j++)
   {
     x2 = x2.Append(req.poly2.points[j].x);
     y2 = y2.Append(req.poly2.points[j].y);
   }

   if (req.operation == req.UNION)
     merge_polys (x1, y1, x2, y2, &xout, &yout);
   else if (req.operation == req.INTERSECTION)
     intersect_poly (x1,y1, x2, y2, &xout, &yout); 

   for(int j = 0; j < yout.Num(); j++)
   {
     geometry_msgs::Point32 p;
     p.x = xout[j].D();
     p.y = yout[j].D();
     resp.poly_clip.points.push_back(p);
   }
   return true;
}

int main(int argc, char* argv [])
{	
    ros::init( argc, argv, "intersect_polygons");
    ros::NodeHandle nh;
    ros::ServiceServer srv = nh.advertiseService("/intersect_poly", srvcallback);
    
    while(true)
    {
       ros::spin();
       sleep(0.1);
    }
}
