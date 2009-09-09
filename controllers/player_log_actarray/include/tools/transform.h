/*
 * Copyright (c) 2008 Dejan Pangercic <pangercic -=- cs.tum.edu>
 *
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
 *
 * $Id: transform.h 21045 2009-08-07 20:52:44Z dejanpan $
 *
 */

/** \author Dejan Pangercic */

#ifndef _PLAYER_LOG_ACTARRAY_TRANSFORM_H_
#define _PLAYER_LOG_ACTARRAY_TRANSFORM_H_

// ROS includes
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <kdl/frames.hpp>

using namespace KDL;
using namespace sensor_msgs;

namespace player_log_actarray
{
  
  namespace transform
  {
    bool rotatePointCloud (const PointCloud &cloud_in, PointCloud &cloud_out, double angle)
    {
      Vector p,p2; 
      Rotation R;
    for (unsigned int point; point < cloud_in.points.size(); point++)
      {
	p.x(cloud_in.points[point].x);
	p.y(cloud_in.points[point].y);
	p.z(cloud_in.points[point].z);
	R.DoRotX(angle);
	p2 = R*p;
	cloud_out.points[point].x = p2.x();
      }
    }
  }
}
#endif
