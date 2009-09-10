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
using namespace geometry_msgs;

namespace player_log_actarray
{
  
  namespace transform
  {
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // rotate PointCloud
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool rotatePointCloud (const PointCloud &cloud_in, PointCloud &cloud_out, double angle, Point32 axis)
    { 
      Vector p,p2; 
      Rotation R;
        if(axis.x)
	    R.DoRotX(angle);
	  else if (axis.y)
	    R.DoRotY(angle);
	  else if  (axis.z)
	    R.DoRotZ(angle);
	  else
	    {
	      ROS_ERROR("Axis must [1 0 0] format!");
	      return false;
	    }
	cloud_out.points.resize(0);
	cloud_out.header.frame_id =  cloud_in.header.frame_id;
	cloud_out.channels.resize (cloud_in.channels.size());
      for (int i = 0; i < cloud_in.channels.size(); i++)
	cloud_out.channels[i].name =  cloud_in.channels[i].name;

      ROS_INFO("in rpc %f %f %f %f", angle, axis.x, axis.y, axis.z);
      for (unsigned int point = 0; point < cloud_in.points.size(); point++)
	{
	  //resize
	  for (unsigned int d = 0; d < cloud_in.channels.size (); d++)
	    cloud_out.channels[d].values.resize (point + 1);
	  //rotate, fill in
	  p.x(cloud_in.points[point].x);
	  p.y(cloud_in.points[point].y);
	  p.z(cloud_in.points[point].z);
	  p2 = R*p;
	  cloud_out.points.resize(point + 1);
	  cloud_out.points[point].x = p2.x();
	  cloud_out.points[point].y = p2.y();
	  cloud_out.points[point].z = p2.z();

	  // Save the rest of the values
	  for (int i = 0; i < cloud_in.channels.size(); i++)
	    {
	      cloud_out.channels[i].values[point] =  cloud_in.channels[i].values[point];
	    }
	}
      return true;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // shift PointCloud
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool translatePointCloud (const PointCloud &cloud_in, PointCloud &cloud_out, Point32 trans)
    {
      cloud_out.points.resize(0);
      cloud_out.header.frame_id =  cloud_in.header.frame_id;
      cloud_out.channels.resize (cloud_in.channels.size());
      for (int i = 0; i < cloud_in.channels.size(); i++)
	cloud_out.channels[i].name =  cloud_in.channels[i].name;

       for (unsigned int point = 0; point < cloud_in.points.size(); point++)
	{
	  //resize
	  for (unsigned int d = 0; d < cloud_in.channels.size (); d++)
	    cloud_out.channels[d].values.resize (point + 1);
	  //rotate, fill in
	  cloud_out.points.resize(point + 1);
	  cloud_out.points[point].x = cloud_in.points[point].x + trans.x;
	  cloud_out.points[point].y = cloud_in.points[point].y + trans.y;
	  cloud_out.points[point].z = cloud_in.points[point].z + trans.z;
	  // Save the rest of the values
	  for (int i = 0; i < cloud_in.channels.size(); i++)
	    cloud_out.channels[i].values[point] =  cloud_in.channels[i].values[point];
	}
       return true;
    } 
  }
}
#endif
