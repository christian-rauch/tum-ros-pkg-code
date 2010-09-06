/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

/* author: Radu Bogdan Rusu <rusu@cs.tum.edu> */

// ROS core
#include "ros/node_handle.h"
#include "ros/time.h"
#include "ros/common.h"

#include <cfloat>

#define BOOL2STR_TF(b) ( b ? "true" : "false" )

using namespace std;

class Snapshot: public ros::node
{
  public:
    Snapshot () : ros::node ("composite_snapshot") { }
    ~Snapshot () { }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);

  // Here is where we create our ROS node
  Snapshot p;
  
  std::string topic ("/composite/composite_snapshot");
  
  if (p.has_param (topic))
  {
    int param_value;
    if (p.get_param (topic, param_value))
    {
      param_value *= -1;
        
      fprintf (stderr, "Parameter found, changing value to %d.\n", param_value);
      p.set_param (topic, param_value);
    }
    else
      fprintf (stderr, "Cannot get parameter %s value!\n", topic.c_str ());
  }
  else
    p.set_param (topic, 1);
  
  // Shutdown
  p.shutdown ();

  ros::fini ();

  return (0);
}
/* ]--- */
