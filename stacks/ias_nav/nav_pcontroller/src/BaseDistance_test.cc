/*
 * Copyright (c) 2009, Ingo Kresse <kresse@in.tum.de>
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


#include <vector>
#include <algorithm> //random_shuffle
#include <math.h>
#include <stdio.h>

#include <assert.h>

#include "BaseDistance.h"

#define Vector2 BaseDistance::Vector2

bool distance_test()
{
  std::vector<Vector2> points;
  Vector2 n;
  double d, eps=0.00001;

  BaseDistance tester;
  tester.setFootprint(0.8, -0.3, 0.5, -0.7, 0.1);

  // edge cases
  points.push_back(Vector2(3.2f, 0.0f));
  d = tester.distance(points, &n);
  assert((d-2.4)<eps && n.x == 3.2f && n.y == 0.0f);

  points.push_back(Vector2(0.0f, 2.8f));
  d = tester.distance(points, &n);
  assert((d-2.3 < eps) && n.x == 0.0f && n.y == 2.8f);

  points.push_back(Vector2(0.0f, -2.9f));
  d = tester.distance(points, &n);
  assert((d-2.2 < eps) && n.x == 0.0f && n.y == -2.9f);

  points.push_back(Vector2(-2.4f, 0.0f));
  d = tester.distance(points, &n);
  assert((d-2.1 < eps) && n.x == -2.4f && n.y == 0.0f);


  // corner cases
  points.push_back(Vector2(1.76f, 1.6f));
  d = tester.distance(points, &n);
  assert((d-1.46 < eps) && n.x == 1.76f && n.y == 1.6f);

  points.push_back(Vector2(-1.18f, 1.55f));
  d = tester.distance(points, &n);
  assert((d-1.37 < eps) && n.x == -1.18f && n.y == 1.55f);

  points.push_back(Vector2(-1.05f, -1.7f));
  d = tester.distance(points, &n);
  assert((d-1.25 < eps) && n.x == -1.05f && n.y == -1.7f);

  points.push_back(Vector2(0.95f, -1.82f));
  d = tester.distance(points, &n);
  assert((d-1.13 < eps) && n.x == 0.95f && n.y == -1.82f);


  // check if the order matters - it should not.
  for(int i=0; i < 1000; i++) {
    std::random_shuffle(points.begin(), points.end());
    d = tester.distance(points, &n);
    assert((d-1.13 < eps) && n.x == 0.95f && n.y == -1.82f);
  }
  return true;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "BaseDistance_test");
  distance_test();
}
