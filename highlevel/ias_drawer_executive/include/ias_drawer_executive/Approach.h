/* 
 * Copyright (c) 2010, Thomas Ruehr <ruehr@cs.tum.edu>
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


#ifndef __APPROACH_H__
#define __APPROACH_H__



#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>


class Approach {

    public :

    enum SensorArea {inside, front};

    std_srvs::Empty serv;
    RobotArm *arm;
    tf::Stamped<tf::Pose> plateCenter,startPose;
    btVector3 diff;
    Pressure *pressure;
    float pressureZeroR, pressureZeroL;
    float pressureDiff;
    bool touched;
    float firstTouch;
    int side_;
    int sensors_;

    // sensors 0 = inside, 1 = front
    void init(int side, tf::Stamped<tf::Pose> approachP, tf::Stamped<tf::Pose> plateCenter, SensorArea sensor = Approach::inside);
    void move_to(float ap);
    float increment(float ap);
    bool finish();

};


class Lift {

    public :

    std_srvs::Empty serv;
    RobotArm *arm;
    tf::Stamped<tf::Pose> plateCenter,startPose;
    btVector3 diff;
    btVector3 diffToRob;

    int side_;

    void init(int side);

    //void increment(float up, float back);
    void increment(float x, float y, float z);

};

#endif
