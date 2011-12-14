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


#ifndef __POSES_H__
#define __POSES_H__

class Poses {
public:
static double milehighPoseA[7];
static double milehighPoseB[7];
//{-0.82681730923312724, 0.15596892532371576, -0.16835406527856578, -1.7773528734848965, -197.19522687315879, -1.0987956365304499, 284.37391632306253};
// arm joint angles for seeing the marker with the hand camera
//high drawer
static double highPoseA[7];
static double highPoseB[7];

//low drawer
static double lowPoseA[7];
static double lowPoseB[7];

//middle drawer
static double midPoseA[7];
static double midPoseB[7];

//tucking pose, expects left arm to be tucked already
static double tuckPose[7];
static double tuckPoseForLeft[7];
//untucking poses, hand follows the base in a circular motion
static double untuckPoseA[7];
static double untuckPoseB[7];

static double leftHighB[7];
static double leftHighA[7];

static double lf0[7];
static double lf1[7];
static double lf2[7];
static double lf3[7];

static double dishA[7];
static double dishB[7];
static double dishC[7];
static double dishD[7];

// robot poses in map frame
static double poseA[4]; // island left
static double poseB[4]; //island middle
static double poseC[4]; //island right
//double poseD[4] = { .212, 2.89, 0, 1}; // below oven
static double poseD[4]; // below oven
static double poseE[4]; //sink left
static double poseF[4]; // sink dishwasher
static double poseG[4]; // sink righ / trash
static double poseH[4]; // right of oven
static double poseI[4]; // fridge
static double poseJ[4]; // fridge

//double poseD1[4] = { -0.996, 2.152, 0.921, -.390}; // %8
//double poseD2[4] = { -.754, 0.239, 0.952, .306}; // %9
//double poseD3[4] = { .242, 2.263, 0, 1}; // right of heater
//double poseD4[4] = { .242, 2.263, 0, 1}; // right of heater

static double pose0[4]; // somewhere in between island and sink

static double *poses[20];

static int drw_top ;
static int drw_mid ;
static int drw_low ;

static double inDrawer0[];

static double prepDishL0[7];
static double prepDishR0[7];
static double prepDishL1[7];
static double prepDishR1[7];

static double prepDishRT[7];
static double prepDishLT[7];


private:
   Poses(){}
};

/*




*/

#endif
