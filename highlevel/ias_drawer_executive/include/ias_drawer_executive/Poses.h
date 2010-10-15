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
static float milehighPoseA[7];
static float milehighPoseB[7];
//{-0.82681730923312724, 0.15596892532371576, -0.16835406527856578, -1.7773528734848965, -197.19522687315879, -1.0987956365304499, 284.37391632306253};
// arm joint angles for seeing the marker with the hand camera
//high drawer
static float highPoseA[7];
static float highPoseB[7];

//low drawer
static float lowPoseA[7];
static float lowPoseB[7];

//middle drawer
static float midPoseA[7];
static float midPoseB[7];

//tucking pose, expects left arm to be tucked already
static float tuckPose[7];
static float tuckPoseForLeft[7];
//untucking poses, hand follows the base in a circular motion
static float untuckPoseA[7];
static float untuckPoseB[7];

static float leftHighB[7];
static float leftHighA[7];

static float lf0[7];
static float lf1[7];
static float lf2[7];
static float lf3[7];

static float dishA[7];
static float dishB[7];
static float dishC[7];
static float dishD[7];

// robot poses in map frame
static float poseA[4]; // island left
static float poseB[4]; //island middle
static float poseC[4]; //island right
//float poseD[4] = { .212, 2.89, 0, 1}; // below oven
static float poseD[4]; // below oven
static float poseE[4]; //sink left
static float poseF[4]; // sink dishwasher
static float poseG[4]; // sink righ / trash
static float poseH[4]; // right of oven
static float poseI[4]; // fridge
static float poseJ[4]; // fridge

//float poseD1[4] = { -0.996, 2.152, 0.921, -.390}; // %8
//float poseD2[4] = { -.754, 0.239, 0.952, .306}; // %9
//float poseD3[4] = { .242, 2.263, 0, 1}; // right of heater
//float poseD4[4] = { .242, 2.263, 0, 1}; // right of heater

static float pose0[4]; // somewhere in between island and sink

static float *poses[20];

static int drw_top ;
static int drw_mid ;
static int drw_low ;

static float inDrawer0[];

static float prepDishL0[7];
static float prepDishR0[7];
static float prepDishL1[7];
static float prepDishR1[7];

static float prepDishRT[7];
static float prepDishLT[7];


private:
   Poses(){}
};

/*




*/

#endif
