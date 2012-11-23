#include "FRICheck.hh"

#include <float.h>
#include <string.h>
#include <math.h>

#define DEG *M_PI/180.0
#define RAD /M_PI*180.0

// limits as of KRC/roboter/dlrrc/cfg/rsidef7.dat
const float FRICheck::margin_ = 0.5 DEG;
const float FRICheck::lim_low_[7] = {-170 DEG, -120 DEG, -170 DEG, -120 DEG, -170 DEG, -120 DEG, -170 DEG};
const float FRICheck::lim_high_[7] = { 170 DEG,  120 DEG,  170 DEG,  120 DEG,  170 DEG,  120 DEG,  170 DEG};
const float FRICheck::lim_vel_[7] = {120 DEG, 120 DEG, 160 DEG, 160 DEG, 250 DEG, 220 DEG, 220 DEG};
const float FRICheck::lim_acc_[7] = {1200 DEG, 1200 DEG, 1600 DEG, 1600 DEG, 2500 DEG, 4400 DEG, 4400 DEG};


FRICheck::FRICheck() :
  j5_angles_(0), j6_min_(0), j6_max_(0), j56_length_(0)
{
  memset(pos_, 0, sizeof(float)*7);
  memset(vel_old_, 0, sizeof(float)*7);
}


void FRICheck::setPos(float* pos)
{
  memcpy(pos_, pos, sizeof(float)*7);
}


void FRICheck::adjust(float *pos, float rate, float safety_factor)
{
  float vel[7];
  
  // compute increments
  for(int i=0; i < 7; i++)
    vel[i] = pos[i] - pos_[i];

  // limit hand movements
  hand_check(vel, vel_old_, pos_, rate);
  
  // limit increments
  safety_check(vel, vel_old_, pos_, rate, safety_factor);
  
  // convert position increments back to positions
  for(int i=0; i < 7; i++)
    pos[i] = pos_[i] + vel[i];

  // remember old positions and velocities
  memcpy(pos_, pos, sizeof(float)*7);
  memcpy(vel_old_, vel, sizeof(float)*7);
}


//This mapping of the limits was done choosing a value for j5, and then exploring the maximum and minimum value of j6.

#include <stdio.h>
#include <stdlib.h>


double FRICheck::j5_angles_left[]={-130.0 DEG, -120.0 DEG, -100 DEG, -80 DEG, -60 DEG, -50 DEG, -40 DEG, -35 DEG, -30 DEG, -25 DEG, -20 DEG, -10 DEG, 0 DEG, 10 DEG, 20 DEG, 30 DEG, 40 DEG, 50 DEG, 60 DEG, 70 DEG, 80  DEG,100 DEG, 120 DEG, 130 DEG};
double FRICheck::j6_min_left[]={-170 DEG, -170 DEG, -170 DEG, -170 DEG, -170 DEG,-170 DEG,-170 DEG,-170 DEG,-170 DEG,-170 DEG,-170 DEG,-170 DEG,-170 DEG,-170 DEG,-170 DEG,-155 DEG,-122 DEG,-97 DEG,-90 DEG,-86 DEG,-82 DEG,-82 DEG,-82 DEG, -87 DEG};
double FRICheck::j6_max_left[]={-2 DEG, 0 DEG, 0 DEG, 0 DEG, 0 DEG, 4 DEG, 20 DEG, 47 DEG, 65 DEG, 78 DEG, 90 DEG, 110 DEG, 170 DEG, 170 DEG, 170 DEG, 142 DEG, 140 DEG, 140 DEG, 134 DEG, 134 DEG, 138 DEG, 138 DEG, 125 DEG, 112 DEG};
const int FRICheck::length_left = 24;

double FRICheck::j5_angles_right[] = {-130 DEG,-120 DEG, -100 DEG, -80 DEG, -60 DEG, -50 DEG, -40 DEG, -35 DEG, -30 DEG, -25 DEG, -20 DEG, -10 DEG, 0 DEG, 10 DEG, 20 DEG, 30 DEG, 40 DEG, 50 DEG, 60 DEG, 70 DEG, 80 DEG, 100 DEG, 120 DEG, 130 DEG};
double FRICheck::j6_min_right[] = {-115 DEG, -130 DEG, -138 DEG, -140 DEG, -140 DEG, -138 DEG, -132 DEG, -132 DEG, -135 DEG, -150 DEG, -165 DEG, -170 DEG, -170 DEG, -170 DEG, -170 DEG, -70 DEG, -27 DEG, -12 DEG, -5.5 DEG, -3 DEG, -4 DEG, -3 DEG, 0 DEG};
double FRICheck::j6_max_right[] = { 90 DEG, 95 DEG, 84 DEG, 90 DEG, 96 DEG, 102 DEG, 123 DEG, 145 DEG, 156 DEG, 160 DEG, 170 DEG, 170 DEG, 170 DEG, 170 DEG, 170 DEG, 170 DEG, 170 DEG, 170 DEG, 170 DEG, 170 DEG, 170 DEG, 170 DEG, 170 DEG, 170 DEG};
const int FRICheck::length_right = 24;


//interpolate given two data points: x1,y1 and x2,y2
double FRICheck::interpolate(double x, double x1, double y1, double x2, double y2)
{
  double diff_y = y2 - y1;
  double diff_x = x2 - x1;
  double dx = x - x1;
  return(y1 + (diff_y/diff_x)*dx);
}

double FRICheck::slope(double x1, double y1, double x2, double y2)
{
  return (y2 - y1)/(x2 - x1);
}

int FRICheck::find_index(float j5){
  int i=0;

  // check for errors
  if ( (j5 < j5_angles_[0]) || (j5 > j5_angles_[j56_length_-1]) ){
    printf("%f not in [%f .. %f]: ", j5, j5_angles_[0],j5_angles_[j56_length_-1]);
    printf("Error, check the input value.\n");
    return(-1);
  }

  //The array must increase monotically
  //need the index where we interpolate
  int index=0;
  for (i=0; i<= j56_length_-1 ; i++){
    if ((j5 > j5_angles_[i]) & (j5 <= j5_angles_[i+1] )){
      index=i;
      break;
    }
  }
  
  return(index);
}

double FRICheck::slope_min_j6(int index)
{
  return slope(j5_angles_[index], j6_min_[index], j5_angles_[index+1], j6_min_[index+1]);
}

double FRICheck::slope_max_j6(int index)
{
  return slope(j5_angles_[index], j6_max_[index], j5_angles_[index+1], j6_max_[index+1]);
}


double FRICheck::min_j6(float j5, int index)
{
  float min_lim = interpolate(j5,
                              j5_angles_[index],  j6_min_[index],
                              j5_angles_[index+1],j6_min_[index+1]);
  return(min_lim);
}

double FRICheck::max_j6(float j5, int index)
{
  float max_lim = interpolate(j5,
                              j5_angles_[index],  j6_max_[index],
                              j5_angles_[index+1],j6_max_[index+1]);
  return(max_lim);
}


void FRICheck::hand_check(float vel[7], float vel_old[7], float pos[7], float rate)
{
  // arm side for hand limit check not set, skipping it...
  if(j56_length_ == 0 || j5_angles_ == 0 || j6_min_ == 0 || j6_max_ == 0)
    return;

  // do a special check for the last two joints, they need to use the 2d map of their limits
  // check if we are inside the nice area, if not, stop!
  double newpos5=pos[5]+vel[5];
  int index = find_index(newpos5);
  if(index == -1) {
    vel[5] = 0.0;
    vel[6] = 0.0;
  } 
  double min_lim6 = min_j6(newpos5,index);
  double max_lim6 = max_j6(newpos5,index);

  //lower limit, speed negative
  if(pos[6]+vel[6] > max_lim6) {
    // normal direction is: v_n = [slope -1]'
    // projection is: v_n * (v_n*v' / (v'*v))
    double slope = slope_max_j6(index);
    double projected_vel = (slope*vel[5] - vel[6]) / (1 + slope*slope);

    if(projected_vel > 0) {
      vel[5] = slope*projected_vel;
      vel[6] =      -projected_vel;
    } else {
      vel[5] = 0.0;
      vel[6] = 0.0;
    }
  }

  //lower limit, speed negative
  if(pos[6]+vel[6] < min_lim6) {
    // normal direction is: v_n = [slope -1]'
    // projection is: v_n * (v_n*v' / (v'*v))
    double slope = slope_min_j6(index);
    double projected_vel = (-slope*vel[5] + vel[6]) / (1 + slope*slope);

    if(projected_vel > 0) {
      vel[5] = -slope*projected_vel;
      vel[6] =        projected_vel;
    } else {
      vel[5] = 0.0;
      vel[6] = 0.0;
    }
  }
}

#define EPS FLT_EPSILON

void FRICheck::safety_check(float vel[7], float vel_old[7], float pos[7], float rate, float safety_factor)
{
  // final check on all the joints, avoiding to exceed their mechanical limits
  for(int i=0; i < 7; i++) {
    double vel_max = lim_vel_[i] * rate;
    double acc_max = lim_acc_[i]*rate*rate*safety_factor;

    // handle velocity limits
    vel[i] = (vel[i] >  vel_max) ?  vel_max : vel[i];
    vel[i] = (vel[i] < -vel_max) ? -vel_max : vel[i];

    // handle acceleration limits
    vel[i] =  (vel[i]  > vel_old[i] + acc_max) ? vel_old[i] + acc_max : vel[i];
    vel[i] =  (vel[i]  < vel_old[i] - acc_max) ? vel_old[i] - acc_max : vel[i];

    //Limit handling in two stages:
    //Stage 1:
    // calculate the maximum speed now,
    // so that we could still stop in time for the limit, i.e.
    // | v(t) = a_max*t,  x(t) = Int(v(t)*dt) = .5*a_max*t^2
    // | ==> v(x) = sqrt(2*a_max*x)
    // | x: distance to limit
    // | v: velocity
    // | a_max: maximum allowed accelearation

    // BUT: We need to know the velocity that we mustn't exceed
    //      at the end of the NEXT timestep:
    // | [ x_next = x - .5*v
    // | [ v = sqrt(2*a_max*x_next)
    // | ==> v(x) = -0.5*a + sqrt(0.5*a_max^2 + 2*a_max*x)

    if (vel[i] > 0.0) { // positive speed, worry about upper limit
      double dist_to_limit = (lim_high_[i] - margin_) - pos[i];
      if (dist_to_limit > 0.0) { //only change vel[i] if we are before the limit
        double vel_max = -0.5*acc_max + sqrt(acc_max*(2*dist_to_limit + 0.5*acc_max)) - EPS;
        
        vel[i] = ( vel[i] > vel_max ) ? vel_max : vel[i];
      }
    } else { //negative speed, worry about lower limit
      double dist_to_limit = pos[i] - (lim_low_[i] + margin_);
      if (dist_to_limit > 0.0) {
        double vel_max = -0.5*acc_max + sqrt(acc_max*(2*dist_to_limit + 0.5*acc_max)) - EPS;
        
        vel[i] = ( vel[i] < -vel_max ) ? -vel_max : vel[i];
      }
    }

    // Stage 2:
    // emergency braking taking advantage of the limit margin (0.5deg)
    if( (pos[i] < lim_low_[i]  + margin_) && ( vel[i] < 0.0 ) ) {  //negative velocity, hitting lower limit
      vel[i] =  (0.0  > vel_old[i] + acc_max) ? vel_old[i] + acc_max : 0.0;
    }
    if( (pos[i] > lim_high_[i] - margin_) && ( vel[i] > 0.0 ) ){  //positive velocity, hitting upper limit
      vel[i] =  (0.0  < vel_old[i] - acc_max) ? vel_old[i] - acc_max : 0.0;
    }
  }
}


void FRICheck::setHandLimits(double *j5_angles, double *j6_min, double *j6_max, int length)
{
  j5_angles_ = j5_angles;
  j6_min_ = j6_min;
  j6_max_ = j6_max;
  j56_length_ = length;
}

bool FRICheck::setHandSide(FRICheck::HandSide side)
{
  switch(side) {
  case LEFT:
    setHandLimits(j5_angles_left, j6_min_left, j6_max_left, length_left);
    return true;
  case RIGHT:
    setHandLimits(j5_angles_right, j6_min_right, j6_max_right, length_right);
    return true;
  default:
    setHandLimits(0, 0, 0, 0);
    return false;
  }
}
