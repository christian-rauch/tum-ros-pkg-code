#include "FRICheck_legacy.hh"

#include <string.h>
#include <math.h>

#define DEG *M_PI/180.0
#define RAD /M_PI*180.0



// limits as of KRC/roboter/dlrrc/cfg/rsidef7.dat
const float FRICheck_legacy::lim_low[7] = {-169.5 DEG, -119.5 DEG, -169.5 DEG, -119.5 DEG, -169.5 DEG, -119.5 DEG, -169.5 DEG};
const float FRICheck_legacy::lim_high[7] = { 169.5 DEG,  119.5 DEG,  169.5 DEG,  119.5 DEG,  169.5 DEG,  119.5 DEG,  169.5 DEG};

const float FRICheck_legacy::lim_vel[7] = {120 DEG, 120 DEG, 160 DEG, 160 DEG, 250 DEG, 220 DEG, 220 DEG};
const float FRICheck_legacy::lim_acc[7] = {1200, 1200, 1600, 1600, 2500, 4400, 4400};


void safety_check(float *vel, float *vel_old, float *pos, float rate);



FRICheck_legacy::FRICheck_legacy() :
  j5_angles(0), j6_min(0), j6_max(0), length(0)
{
  memset(pos_, 0, sizeof(float)*7);
  memset(vel_old_, 0, sizeof(float)*7);
}


void FRICheck_legacy::setPos(float* pos)
{
  memcpy(pos_, pos, sizeof(float)*7);
}



void FRICheck_legacy::adjust(float *pos, float rate)
{
  
  float vel[16];
  
  // compute increments
  for(int i=0; i < 7; i++)
    vel[i] = pos[i] - pos_[i];
  
  // limit increments
  safety_check(vel, vel_old_, pos_, rate);
    
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


double j5_angles_left[]={-120.0 DEG, -100 DEG, -80 DEG, -60 DEG, -40 DEG, -35 DEG, -30 DEG, -25 DEG, -20 DEG, -10 DEG, 0 DEG, 10 DEG, 20 DEG, 30 DEG, 40 DEG, 50 DEG, 60 DEG, 70 DEG, 80  DEG,100 DEG, 120 DEG};
double j6_min_left[]={-170 DEG, -170 DEG, -170 DEG, -170 DEG,-170 DEG,-170 DEG,-170 DEG,-170 DEG,-170 DEG,-170 DEG,-170 DEG,-160 DEG,-150 DEG,-140 DEG,-105 DEG,-90 DEG,-85 DEG,-85 DEG,-80 DEG,-75 DEG,-75 DEG};
double j6_max_left[]={-4 DEG, -4 DEG, 0 DEG, 0 DEG, 10 DEG, 30 DEG, 50 DEG, 60 DEG, 65 DEG, 70 DEG, 80 DEG, 80 DEG, 85 DEG, 90 DEG, 90 DEG, 90 DEG, 90 DEG, 90 DEG, 90 DEG, 90 DEG, 90 DEG};
int length_left = 21;

double j5_angles_right[] = {-120 DEG, -100 DEG, -80 DEG, -60 DEG, -40 DEG, -35 DEG, -30 DEG, -25 DEG, -20 DEG, -10 DEG, 0 DEG, 10 DEG, 20 DEG, 30 DEG, 40 DEG, 50 DEG, 60 DEG, 70 DEG, 80 DEG, 100 DEG, 120 DEG};
double j6_max_right[] = { 84 DEG, 84 DEG, 90 DEG, 90 DEG, 100 DEG, 120 DEG, 140 DEG, 150 DEG, 155 DEG, 160 DEG, 170 DEG, 170 DEG, 170 DEG, 170 DEG, 170 DEG, 170 DEG, 170 DEG, 170 DEG, 170 DEG, 170 DEG, 170 DEG};
double j6_min_right[] = {-80 DEG, -80 DEG, -80 DEG, -80 DEG, -80 DEG, -80 DEG, -80 DEG, -80 DEG, -80 DEG, -80 DEG, -80 DEG, -70 DEG, -60 DEG, -50 DEG, -15 DEG, 0 DEG, 5 DEG, 5 DEG, 10 DEG, 15 DEG, 15 DEG};
int length_right = 21;

// general limits (applying to both arms)
double liml[7] = {-169.5 DEG, -119.5 DEG, -169.5 DEG, -119.5 DEG, -169.5 DEG, -119.5 DEG, -169.5 DEG};
double limh[7] = { 169.5 DEG,  119.5 DEG,  169.5 DEG,  119.5 DEG,  169.5 DEG,  119.5 DEG,  169.5 DEG};


//interpolate given two data points: x1,y1 and x2,y2
double FRICheck_legacy::interpolate(double x, double x1, double y1, double x2, double y2)
{
  double diff_y=y2 - y1;
  double diff_x=x2 - x1;
  double dx=x - x1;
  return(y1 + (diff_y/diff_x)*dx);
}

int FRICheck_legacy::find_index(float j5){
    //printf("Array length: %d\n",length);
    
    int i=0;

    // check for errors
    if ( (j5 < j5_angles[0]) || (j5 > j5_angles[length-1]) ){
      printf("%f not in [%f .. %f]: ", j5, j5_angles[0],j5_angles[length-1]);
      printf("Error, check the input value.\n");
      return(-1);
    }

    //The array must increase monotically
    //need the index where we interpolate
    int index=0;
    for (i=0; i<= length-1 ; i++){
        if ((j5 > j5_angles[i]) & (j5 <= j5_angles[i+1] )){
            index=i;
            break;
        }
    }

    return(index);
}

double FRICheck_legacy::min_j6(float j5, int index)
{
  float min_lim = interpolate(j5,
                              j5_angles[index],  j6_min[index],
                              j5_angles[index+1],j6_min[index+1]);
  return(min_lim);
}

double FRICheck_legacy::max_j6(float j5, int index)
{
  float max_lim = interpolate(j5,
                              j5_angles[index],  j6_max[index],
                              j5_angles[index+1],j6_max[index+1]);
  return(max_lim);
}


void FRICheck_legacy::safety_check(float *vel, float *vel_old, float *pos, float rate)
{
  if(length == 0 || j5_angles == 0 || j6_min == 0 || j6_max == 0) {
    printf("arm side for safety limits checking not set, panicking...\n");
    exit(-1);
  }

  //The following values vel_max and acc_max are decided by us
  double vel_max = (100 DEG) * rate;
  double acc_max = (600 DEG) * rate * rate;

  //These are the hard acceleration limits defined in the RSI-Def7 file
  double acc_max_hard[7];
  acc_max_hard[0]=(1200 DEG) * rate * rate;
  acc_max_hard[1]=(1200 DEG) * rate * rate;
  acc_max_hard[2]=(1600 DEG) * rate * rate;
  acc_max_hard[3]=(1600 DEG) * rate * rate;
  acc_max_hard[4]=(2500 DEG) * rate * rate;
  acc_max_hard[5]=(4400 DEG) * rate * rate;
  acc_max_hard[6]=(4400 DEG) * rate * rate;

  //rate down the accel limits for safety
  for (unsigned int i=0; i <7 ; i++) {
    acc_max_hard[i]=acc_max_hard[i]*0.9;
  }

  // do a special check for the last two joints, they need to use the 2d map of their limits
  // check if we are inside the nice area, if not, stop!
  double newpos5=pos[5]+vel[5];
  int index = find_index(newpos5);
  double min_lim6=min_j6(newpos5,index);
  double max_lim6=max_j6(newpos5,index);
  int n_cycles=5;

  if(index == -1) {
    vel[5] = 0.0;
    vel[6] = 0.0;
  }
  
  //lower limit, speed negative
  if ( ( pos[6]+vel[6] < min_lim6 ) && (pos[6]+n_cycles*vel[6] < min_lim6 ) ) {
    vel[6] =  (0.0  > vel_old[6] + acc_max) ? vel_old[6] + acc_max : 0.0;
    //limit 5 also (both sides)
    vel[5] =  (0.0  > vel_old[5] + acc_max) ? vel_old[5] + acc_max : 0.0;
    vel[5] =  (0.0  < vel_old[5] - acc_max) ? vel_old[5] - acc_max : 0.0;
  }

  //upper limit, speed positive
  if (( pos[6]+vel[6] > max_lim6 ) && ( pos[6]+n_cycles*vel[6] > max_lim6  )  ) {
    vel[6] =  (0.0  < vel_old[6] - acc_max) ? vel_old[6] - acc_max : 0.0;
    //limit 5 also (both sides)
    vel[5] =  (0.0  > vel_old[5] + acc_max) ? vel_old[5] + acc_max : 0.0;
    vel[5] =  (0.0  < vel_old[5] - acc_max) ? vel_old[5] - acc_max : 0.0;
  }


  // final check on all the joints, avoiding to exceed their mechanical limits
  for(int i=0; i < 7; i++) {

    // handle velocity limits
    vel[i] = (vel[i] >  vel_max) ?  vel_max : vel[i];
    vel[i] = (vel[i] < -vel_max) ? -vel_max : vel[i];

    // handle normal acceleration limits
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
    // | [ x_next = x - v
    // | [ v = sqrt(2*a_max*x_next)
    // | ==> v(x) = -a + sqrt(a_max^2 + 2*a_max*x)
    if (vel[i] > 0.0) { // positive speed, worry about upper limit
        double dist_to_limit = limh[i] - pos[i];
        if (dist_to_limit > 0.0) { //only change vel[i] if we are before the limit
            double vel_max = -acc_max_hard[i] + sqrt(acc_max_hard[i]*(2*dist_to_limit+acc_max_hard[i]));

            vel[i] = ( vel[i] > vel_max ) ? vel_max : vel[i];
        }
    } else { //negative speed, worry about lower limit
        double dist_to_limit = pos[i] - liml[i];
        if (dist_to_limit > 0.0) {
            double vel_max = -acc_max_hard[i] + sqrt(acc_max_hard[i]*(2*dist_to_limit+acc_max_hard[i]));

            vel[i] = ( vel[i] < -vel_max ) ? -vel_max : vel[i];
        }
    }

    // Stage 2:
    // emergency braking taking advantage of the limit margin (0.5deg)
    if( ( (pos[i]) < liml[i] ) && ( vel[i] < 0.0 ) ) {  //negative velocity, hitting lower limit
      vel[i] =  (0.0  > vel_old[i] + acc_max_hard[i]) ? vel_old[i] + acc_max_hard[i] : 0.0;
    }
    if( ( (pos[i]) > limh[i] ) && ( vel[i] > 0.0 ) ){  //positive velocity, hitting upper limit
      vel[i] =  (0.0  < vel_old[i] - acc_max_hard[i]) ? vel_old[i] - acc_max_hard[i] : 0.0;
    }
  }
}


int FRICheck_legacy::safety_set_side(int side)
{
  switch(side) {
  case SAFETY_LEFT:
    j5_angles = j5_angles_left;
    j6_min = j6_min_left;
    j6_max = j6_max_left;
    length = length_left;
    return 1;
  case SAFETY_RIGHT:
    j5_angles = j5_angles_right;
    j6_min = j6_min_right;
    j6_max = j6_max_right;
    length = length_right;
    return 1;
  default:
    return 0;
  }
}
