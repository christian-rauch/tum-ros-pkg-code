#include <stdio.h>
#include <float.h>
#include <assert.h>
#include <string.h>
#include <math.h>

#include "../src/FRICheck.hh"

#define DEG *M_PI/180.0
#define RAD /M_PI*180.0

//////////////////////////////////////////////////
// helper function

// for concatenating file names
char* scat(const char* str1, const char* str2)
{
  static char buffer[1024];
  buffer[0] = '\0';

  return strcat(strcat(buffer, str1), str2);
}


void extractLimits(FILE* f, double *angles, int len,
                   double* min6, double* min7, double* max6, double *max7)
{
  char buffer[1024];

  // initialize limits
  for(int i=0; i < len; i++) {
    min6[i] =  FLT_MAX;  min7[i] =  FLT_MAX;
    max6[i] = -FLT_MAX;  max7[i] = -FLT_MAX;
  }

  // bin limits
  double bin_limits[len+1];
  double *bin_low=&(bin_limits[0]), *bin_high=&(bin_limits[1]);

  // set collection bins around desired angles for J6
  bin_limits[0] = -FLT_MAX;
  for(int i=0; i < len; i++)
    bin_limits[i+1] = (angles[i+1] + angles[i])/2.0;
  bin_limits[len] = FLT_MAX;

  int bin=0;
  float j6, j7;
  while(fgets(buffer, sizeof(buffer), f) != 0) {
    if(buffer[0] == '#')
      continue;
    if(sscanf(buffer, "%f %f", &j6, &j7) != 2)
      continue;

    // find correct bin
    while(j6 < bin_low[bin]  && bin > 0)
      bin--;
    while(j6 > bin_high[bin] && bin < len)
      bin++;

    // now we should have the correct bin ...
    assert(bin_low[bin] <= j6 && j6 <= bin_high[bin]);

    // record min and max
    if(j7 > max7[bin]) {
      max6[bin] = j6;
      max7[bin] = j7;
    }
    if(j7 < min7[bin]) {
      min6[bin] = j6;
      min7[bin] = j7;
    }
  }
}

void printRawLimits(FILE* f, int len, double* min6, double* min7, double* max6, double *max7)
{
  for(int i=0; i < len; i++)
    fprintf(f, "%f %f\n", min6[i], min7[i]);

  for(int i=len-1; i >= 0; i--)
    fprintf(f, "%f %f\n", max6[i], max7[i]);

  fprintf(f, "%f %f\n", min6[0], min7[0]);
}

void printLimits(FILE* f, double* angles, double* min, double* max, int len)
{
  for(int i=0; i < len; i++)
    fprintf(f, "%f %f\n", angles[i], min[i]);

  for(int i=len-1; i >= 0; i--)
    fprintf(f, "%f %f\n", angles[i], max[i]);

  fprintf(f, "%f %f\n", angles[0], min[0]);
}

double interpolate(double x1, double x2, double y1, double y2, double x)
{
  double t = (x - x1)/(x2 - x1);
  return (1 - t)*y1 + t*y2;
}

void interpolateLimits(int len, double* min6, double* min7, double* max6, double *max7, double* angles, double *min, double *max)
{
  // no interpolation possible at j6 limits
  min[0] = min7[0];
  max[0] = max7[0];

  min[len-1] = min7[len-1];
  max[len-1] = max7[len-1];

  // do interpolation with other points
  for(int i=1; i < len-1; i++) {
    if(min6[i] < angles[i])
      min[i] = interpolate(min6[i], min6[i+1], min7[i], min7[i+1], angles[i]);
    else
      min[i] = interpolate(min6[i-1], min6[i], min7[i-1], min7[i], angles[i]);
    
    if(max6[i] < angles[i])
      max[i] = interpolate(max6[i], max6[i+1], max7[i], max7[i+1], angles[i]);
    else
      max[i] = interpolate(max6[i-1], max6[i], max7[i-1], max7[i], angles[i]);
  }
}

#define MARGIN_IN  17 DEG
#define MARGIN_OUT  3 DEG

bool testLimits(int len, double *angles, double* min, double *max, double* mine, double* maxe)
{
  bool result = true;
  for(int i=0; i < len; i++) {
    if(min[i] < mine[i] - MARGIN_OUT) {
      printf("lower limit @%f too low  (%f << %f) \n", angles[i] RAD, min[i] RAD, mine[i] RAD);
      result = false;
    }
    if(min[i] > mine[i] + MARGIN_IN) {
      printf("lower limit @%f too high (%f >> %f) \n", angles[i] RAD, min[i] RAD, mine[i] RAD);
      result = false;
    }

    if(max[i] > maxe[i] + MARGIN_OUT) {
      printf("upper limit @%f too high (%f >> %f) \n", angles[i] RAD, max[i] RAD, maxe[i] RAD);
      result = false;
    }
    if(max[i] < maxe[i] - MARGIN_IN) {
      printf("upper limit @%f too low (%f << %f) \n", angles[i] RAD, max[i] RAD, maxe[i] RAD);
      result = false;
    }
  }
  return result;
}

bool verifyHandAngles(int len, double* angles, double* min, double *max, const char* side)
{
  FILE *f;

  // auto-extract limits
  f = fopen(scat("limits_recording_", side), "r");
  double min6[len], max6[len], min7[len], max7[len], mine[len], maxe[len];
  extractLimits(f, angles, len, min6, min7, max6, max7);
  fclose(f);

  interpolateLimits(len, min6, min7, max6, max7, angles, mine, maxe);

  bool result = testLimits(len, angles, min, max, mine, maxe);

  // print auto-extracted limits
  f = fopen(scat("limits_extracted_", side), "w");
  printRawLimits(f, len, min6, min7, max6, max7);
  fclose(f);

  // print interpolated limits
  f = fopen(scat("limits_interpolated_", side), "w");
  printLimits(f, angles, mine, maxe, len);
  fclose(f);

  // print current state
  f = fopen(scat("limit_", side), "w");
  printLimits(f, angles, min, max, len);
  fclose(f);

  return result;
}

int main()
{
  bool r, result = true;

  printf("testing left hand angles ... ");
  r = verifyHandAngles(FRICheck::length_left,
                       FRICheck::j5_angles_left,
                       FRICheck::j6_min_left,
                       FRICheck::j6_max_left,
                       "left");
  printf("%s\n", (r) ? "PASSED" : "FAILED");
  result &= r;

  printf("testing right hand angles ... ");
  verifyHandAngles(FRICheck::length_right,
                   FRICheck::j5_angles_right,
                   FRICheck::j6_min_right,
                   FRICheck::j6_max_right,
                   "right");
  printf("%s\n", (r) ? "PASSED" : "FAILED");
  result &= r;

  return (result) ? 0 : -1;
}
