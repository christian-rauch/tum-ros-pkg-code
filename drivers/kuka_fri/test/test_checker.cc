#include <stdio.h>
#include <math.h>
#include <assert.h>

#include <stdlib.h>

#include "FRICheck_legacy.hh"

#include "../src/FRICheck.hh"

#define DEG *M_PI/180.0
#define RAD /M_PI*180.0

#define ITER 20000000
#define MAX_STEP (0.5 DEG)
#define EPS (0.001 DEG)

const float margin = 0.5 DEG;
const double lim_low_[7] = {-170 DEG, -120 DEG, -170 DEG, -120 DEG, -170 DEG, -120 DEG, -170 DEG};
const double lim_high_[7] = { 170 DEG,  120 DEG,  170 DEG,  120 DEG,  170 DEG,  120 DEG,  170 DEG};


class FRICheck_test
{
public:
static bool hand_check_right();
static bool hand_check_movePoint(FRICheck checker, int index, double j5, double j6, double vel_j5, double vel_j6, bool should_brake);
static bool hand_check_random();
};


bool limit_crash_check()
{
  bool test_passed = true;

  float pos_start[] = {100 DEG, 30 DEG, 70 DEG, 50 DEG, 20 DEG, 60 DEG, 70 DEG, 80 DEG};
  float rate = 0.001;

  FRICheck checker;

  // run against positive limit
  float command_speed = 1.0; // try to drive at 1 rad/s = 57 deg/s. way below limits

  float t=0.0;

  for(int joint=0; joint < 1; joint++)
  {
    float pos[7];
    for(int i=0; i < 7; i++)
      pos[i] = pos_start[i];

    // set the starting position and let it settle
    checker.setPos(pos);
    for(int i=0; i < 10; i++)
      checker.adjust(pos, rate, 1.0);

    float q_last = pos[joint];
    float q_last_last = pos[joint];

    // run against positive limits
    for(int i=0; i < 10000; i++)
    {
      t += rate;

      pos[joint] += command_speed * rate;

      checker.adjust(pos, rate, 1.0);

      // vel = (q - q_last)*rate;
      // vel_last = (q_last - q_last_last)*rate;
      //
      // acc = (vel - vel_last)*rate;
      //     = (q - 2*q_last + q_last_last)*rate*rate

      // units(acc): rad*(1/s)*(1/s) = rad*s^-2

      float vel = ((double)pos[joint] - (double) q_last) / ((double) rate);
      float acc = ((double)pos[joint] - 2.0*((double)q_last) + (double)q_last_last) / (((double)rate) * ((double)rate));

      printf("%f   %f %f %f\n", t, pos[joint], vel, acc);

      // TODO write assertions for vel and acc

      // remember last position values
      q_last_last = q_last;
      q_last = pos[joint];
    }
  }

}

bool explicit_check()
{
  bool test_passed = true;

  float pos[] = {100 DEG, 30 DEG, 70 DEG, 50 DEG, 20 DEG, 60 DEG, 70 DEG, 80 DEG};
  float rate = 0.001;

  FRICheck checker_new;

  checker_new.setHandSide(FRICheck::RIGHT);
  checker_new.setPos(pos);

  for(int i=0; i < 10; i++) {
    checker_new.adjust(pos, rate, 0.9);
  }

  for(int iter=0; iter < ITER; iter++) {
    if((iter % (ITER / 40)) == 0) {
      printf(".");
      fflush(stdout);
    }

    //create brownian motion
    for(int i=0; i < 7; i++)
      pos[i] += (drand48()*2-1)*MAX_STEP;
  
    checker_new.adjust(pos, rate, 1.0);

    for(int i=0; i < 7; i++) {
      double lim_lo = lim_low_[i] + margin;
      double lim_hi = lim_high_[i] - margin;

      if(pos[i] > lim_hi) {
        printf("%d[%d]: angle over limit by %f (%f)\n", iter, i, (pos[i] - lim_hi) RAD, pos[i] RAD);
        test_passed = false;
      }
      if(pos[i] < lim_lo) {
        printf("%d[%d]: angle under limit by %f (%f)\n", iter, i, (pos[i] - lim_lo) RAD, pos[i] RAD);
        test_passed = false;
      }
    }

    //TODO: also check for vel and acc limits

  }
  printf("Test %s\n", (test_passed) ? "PASSED" : "FAILED");

  return test_passed;
}

bool FRICheck_test::hand_check_random()
{
  bool test_passed = true;

  float pos[] = {100 DEG, 30 DEG, 70 DEG, 50 DEG, 20 DEG, 60 DEG, 70 DEG, 80 DEG};
  float rate = 0.001;

  FRICheck checker_new;

  checker_new.setHandSide(FRICheck::RIGHT);
  checker_new.setPos(pos);

  for(int i=0; i < 10; i++) {
    checker_new.adjust(pos, rate, 0.9);
  }

  double max_violation = 0.0;

  for(int iter=0; iter < ITER; iter++) {
    if((iter % (ITER / 40)) == 0) {
      printf(".");
      fflush(stdout);
    }

    //create brownian motion
    for(int i=0; i < 7; i++)
      pos[i] += (drand48()*2-1)*MAX_STEP;

    checker_new.adjust(pos, rate, 0.9);

    int index = checker_new.find_index(pos[5]);
    double lim_lo = checker_new.min_j6(pos[5], index);
    double lim_hi = checker_new.max_j6(pos[5], index);

    if(pos[6] > lim_hi) {
      printf("%d[%d]: angle over limit by %f (%f)\n", iter, 6, (pos[6] - lim_hi) RAD, pos[6] RAD);
      test_passed = false;

      max_violation = (fabs(pos[6] - lim_hi) > fabs(max_violation)) ? pos[6] - lim_hi : max_violation;
    }
    if(pos[6] < lim_lo) {
      printf("%d[%d]: angle under limit by %f (%f)\n", iter, 6, (pos[6] - lim_lo) RAD, pos[6] RAD);
      test_passed = false;

      max_violation = (fabs(pos[6] - lim_lo) > fabs(max_violation)) ? pos[6] - lim_lo : max_violation;
    }
  }

  printf("Test %s\n", (test_passed) ? "PASSED" : "FAILED");

  if(!test_passed)
    printf("max_violation: %f\n", max_violation RAD);

  return test_passed;

}

bool FRICheck_test::hand_check_movePoint(FRICheck checker, int index, double j5, double j6, double vel_j5, double vel_j6, bool should_brake)
{
  float pos[]={0,0,0,0,0,j5, j6};
  float vel[]={0,0,0,0,0,vel_j5, vel_j6};
  float vel_old[]={0,0,0,0,0,0,0};
  float rate = 0.001;

  checker.hand_check(vel, vel_old, pos, rate);
  bool res;
  if(should_brake)
    res =  (vel[5] == 0 && vel[6] == 0);
  else
    res = !(vel[5] == 0 && vel[6] == 0);
  
  if(!res)
    printf("%c pos=(%5.3f, %5.3f): vel (%5.3f %5.3f) -> (%5.3f, %5.3f)\n", (res)?'*':'x', j5 RAD, j6 RAD, vel_j5, vel_j6, vel[5], vel[6]);

  return res;
}

bool FRICheck_test::hand_check_right()
{
  bool test_passed = true;

  float pos[]={0,0,0,0,0,0,0};
  double* angles = FRICheck::j5_angles_right;
  double* min = FRICheck::j6_min_right;
  double* max = FRICheck::j6_max_right;

  double inc=0.1 DEG;

  FRICheck checker;

  checker.setHandSide(FRICheck::RIGHT);
  checker.setPos(pos);

  // rising slope, max
  for(int i=2; i < 10; i++) {
    // select a point outside limits
    double a5 = (angles[i] + angles[i+1]) / 2.0;
    double a6 = ((max[i] + max[i+1])/2.0 + 1 DEG);

    assert(i == checker.find_index(a5));

    // These movements should get stopped
    test_passed &= hand_check_movePoint(checker, i,a5, a6,  0.0, inc, true); // move up
    test_passed &= hand_check_movePoint(checker, i,a5, a6, -inc, 0.0, true); // move left
    test_passed &= hand_check_movePoint(checker, i,a5, a6, -inc, inc, true); // move up-left

    // These movements should NOT get stopped
    test_passed &= hand_check_movePoint(checker, i,a5, a6,  0.0, -inc, false); // move down
    test_passed &= hand_check_movePoint(checker, i,a5, a6,  inc,  0.0, false); // move right
    test_passed &= hand_check_movePoint(checker, i,a5, a6,  inc, -inc, false); // move down-right
  }

  // rising slope, min
  for(int i=14; i < 19; i++) {
    // select a point outside limits
    double a5 = (angles[i] + angles[i+1]) / 2.0;
    double a6 = ((min[i] + min[i+1])/2.0 - 1 DEG);

    assert(i == checker.find_index(a5));

    // These movements should get stopped
    test_passed &= hand_check_movePoint(checker, i,a5, a6, 0.0, -inc, true); // move down
    test_passed &= hand_check_movePoint(checker, i,a5, a6, inc,  0.0, true); // move right
    test_passed &= hand_check_movePoint(checker, i,a5, a6, inc, -inc, true); // move down-right

    // These movements should NOT get stopped
    test_passed &= hand_check_movePoint(checker, i,a5, a6,  0.0, inc, false); // move up
    test_passed &= hand_check_movePoint(checker, i,a5, a6, -inc, 0.0, false); // move left
    test_passed &= hand_check_movePoint(checker, i,a5, a6, -inc, inc, false); // move up-left
  }

  // falling slope, max
  for(int i=1; i < 1; i++) {
    // select a point outside limits
    double a5 = (angles[i] + angles[i+1]) / 2.0;
    double a6 = ((max[i] + max[i+1])/2.0 + 1 DEG);

    assert(i == checker.find_index(a5));

    // These movements should get stopped
    test_passed &= hand_check_movePoint(checker, i,a5, a6, 0.0, inc, true); // move up
    test_passed &= hand_check_movePoint(checker, i,a5, a6, inc, 0.0, true); // move right
    test_passed &= hand_check_movePoint(checker, i,a5, a6, inc, inc, true); // move up-right

    // These movements should NOT get stopped
    test_passed &= hand_check_movePoint(checker, i,a5, a6, -inc,  0.0, false); // move left
    test_passed &= hand_check_movePoint(checker, i,a5, a6,  0.0, -inc, false); // move down
    test_passed &= hand_check_movePoint(checker, i,a5, a6, -inc, -inc, false); // move down-left
  }

  // falling slope, min
  for(int i=0; i < 2; i++) {
    // select a point outside limits
    double a5 = (angles[i] + angles[i+1]) / 2.0;
    double a6 = ((min[i] + min[i+1])/2.0 - 1 DEG);

    assert(i == checker.find_index(a5));

    // These movements should get stopped
    test_passed &= hand_check_movePoint(checker, i,a5, a6,  0.0, -inc, true); // move up
    test_passed &= hand_check_movePoint(checker, i,a5, a6, -inc,  0.0, true); // move left
    test_passed &= hand_check_movePoint(checker, i,a5, a6, -inc, -inc, true); // move up-left

    // These movements should NOT get stopped
    test_passed &= hand_check_movePoint(checker, i,a5, a6,  0.0, inc, false); // move down
    test_passed &= hand_check_movePoint(checker, i,a5, a6,  inc, 0.0, false); // move right
    test_passed &= hand_check_movePoint(checker, i,a5, a6,  inc, inc, false); // move down-right
  }
  return test_passed;
}


int main()
{
  //limit_crash_check();

  bool t = FRICheck_test::hand_check_right();
  printf("hand check %s\n", (t) ? "PASSED" : "FAILED");
  explicit_check();
  //FRICheck_test::hand_check_random();
  return 0;
}
