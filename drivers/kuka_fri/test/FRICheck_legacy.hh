#define SAFETY_LEFT 0
#define SAFETY_RIGHT 1



class FRICheck_legacy
{
private:
  float vel_old_[7];
  float pos_[7];

  static const float lim_low[7];
  static const float lim_high[7];
  static const float lim_vel[7];
  static const float lim_acc[7];

  // legacy ///
  double *j5_angles;
  double *j6_min;
  double *j6_max;
  int length;
  /////////////

  double interpolate(double x, double x1, double y1, double x2, double y2);
  int find_index(float j5);
  double min_j6(float j5, int index);
  double max_j6(float j5, int index);
  void safety_check(float *vel, float *vel_old, float *pos, float rate);

public:
  // legacy ///
  int safety_set_side(int side);
  /////////////


  FRICheck_legacy();
  //! set robot position
  void setPos(float* pos);

  //! robot position
  float* pos() {return pos_;}

  //! adjust desired position according to past commands.
  void adjust(float *pos, float rate);
};



