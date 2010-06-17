
class FRICheck_test;

class FRICheck
{
public:

  // LWR arm limits
  static const float margin_;
  static const float lim_low_[7];
  static const float lim_high_[7];
  static const float lim_vel_[7];
  static const float lim_acc_[7];

  // hand checking
  static double j5_angles_left[24];
  static double j6_min_left[24]; 
  static double j6_max_left[24];
  static const int length_left;

  static double j5_angles_right[24];
  static double j6_max_right[24];
  static double j6_min_right[24]; 
  static const int length_right;

private:
  float vel_old_[7];
  float pos_[7];

  double *j5_angles_;
  double *j6_min_;
  double *j6_max_;
  int j56_length_;

  double interpolate(double x, double x1, double y1, double x2, double y2);
  double slope(double x1, double y1, double x2, double y2);
  double slope_min_j6(int index);
  double slope_max_j6(int index);

  int find_index(float j5);
  double min_j6(float j5, int index);
  double max_j6(float j5, int index);
  void hand_check(float vel[7], float vel_old[7], float pos[7], float rate);

  void safety_check(float vel[7], float vel_old[7], float pos[7], float rate, float safety_factor);

  friend class FRICheck_test;
public:
  FRICheck();

  void setPos(float* pos); //!< set robot joint angles
  float* pos() {return pos_;}  //!< robot joint angles

  //! adjust desired position according to past commands.
  void adjust(float *pos, float rate, float safety_factor);

  enum HandSide {
    NONE  = 0,
    LEFT  = 1,
    RIGHT = 2
  };

  //! convenience method to set the hand limit checking
  bool setHandSide(HandSide side);
  //! Set the array for hand limits testing
  void setHandLimits(double *j5_angles, double *j6_min, double *j6_max, int length);
};



