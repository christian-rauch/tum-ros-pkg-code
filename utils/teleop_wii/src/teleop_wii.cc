
/**

@mainpage teleop_wii

@htmlinclude manifest.html

@b teleop_wii is a teleop controller for a base which uses the Nunchuk
extension of a wiimote. This controller is designed for an omnidirectional
base. The joystick controls the lateral movement and the roll controls turning.
Button Z is the dead-man switch.


@section topic ROS topics

Publishes to (name/type):
- @b "scan" / <a href="http://www.ros.org/doc/api/geometry_msgs/html/msg/Twist.html">geometry_msgs/Twist</a> : The velocity that the wiimote is commanding.
- @b "wiimote" / <a href="http://ros.org/doc/api/joy/html/msg/Joy.html">joy/Joy</a> : Raw data of the wiimote and nunchuck (excluding IR sensor).


@section services
- None

@section parameters ROS parameters
- @b "/teleop_wii/speed": @b [double] maximum speed (default: 0.1 m/s)
- @b "/teleop_wii/aspeed": @b [double] speed ratio between rotational / lateral speeds (default: 1.3 rad/m)

*/



#include <unistd.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <joy/Joy.h>

#include <roslib/Time.h>


#include <stdio.h>
#include <stdlib.h>

#include <pthread.h>
#include <errno.h>
#include <math.h>

#include <cwiid.h>


class WiiMote;
//WiiMote *instance;


class WiiMote {

public:
  WiiMote();
  ~WiiMote();
  int init(int retries=5);
  void getraw(double *acc, int *buttons, double *nunchuk_stick,
              double *nunchuk_acc, int *nunchuk_buttons);
  int getdata(double *x, double *y, double *a, int *deadkey);
  void shutdown();

private: 
  void cwiid_callback(cwiid_wiimote_t *wiimote, int mesg_count,
                      union cwiid_mesg mesg[], struct timespec *timestamp);
  static void cwiid_callback_s(cwiid_wiimote_t *wiimote, int mesg_count,
                               union cwiid_mesg mesg[], struct timespec *timestamp);

  cwiid_wiimote_t *wiimote_;   // Wiimote handle
  struct acc_cal calib_;       // Wiimote calibration data

  pthread_cond_t  cond_;
  pthread_mutex_t mutex_;

  int deadkey_;
  double x_, y_, a_, a_last_;
  bool fresh_, ok_;

  // rotation specific parameters
  const static double dead_ = 0.2; // size of deadzone [rad]
  const static double gain_ = 0.8; // gain outside deadzone
  const static double iir_gain_ = 0.3; // gain of iir filter

  // if this timeout elapses without any message from the wiimote,
  // we assume connection loss
  const static int message_timeout_=1;

  WiiMote* last_instance_;
  static WiiMote* instance_;
};

WiiMote* WiiMote::instance_ = 0;


WiiMote::WiiMote()
{
  x_=0.0; y_=0.0; a_=0.0; deadkey_=0;

  pthread_cond_t  c = PTHREAD_COND_INITIALIZER;
  cond_  = c;

  pthread_mutex_t m = PTHREAD_MUTEX_INITIALIZER;
  mutex_ = m;

  fresh_ = false;
  ok_ = true;

  last_instance_ = instance_;
  instance_ = this;
}

WiiMote::~WiiMote()
{
  instance_ = last_instance_;
}

void WiiMote::cwiid_callback_s(cwiid_wiimote_t *wiimote, int mesg_count,
                               union cwiid_mesg mesg[], struct timespec *timestamp)
{
  WiiMote *w;
  for(w = instance_; w != 0; w = w->last_instance_)
    if(w->wiimote_ == wiimote)
      break;

  if(w)
    w->cwiid_callback(wiimote, mesg_count, mesg, timestamp);
}


void WiiMote::cwiid_callback(cwiid_wiimote_t *wiimote, int mesg_count,
                             union cwiid_mesg mesg[], struct timespec *timestamp)
{
  int i, j, deadkey;
  double acc[3], angle, sig, d, l, a, a_iir;

  double a_gain = 100 / (gain_ * (M_PI/2.0 - dead_));  /* scale angle values to [-1.0 .. 1.0] */

  for (i = 0; i < mesg_count; i++) {
    switch (mesg[i].type) {

    case CWIID_MESG_NUNCHUK:

      /* convert nunchuck commands to (x, y, a) velocities: The
       *  joystick is mapped to x and y, the roll (after some
       *  filtering) is mapped to 'a'.
       */

      pthread_mutex_lock(&mutex_);

      // Normalize force vectors to [-1 .. 1]. 
      for (j = 0; j < 3; j++)
        acc[j] = ((double) mesg[i].nunchuk_mesg.acc[j]
                  - calib_.zero[j]) / (calib_.one[j] - calib_.zero[j]);

      // Length of force vector
      l = sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);

      angle = atan2(acc[0], acc[2]) / l;  // Compute roll angle.

      // IIR filter
      a_iir = a_last_ * (1 - iir_gain_) + iir_gain_ * angle;
      a_last_ = a_iir; // Remember last iir value.
      
      sig = (a_iir < 0) ? -1 : 1;          // Do we have positive or negative control input?
      d = (fabs(a_iir) > dead_) ? 1 : 0;    // Are we inside deadzone?
      a = d * gain_ * (a_iir - sig * dead_); // Compute deadzone gain.

      // Deadman switch
      deadkey = (mesg[i].nunchuk_mesg.buttons & CWIID_NUNCHUK_BTN_Z) ? 1 : 0;

      // scale to [-1 .. 1]
      y_ = 0.01 * deadkey * (-mesg[i].nunchuk_mesg.stick[CWIID_X] + 127);
      x_ = 0.01 * deadkey * ( mesg[i].nunchuk_mesg.stick[CWIID_Y] - 135);
      a_ = 0.01 * deadkey * (-a*a_gain);

      // limit to [-1 .. 1]
      x_ = (x_ > 1) ? 1 : (x_ < -1) ? -1 : x_;
      y_ = (y_ > 1) ? 1 : (y_ < -1) ? -1 : y_;
      a_ = (a_ > 1) ? 1 : (a_ < -1) ? -1 : a_;
      deadkey_ = deadkey;

      fresh_ = true;
      pthread_cond_broadcast(&cond_);
      pthread_mutex_unlock(&mutex_);

      break;
    case CWIID_MESG_ERROR:
      ROS_DEBUG("(callback): Got wiimote error message: %d", mesg->error_mesg.error);
      ok_ = false;
      break;
    default:
      ROS_DEBUG("(callback): Unknown Report %d", (int) mesg[i].type);
      break;
    }
  }
}


int WiiMote::init(int retries)
{
  bdaddr_t bdaddr = {{0, 0, 0, 0, 0, 0}}; //BDADDR_ANY

  /* Connect to the wiimote. Retry a few times if it doesn't work. */
  for (int i = 0; i < retries; i++) {
    ROS_INFO("Trying to connect, press 1 + 2 on the wiimote...");
    if ((wiimote_ = cwiid_open(&bdaddr, 0)) != NULL)
      break;
  }
  if (!wiimote_) {
    ROS_ERROR("Unable to connect to wiimote.");
    return 0;
  }
  ROS_INFO("Connected.");

  if (cwiid_set_rpt_mode(wiimote_, CWIID_RPT_BTN | CWIID_RPT_NUNCHUK | CWIID_RPT_ACC)) {
    ROS_ERROR("Error setting report mode.");
    return 0;
  }

  cwiid_get_acc_cal(wiimote_, CWIID_EXT_NUNCHUK, &calib_);

  cwiid_enable(wiimote_, CWIID_FLAG_MESG_IFC);
  if (cwiid_set_mesg_callback(wiimote_, WiiMote::cwiid_callback_s)) {
    ROS_ERROR("Unable to set message callback.");
    return 0;
  }
  return 1;
}


void WiiMote::shutdown()
{
  if(wiimote_)
    if(cwiid_close(wiimote_))
      ROS_ERROR("Error on wiimote disconnect.");
  wiimote_ = 0;
}


void WiiMote::getraw(double *acc, int *buttons,
                     double *nunchuk_stick, double *nunchuk_acc, int *nunchuk_buttons)
{
  struct cwiid_state state;
  cwiid_get_state(wiimote_, &state);

  if(acc)
    for(int i=0; i < 3; i++)
      acc[i] = state.acc[i];
  if(buttons)
    *buttons = state.buttons;
  if(nunchuk_stick)
    for(int i=0; i < 2; i++)
      nunchuk_stick[i] = state.ext.nunchuk.stick[i];
  if(nunchuk_acc)
    for(int i=0; i < 3; i++)
      nunchuk_acc[i] = state.ext.nunchuk.acc[i];
  if(nunchuk_buttons)
    *nunchuk_buttons = state.ext.nunchuk.buttons;
}


int WiiMote::getdata(double  *x, double *y, double *a, int *deadkey)
{
  struct timespec timeout;
  clock_gettime(CLOCK_REALTIME, &timeout);
  timeout.tv_sec += message_timeout_;

  pthread_mutex_lock(&mutex_);

  // wait for fresh wiimote data
  while(!fresh_) {
    int rc = pthread_cond_timedwait(&cond_, &mutex_, &timeout);
    // the error state is only set after 20 seconds, so we use a
    // 1-second timeout ourselves.
    if(rc == ETIMEDOUT || !ok_) {
      // we have an error - give up and return
      pthread_mutex_unlock(&mutex_);
      return 0;
    }
  }

  // copy the data
  if(x) *x = x_;
  if(y) *y = y_;
  if(a) *a = a_;
  if(deadkey) *deadkey = deadkey_;

  fresh_ = false;

  pthread_mutex_unlock(&mutex_);
  return 1;
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "wii");
  ros::NodeHandle n("~");

  double speed, aspeed;

  WiiMote mote;

  // get maximum speed (m/s)
  n.param("speed", speed, 0.1);
  n.param("aspeed", aspeed, 1.3);

  if(!mote.init())
    return 0;

  ros::Publisher pub =  n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::Publisher pub_j =  n.advertise<joy::Joy>("wiimote", 1);

  ros::Time t = ros::Time::now();

  int zero=0;  // count the number of released-deadkey readings
  while(n.ok()) {
    int d;
    double x, y, a;
    if(!mote.getdata(&x, &y, &a, &d)) {
      ROS_ERROR("Wiimote connection problems, exiting...");
      mote.shutdown();
      return -1;
    }

    if(d != 0 || zero < 3) {
      geometry_msgs::Twist msg;
      msg.linear.x = speed*x;
      msg.linear.y = speed*y;
      msg.angular.z = speed*a*aspeed;
      pub.publish(msg);

      zero = (d==0) ? zero+1 : 0;
    }

    // publish raw wiimote data
    {
      int btn, nbtn;
      double acc[3], nacc[3], nstick[2];
      mote.getraw(acc, &btn, nstick, nacc, &nbtn);

      joy::Joy msg;
      msg.axes.resize(8);
      msg.buttons.resize(15);

      msg.axes[0] = acc[0];
      msg.axes[1] = acc[1];
      msg.axes[2] = acc[2];
      msg.axes[3] = nacc[0];
      msg.axes[4] = nacc[1];
      msg.axes[5] = nacc[2];
      msg.axes[6] = nstick[0];
      msg.axes[7] = nstick[1];

      for(int i=0; i < 13; i++)
        msg.buttons[i] = (btn & (1 << i)) >> i;
      
      msg.buttons[13] = nbtn & 1;
      msg.buttons[14] = (nbtn & 2) >> 1;

      pub_j.publish(msg);
    }
    

    if(ros::Time::now() - t > ros::Duration(1.0)) {
      // reread speed parameters
      n.getParam("speed", speed);
      n.getParam("aspeed", aspeed);
      t = ros::Time::now();
    }
  }

  mote.shutdown();

  return 0;
}
