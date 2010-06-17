
#include "FRIData.hh"
#include <pthread.h>

#include <string>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

class FRIThread;

class ROSComm {
public:
  ROSComm();
  bool configure(FRIThread *fri);
  bool open();
  void publishData(const RobotData &data, const RobotCommand &cmd);
  void publishStatus(const RobotStatus &status);
  bool receiveCommand(RobotCommand* cmd);
  bool runstop();

  // threading support
  void start(FRIThread *fri);
  void finish();
private:
  std::string side_;
  bool runstop_;
  RobotStatus status_;
  void status_update(diagnostic_updater::DiagnosticStatusWrapper &s);
  void runstop_receiver(const std_msgs::BoolConstPtr &msg);
  ros::NodeHandle *n_;
  ros::Publisher pub_;
  ros::Subscriber runstop_sub_;
  diagnostic_updater::Updater *diagnostic_;
  sensor_msgs::JointState joint_state_;

  // threading support
  FRIThread* fri_;
  double rate_;
  void* run();
  static void* run_s(void *ptr) { return ((ROSComm *) ptr)->run(); }

  pthread_t thread_;
  bool running_, exitRequested_;
};
