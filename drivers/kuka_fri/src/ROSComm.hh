
#include "FRIData.hh"
#include <pthread.h>

#include <string>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <sensor_msgs/JointState.h>
#include <kuka_fri/ImpedanceCommand.h>
#include <soft_runstop/Handler.h>

class FRIThread;

class ROSComm {
public:
  ROSComm();
  ~ROSComm();
  bool configure(FRIThread *fri);
  bool open();
  void publishData(const RobotData &data, const RobotCommand &cmd);
  void publishStatus(const RobotStatus &status);
  bool runstop();

  // threading support
  void start(FRIThread *fri);
  void finish();
private:
  std::string side_;
  soft_runstop::Handler* soft_runstop_handler_;
  RobotStatus status_;
  void status_update(diagnostic_updater::DiagnosticStatusWrapper &s);
  void runstop_receiver(const std_msgs::BoolConstPtr &msg);
  void impedanceCommand(const kuka_fri::ImpedanceCommand::ConstPtr& msg);
  ros::NodeHandle *n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

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
