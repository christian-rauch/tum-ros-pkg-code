

#include "ROSComm.hh"
#include "FRIComm.hh"
#include "FRICheck.hh"
#include <signal.h>
#include <pthread.h>

using namespace std;

ROSComm::ROSComm() :
  soft_runstop_handler_(0), n_(0), diagnostic_(0),
  running_(false), exitRequested_(false)
{
  memset(&status_, 0, sizeof(status_));
}

ROSComm::~ROSComm()
{
  delete n_;
  delete diagnostic_;
  delete soft_runstop_handler_;
}

bool ROSComm::configure(FRIThread *fri)
{
  if(!ros::master::check())
    return false;

  if(!n_)
    n_ = new ros::NodeHandle("~");

  int local_port=0, remote_port=0;
  std::string remote_ip;
  bool ok = true;

  ok &= n_->getParam("local_port", local_port);
  ok &= n_->getParam("remote_port", remote_port);
  ok &= n_->getParam("remote_ip", remote_ip);

  if(ok)
    fri->configureNetwork(local_port, remote_ip.c_str(), remote_port);

  n_->param("side", side_, std::string("none"));

  if(side_ == "left")
    fri->limitChecker()->setHandSide(FRICheck::LEFT);
  else if(side_ == "right")
    fri->limitChecker()->setHandSide(FRICheck::RIGHT);
  else
    fri->limitChecker()->setHandSide(FRICheck::NONE);

  if(ok)
    ROS_INFO("connecting to %s arm: [%d -> %s:%d]",
             side_.c_str(), local_port, remote_ip.c_str(), remote_port);

  joint_state_.name.resize(7);
  joint_state_.position.resize(7);
  joint_state_.velocity.resize(7);
  joint_state_.effort.resize(7);

  for(int i=0; i < 7; i++) {
    joint_state_.name[i] = side_+string("_arm_")+string(1,'0'+i)+string("_joint");
  }

  n_->param("rate", rate_, 125.0);

  return ok;
}

#define CLAMP(v, min, max) ((v <= min) ? min : ((v >= max) ? max : v))

void ROSComm::status_update(diagnostic_updater::DiagnosticStatusWrapper &s)
{
  int quality = CLAMP(status_.quality, 0, 3);
  int state = CLAMP(status_.state, 0, 2);
  int control_mode = CLAMP(status_.controlMode, 0, 3);

  char qualities[] = {'x', '-', '+', '*'};
  char comm_states[] = {'0', 'M', 'C'};

  unsigned char level = (state == 2 && status_.power == 0x7f)
    ? (unsigned char) diagnostic_msgs::DiagnosticStatus::OK
    : (unsigned char) diagnostic_msgs::DiagnosticStatus::ERROR;
  
  s.summaryf(level, "%c%c",
    comm_states[state],
    qualities[quality]);

  const char *quality_strings[] = {"Unacceptable", "Bad", "Okay", "Perfect"};
  const char *state_strings[] = {"Reset", "Monitor", "Command"};
  const char *control_modes[] = {"Other", "Position", "Cartesian Impedance", "Joint Impedance"};

  double freq = (status_.cmdSampleTime==0.0) ? 0.0 : 1.0/status_.cmdSampleTime;

  s.addf("Runstop", "%s", (status_.runstop) ? "ON" : "off");
  s.addf("Interface Mode", "%s", state_strings[state]);
  s.addf("Control Mode", "%s", control_modes[control_mode]);
  s.addf("Comm. Status", "%s", (status_.connected) ? "Connected": "Offline");
  s.addf("Comm. frequency", "%3.1f Hz", freq);
  s.addf("Comm. answer rate", "%3.3f", status_.answerRate);
  s.addf("Comm. latency", "%5.5f", status_.latency);
  s.addf("Comm. jitter", "%5.5f", status_.jitter);
  s.addf("Comm. miss rate", "%5.5f", status_.missRate);
  s.addf("Comm. miss counter", "%d", status_.missCounter);
  s.addf("Comm. quality", "%s", quality_strings[quality]);
  s.addf("Safety Factor", "%f", status_.safety);
  s.addf("Mean computation time", "%d us", status_.calcTimeMean);
  s.addf("Max computation time", "%d us", status_.calcTimeMax);
  s.addf("Motor State", "%s", (status_.power == 0x7f) ? "ON" : "OFF");

  float *t=status_.temperature;
  s.addf("Temperatures [deg C]", "%3.1f %3.1f %3.1f %3.1f %3.1f %3.1f %3.1f",
         t[0], t[1], t[2], t[3], t[4], t[5], t[6]);
}

bool ROSComm::open()
{
  // avoid blocking for too long...
  if(!ros::master::check())
    return false;

  if(!n_)
    n_ = new ros::NodeHandle("~");

  soft_runstop_handler_ = new soft_runstop::Handler(Duration(0.5));
  pub_ = n_->advertise<sensor_msgs::JointState>("/joint_states", 1);
  sub_ = n_->subscribe("command", 1, &ROSComm::impedanceCommand, this);

  string Side = side_;
  Side[0] = side_[0] + ('A'-'a');

  diagnostic_ = new diagnostic_updater::Updater();
  diagnostic_->setHardwareID(string("kuka_lwr_")+string(side_));
  diagnostic_->add("Arm "+Side, this, &ROSComm::status_update);

  return true;
}


void ROSComm::publishData(const RobotData &data, const RobotCommand &cmd)
{
  //TODO: make a timestamp when message is received in FRIComm, not now.
  joint_state_.header.stamp = ros::Time::now();

  for(int i=0; i < 7; i++) {
    joint_state_.position[i] = data.position[i];  
    joint_state_.velocity[i] = 0.0; // TODO: remember old pos and calculate
    double dist = cmd.stiffness[i]*fabs(data.position[i] - data.commanded[i]);
    double stiffness = cmd.stiffness[i];
    joint_state_.effort[i] = stiffness*dist; //TODO: compute effort according to sine distr.
  }

  pub_.publish(joint_state_); 
}


void ROSComm::publishStatus(const RobotStatus &status)
{
  status_ = status;
  diagnostic_->update();
}


void ROSComm::impedanceCommand(const kuka_fri::ImpedanceCommand::ConstPtr& msg)
{
  RobotCommand cmd = fri_->cmd();

  if (msg->velocity.size() == 7)
    for (int i=0; i<7; i++)
      cmd.command[i] = msg->velocity[i];

  if (msg->stiffness.size() == 7)
    for (int i=0; i<7; i++)
      cmd.stiffness[i] = msg->stiffness[i];

  if (msg->damping.size() == 7)
    for (int i=0; i<7; i++)
      cmd.damping[i] = msg->damping[i];

  if (msg->add_torque.size() == 7)
    for (int i=0; i<7; i++)
      cmd.addTorque[i] = msg->add_torque[i];

  fri_->setCmd(cmd);
}


bool ROSComm::runstop()
{
  return soft_runstop_handler_->getState();
}


void ROSComm::start(FRIThread *fri)
{
  fri_ = fri;
  pthread_create(&thread_, 0, &ROSComm::run_s, (void *) this);
}


void* ROSComm::run()
{
  while(!open() && !exitRequested_)
    usleep(200000);

  running_ = true;  // roscore is up, we are running

  // some process data
  RobotData data;
  RobotStatus status;
  RobotCommand cmd;

  memset(&status, 0, sizeof(status));

  int seq=0;

  ros::Rate loop_rate(rate_);

  while(!exitRequested_) {

    fri_->setRunstop(runstop());

    cmd = fri_->cmd();
    data = fri_->data();
    status = fri_->status();

    if(seq != data.seq) {
      publishData(data, cmd); // publish results
    }
    publishStatus(status);  // publish status messages

    seq = data.seq;

    ros::spinOnce();
    loop_rate.sleep();
  }
  running_ = false;

  return 0;
}


void ROSComm::finish()
{
  exitRequested_=true;
  ros::requestShutdown();
  pthread_join(thread_, 0);
}
