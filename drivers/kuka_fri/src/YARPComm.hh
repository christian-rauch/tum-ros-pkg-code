#include "FRIData.hh"

#include <pthread.h>

#include <yarp/os/all.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>

class FRIThread;

class YARPComm {
public:
  YARPComm(const char* prefix);
  ~YARPComm();
  bool open();
  void publishData(const RobotData &data, const RobotCommand &cmd_old);
  bool receiveCommand(RobotCommand* cmd);

  // threading support
  void start(FRIThread *fri);
  void finish();
private:
  void sendData(yarp::os::BufferedPort<yarp::os::Bottle> &p, const float* data, int n);
  bool receiveBottle(yarp::os::BufferedPort<yarp::os::Bottle> &port, float* data, int n);
  bool receiveKRLCmd(yarp::os::BufferedPort<yarp::os::Bottle> &port, int* iData, float* rData);

  const char* prefix_;

  yarp::os::Network *net_;

  yarp::os::BufferedPort<yarp::os::Bottle> port_commands;
  yarp::os::BufferedPort<yarp::os::Bottle> port_commanded;
  yarp::os::BufferedPort<yarp::os::Bottle> port_position;
  yarp::os::BufferedPort<yarp::os::Bottle> port_torque;
  yarp::os::BufferedPort<yarp::os::Bottle> port_torque_tcp;
  yarp::os::BufferedPort<yarp::os::Bottle> port_kukacommand;

  yarp::os::BufferedPort<yarp::os::Bottle> port_stiffness;
  yarp::os::BufferedPort<yarp::os::Bottle> port_damping;
  yarp::os::BufferedPort<yarp::os::Bottle> port_add_torque;

  yarp::os::BufferedPort<yarp::os::Bottle> port_krlcommand;

  // threading support
  FRIThread* fri_;
  void* run();
  static void* run_s(void *ptr) { return ((YARPComm *) ptr)->run(); }

  pthread_t thread_;
  bool running_, exitRequested_;
};
