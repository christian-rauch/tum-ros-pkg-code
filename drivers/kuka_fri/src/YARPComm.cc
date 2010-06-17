
#include <yarp/os/all.h>
#include <yarp/os/impl/Logger.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>

#include "YARPComm.hh"
#include "FRIComm.hh"

#include <signal.h>

#include <string>

using namespace yarp::os;

YARPComm::YARPComm(const char* prefix) :
  prefix_(prefix),
  running_(false), exitRequested_(false)
{
  // making checkNetwork() quiet
  yarp::os::impl::Logger::get().setVerbosity(-1);
}

bool YARPComm::open()
{
  if(!yarp::os::Network::checkNetwork())
    return false;

  // returning to normal verbosity
  yarp::os::impl::Logger::get().setVerbosity(0);

  std::string tprefix(prefix_);
  port_commands.open((tprefix+"/cmd").c_str());
  port_commanded.open((tprefix+"/cmded").c_str());
  port_position.open((tprefix+"/pos").c_str());
  port_torque.open((tprefix+"/torque").c_str());
  port_torque_tcp.open((tprefix+"/torque_tcp").c_str());
  port_kukacommand.open((tprefix+"/sent_to_rsi").c_str());

  port_stiffness.open((tprefix+"/stiffness").c_str());
  port_damping.open((tprefix+"/damping").c_str());
  port_add_torque.open((tprefix+"/add_torque").c_str());

  return true;
}


bool YARPComm::receiveBottle(BufferedPort<Bottle> &port, float* data, int n)
{
  if(port.getPendingReads() <= 0) {
    return false;
  } else {
    // read commands, emptying the queue
    while(port.getPendingReads() > 0) {
      Bottle *b = port.read();
      for(int i=0; i < n; i++)
        data[i] = b->get(i).asDouble();
    }
    return true;
  }
}

bool YARPComm::receiveCommand(RobotCommand* cmd)
{
  bool got_data = false;

  got_data |= receiveBottle(port_commands, cmd->command, 7);
  got_data |= receiveBottle(port_stiffness, cmd->stiffness, 7);
  got_data |= receiveBottle(port_damping, cmd->damping, 7);
  got_data |= receiveBottle(port_add_torque, cmd->addTorque, 7);
  
  return got_data;
}


void YARPComm::publishData(const RobotData &data, const RobotCommand &cmd_old)
{
  sendData(port_position, data.position, 7);
  sendData(port_commanded, data.commanded, 7);
  sendData(port_torque, data.torque, 7);
  sendData(port_torque_tcp, data.torqueTCP, 12);
  sendData(port_kukacommand, cmd_old.command, 7);
}


void YARPComm::sendData(BufferedPort<Bottle> &p, const float* data, int n)
{
  Bottle &b=p.prepare();
  b.clear();

  for(int i=0; i < n; i++)
    b.addDouble(data[i]);

  p.write();
}


void YARPComm::start(FRIThread *fri)
{
  fri_ = fri;
  pthread_create(&thread_, 0, &YARPComm::run_s, (void *) this);
}


void* YARPComm::run()
{
  // this may take > 10s if YARP is down.
  while(!open() && !exitRequested_)
    usleep(500000);

  running_ = true;  // YARP is up, we are running

  // some process data
  RobotData data;
  RobotCommand cmd;

  int seq = 0;

  while(!exitRequested_) {
    receiveCommand(&cmd);
    fri_->setCmd(cmd);

    data = fri_->data();
    if(seq != data.seq)
      publishData(data, cmd);  // publish arm state

    Time::delay(0.002);
  }
  running_ = false;

  return 0;
}


void YARPComm::finish()
{
  exitRequested_ = true;
  pthread_join(thread_, 0);
}
