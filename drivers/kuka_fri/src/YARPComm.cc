
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
  prefix_(prefix), net_(0),
  running_(false), exitRequested_(false)
{
  // making checkNetwork() quiet
  yarp::os::impl::Logger::get().setVerbosity(-1);
}


YARPComm::~YARPComm()
{
  if(net_) {
    net_->fini();
    delete net_;
  }
}


bool YARPComm::open()
{
  if(!yarp::os::Network::checkNetwork())
    return false;

  // returning to normal verbosity
  yarp::os::impl::Logger::get().setVerbosity(0);

  if(net_ == 0) {
    net_ = new Network();
    net_->init();
  }

  std::string tprefix(prefix_);
  port_commands.open((tprefix+"/cmd").c_str());
  port_commanded.open((tprefix+"/cmded").c_str());
  port_position.open((tprefix+"/pos").c_str());
  port_torque.open((tprefix+"/torque").c_str());
  port_torque_raw.open((tprefix+"/torque_raw").c_str());
  port_torque_tcp.open((tprefix+"/torque_tcp").c_str());
  port_kukacommand.open((tprefix+"/sent_to_rsi").c_str());

  port_stiffness.open((tprefix+"/stiffness").c_str());
  port_damping.open((tprefix+"/damping").c_str());
  port_add_torque.open((tprefix+"/add_torque").c_str());

  port_krlcommand.open((tprefix+"/krl_command").c_str());

  port_cart_stiffness.open((tprefix+"/cart_stiffness").c_str());
  port_cart_damping.open((tprefix+"/cart_damping").c_str());
  port_cart_force_torque.open((tprefix+"/cart_force_torque").c_str());
  port_cart_activate.open((tprefix+"/cart_activate").c_str());

  port_current_cart_stiffness.open((tprefix+"/current_cart_stiffness").c_str());
  port_current_cart_damping.open((tprefix+"/current_cart_damping").c_str());
  port_current_cart_force_torque.open((tprefix+"/current_cart_force_torque").c_str());

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


bool YARPComm::receiveKRLCmd(BufferedPort<Bottle> &port, int* iData, float* rData)
{
  if(port.getPendingReads() <= 0) {
    return false;
  } else {
    // read command
    Bottle *b = port.read();
    int i=0, n_i=0, n_f=0;
    for(; i < b->size() && b->get(i).isInt() && i < 16; i++)
      iData[i] = b->get(i).asInt();
    n_i=i;
    
    for(; i < b->size() && b->get(i).isDouble() && i < 32; i++)
      rData[i-n_i] = b->get(i).asDouble();
    n_f=i-n_i;

    printf("YARP: got command [");
    if(n_i > 0) printf("%d", iData[0]);
    for(i=1; i < n_i; i++) printf(", %d", iData[i]);
    printf("] [");
    if(n_f > 0) printf("%2.2f", rData[0]);
    for(i=1; i < n_f; i++) printf(", %2.2f", rData[i]);
    printf("]\n");
  }
  return true;
}


bool YARPComm::receiveCommand(RobotCommand* cmd)
{
  bool got_data = false;

  got_data |= receiveBottle(port_commands, cmd->command, 7);
  got_data |= receiveBottle(port_stiffness, cmd->stiffness, 7);
  got_data |= receiveBottle(port_damping, cmd->damping, 7);
  got_data |= receiveBottle(port_add_torque, cmd->addTorque, 7);

  got_data |= receiveBottle(port_cart_stiffness, cmd->cartImpedance.K, 6*6);
  got_data |= receiveBottle(port_cart_damping, cmd->cartImpedance.D, 6*6);
  got_data |= receiveBottle(port_cart_force_torque, cmd->cartImpedance.ft, 6);

  float cart_activate;
  if(receiveBottle(port_cart_activate, &cart_activate, 1))
  {
    cmd->useCartesianImpedance = (cart_activate > 0.5);
    got_data = true;
  }
 
  return got_data;
}


void YARPComm::publishData(const RobotData &data, const RobotCommand &cmd_old)
{
  sendData(port_position, data.position, 7);
  sendData(port_commanded, data.commanded, 7);
  sendData(port_torque, data.torque, 7);
  sendData(port_torque_raw, data.torque_raw, 7);
  sendData(port_torque_tcp, data.torqueTCP, 12);
  sendData(port_kukacommand, cmd_old.command, 7);

  sendData(port_current_cart_stiffness, cmd_old.cartImpedance.K, 6*6);
  sendData(port_current_cart_damping,   cmd_old.cartImpedance.D, 6*6);
  sendData(port_current_cart_force_torque,   cmd_old.cartImpedance.ft, 6);
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

  int iData[16];
  float rData[16];

  int seq = 0;

  while(!exitRequested_) {
    cmd = fri_->cmd();
    if(receiveCommand(&cmd))
      fri_->setCmd(cmd);

    if(receiveKRLCmd(port_krlcommand, iData, rData))
      fri_->postCommand(iData, rData);

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
