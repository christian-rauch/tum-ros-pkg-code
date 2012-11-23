
#ifndef FRICOMM_HH
#define FRICOMM_HH

#include <netinet/in.h>
#include <pthread.h>

#include <sys/time.h>

#include <friComm.h>  // KUKA proprietary header

#include <string>

#include "FRIData.hh"

class FRICheck;


/*!

FRIComm is a low-level communication class for the KUKA FRI robot
interface.  It provides functions for receiving data, get data, set
commands and send response packets. When the respond method is called,
a limiting is performed for velocity, acceleration and joint limits.
It is usually used like this:

\code
FRIComm fri(...);
fri.open();
while(running) { 
  fri.receive();
  data = fri.getData();
  ...
  fri.setCmd(...);
  fri.respond();
}
...
fri.close();  // does a read / respond loop until out of command mode.
\endcode

setCmd() converts the desired velocity to a position increment. As an
alternative, fri_cmd_ can be written directly. This position
increment is limited each time the respond() method is called.

FRIComm stores only two fri structs (and the last commanded position,
so repeated calls to setCmd() do not accumulate movements).

\note FRIComm does \e not remember the desired speed because it does
not know how the command was set: As a velocity using setCmd() or as a
position increment using fri_cmd_. The semantics of "remembering" is
unclear and potentially dangerous. The user of the class FRIComm must
set the desired speed / position increment in every cycle.

It is your responsibility to decelerate the arm before closing!

 */

class CartesianImpedanceControl;

class FRIComm
{
public:
  FRIComm();
  ~FRIComm();

  //! set network parameters
  void configureNetwork(int local_port, std::string remote_address, int remote_port);

  bool open();  //!< opens  the UDP connection
  void close(); //!< closes the UDP connection

  FRICheck* limitChecker() {return safety_;}

  RobotData data();
  RobotStatus status();

  RobotCommand cmd();
  void setCmd(RobotCommand cmd);


  void setRunstop(bool stopped);
  bool runstop();
  bool postCommand(int iData[16], float rData[16]);

  bool receive();  //!< receives a UDP measurement packet.
  void respond();  //!< sends the response packet.

  static const char* printStatus(char* buffer,
                                 RobotData data,
                                 RobotCommand cmd,
                                 RobotStatus status);

  tFriMsrData fri_msr_;
  tFriCmdData fri_cmd_;
private:
  static const float KRL_TIMEOUT=4.5;
  static const float KRL_GUARD_TIME = 0.8;
  float krl_time_;
  void processCommand(); //!< processes running command requests from postCommand() ( called from respond() )
  void stateHandler();   //!< watches the current robot state and posts commands if necessary

  FRICheck *safety_;
  FRICheck *safety2_;

  unsigned int seq_;
  bool closing_;
  bool runstop_;
  
  int local_port_, remote_port_;
  std::string remote_address_;
  int socket_;
  struct sockaddr_in remote_addr_;

  bool useCartesianImpedance_;
  CartesianImpedance cartesianImpedanceCmd_;

  CartesianImpedanceControl* cartesianImpedance_;

  // time measurement helpers
  struct timeval last_tick;
  long long duration_sum;
  int duration_num;
  int duration_max;
  int duration_mean_published;
  int duration_max_published;
  void tick();
  void tock();
};



////////////////////////////////////////////////////////////////////////////////////

class FRIThread
{
public:
  FRIThread();

  //! set network parameters
  void configureNetwork(int local_port, const char* remote_address, int remote_port);

  bool start();

  bool running();
  void finish();

  RobotData data();

  RobotCommand cmd();
  void setCmd(RobotCommand cmd);

  bool runstop();
  void setRunstop(bool stopped);

  RobotStatus status();

  FRICheck* limitChecker();

  void postCommand(int iData[16], float rData[16]);
private:
  FRIComm fri;

  void* run();
  static void* run_s(void *ptr) { return ((FRIThread *) ptr)->run(); }
  bool exitRequested_, running_;

  pthread_t thread_;
  pthread_mutex_t mutex_;

  class KRLCmd {
  public:
    int iData[16];
    float rData[16];
    bool fresh;
  };

  bool runstop_buffer_;
  RobotData data_buffer_;
  RobotCommand cmd_buffer_;
  RobotStatus status_buffer_;
  KRLCmd krlcmd_buffer_;
};

#endif // FRICOMM_HH
