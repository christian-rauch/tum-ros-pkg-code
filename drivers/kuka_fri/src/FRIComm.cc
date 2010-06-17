
#include <unistd.h>
#include <stdio.h>
#include <math.h>

#include <string.h>

#include <pthread.h>

// network
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>

#include "FRIComm.hh"
#include "FRICheck.hh"

FILE *LOG=0;

#define DEG *M_PI/180.0
#define RAD /M_PI*180.0

// threading helper class
class pthread_scoped_lock
{
public:
  pthread_scoped_lock(pthread_mutex_t *mutex) : mutex_(mutex) { pthread_mutex_lock(mutex_); }
  ~pthread_scoped_lock() { unlock(); }
  void unlock() { pthread_mutex_unlock(mutex_); }
private:
  pthread_mutex_t *mutex_;
};


RobotData::RobotData() :
  seq(0)
{
  for(int i=0; i < 7; i++) {
    commanded[i] = 0.0;
    position[i] = 0.0;
    torque[i] = 0.0;
  }
  for(int i=0; i < 6; i++) {
    torqueTCP[i] = 0.0;
  }
}


RobotCommand::RobotCommand()
{
  for(int i=0; i < 7; i++) {
    command[i] = 0.0;
    stiffness[i] = DEFAULT_STIFFNESS;
    damping[i] = DEFAULT_DAMPING;
    addTorque[i] = 0.0;
  }
}


void RobotCommand::incrementPosition(float* newPos, float* currentPos, float rate)
{
  for(int i=0; i < 7; i++)
    newPos[i] = currentPos[i] + command[i]*rate;
}


RobotStatus::RobotStatus()
{
  memset(this, 0, sizeof(this));
}


FRIComm::FRIComm() :
  krl_time_(-1), seq_(0), closing_(false), runstop_(false), socket_(0)
{
  local_port_ = 0;
  remote_address_ = "";
  remote_port_ = 0;

  safety_ = new FRICheck();

  // clear data buffers
  memset(&fri_msr_, 0, sizeof(fri_msr_));
  memset(&fri_cmd_, 0, sizeof(fri_cmd_));

  fri_cmd_.head.datagramId = FRI_DATAGRAM_ID_CMD;
  fri_cmd_.head.packetSize = sizeof(tFriCmdData);

  // full control for joint impedance controller
  fri_cmd_.cmd.cmdFlags = FRI_CMD_JNTPOS | FRI_CMD_JNTSTIFF | FRI_CMD_JNTDAMP | FRI_CMD_JNTTRQ;
}

void FRIComm::configureNetwork(int local_port, const char* remote_address, int remote_port)
{
  local_port_ = local_port;
  remote_address_ = remote_address;
  remote_port_ = remote_port;
}

FRIComm::~FRIComm()
{
  if(safety_)
    delete safety_;
}


bool FRIComm::open()
{
  socket_ = socket(AF_INET, SOCK_DGRAM, 0);
  setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, 0, 0);

  // time out after there was no data for 0.1 seconds.
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 100000;
  setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  bzero((char *) &remote_addr_, sizeof(remote_addr_));
  remote_addr_.sin_family = AF_INET;
  remote_addr_.sin_addr.s_addr = inet_addr(remote_address_);
  remote_addr_.sin_port = htons(remote_port_);

  struct sockaddr_in local_addr;
  bzero((char *) &local_addr, sizeof(local_addr));
  local_addr.sin_family = AF_INET;
  local_addr.sin_addr.s_addr = INADDR_ANY;
  local_addr.sin_port = htons(local_port_);

  int res = bind(socket_, (sockaddr*) &local_addr, sizeof(sockaddr_in));

  if(res == -1) {
    if(errno == EADDRINUSE) {
      printf("# ERROR: Port %d is already taken. Is there another client running?\n", local_port_);
    } else {
      printf("# ERROR: failed to bind the UDP socket to 0.0.0.0:%d\n", local_port_);
    }
    return false;
  }
  return true; 
}


/*! When the connection is in command mode, it switches
 *  to monitor mode and keeps the read() / respond() loop running until that switch
 *  has happened.
 *
 *  It is your resonsibility to decelerate the arm before closing!
 */
void FRIComm::close()
{
  int   intData[16]  = {2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  float realData[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


  if(fri_msr_.intf.state == FRI_STATE_CMD) {

    float last_pos[7] = {9,9,9,9,9,9,9};
    bool stopped=false;
    while(!stopped) {
      if(!receive()) // read may time out ...
        break;       // ... command mode was definitely left.

      // command zero velocity
      respond();

      // check if commanded position has changed due to safety (vel, acc)
      stopped=true;
      for(int i=0; i < 7; i++)
        stopped &= (fri_cmd_.cmd.jntPos[i] == last_pos[i]);

      // remember last position
      for(int i=0; i < 7; i++)
        last_pos[i] = fri_cmd_.cmd.jntPos[i];
    }

    closing_=true;

    postCommand(intData, realData);
    while(fri_msr_.intf.state == FRI_STATE_CMD 
      || (fri_cmd_.krl.boolData & 1) ) {

      if(!receive()) // read may time out ...
        break;       // ... command mode was definitely left.

      // command has timed out without effect
      if(krl_time_ == -1)
        postCommand(intData, realData);

      respond(); // will mirror current pos since closing_ is set
    }

    // keep up communication for a while...
    for(int i=0; i < 100; i++) {
      if(!receive()) // read may time out ...
        break;       // ... command mode was definitely left.
      respond();
    }
  }
  // bye, bye!
  ::close(socket_);

  closing_=false;
}


/*! Waits up to 0.1s for new data.
 * \return true if a valid packet was received
 */
bool FRIComm::receive()
{
  struct sockaddr addr;
  socklen_t addr_len = sizeof(addr);

  int n = recvfrom(socket_, (void*) &fri_msr_, sizeof(fri_msr_), 0, &addr, &addr_len);

  if(n != sizeof(tFriMsrData)) {
    if(errno == EAGAIN)
      return false; // no packet received. return silently

    printf("# (%d) ERROR: %s\n", errno, strerror(errno));
    printf("# ERROR: bad packet length %d (expected: %d)\n", n, (int) sizeof(tFriMsrData));
    return false;
  }

  if(seq_ == 0)
    safety_->setPos(fri_msr_.data.cmdJntPos);

  // prepare response packet
  fri_cmd_.head.sendSeqCount = ++seq_;
  fri_cmd_.head.reflSeqCount = fri_msr_.head.sendSeqCount;

  return true;
}


/*! The position increment is limited before sending so as to not exceed velocity, acceleration and joint angle limits.
 *  This commanded position is rememered and used for subsequent calls to setCmd().
 */
void FRIComm::respond()
{
  if(runstop_) {
    float* pos = safety_->pos();
    for(int i=0; i < 7; i++) {
      fri_cmd_.cmd.jntPos[i] = pos[i];
    }
  }

  // monitor mode settings
  if(fri_msr_.intf.state != FRI_STATE_CMD || fri_msr_.robot.power == 0) {
    for(int i=0; i < 7; i++) {
      fri_cmd_.cmd.jntPos[i] = fri_msr_.data.cmdJntPos[i]+fri_msr_.data.cmdJntPosFriOffset[i];
      fri_cmd_.cmd.addJntTrq[i] = 0.0;
    }
  }

  // TODO: filter stiffness, damping and torque
  safety_->adjust(fri_cmd_.cmd.jntPos, fri_msr_.intf.desiredCmdSampleTime, fri_msr_.intf.safetyLimits);

  stateHandler();
  processCommand();

  // monitor mode settings
  if(fri_msr_.intf.state != FRI_STATE_CMD || closing_) {
    for(int i=0; i < 7; i++) {
      fri_cmd_.cmd.jntPos[i] = fri_msr_.data.cmdJntPos[i]+fri_msr_.data.cmdJntPosFriOffset[i];
      fri_cmd_.cmd.addJntTrq[i] = 0.0;
    }
  }


  // DEBUGGING
  float max_offs=0.0;
  for(int i=0; i < 7; i++) {
    if(fabs(fri_msr_.data.cmdJntPosFriOffset[i]) > max_offs)
      max_offs = fabs(fri_msr_.data.cmdJntPosFriOffset[i]);
  }

  fprintf(LOG, "%d (%f): kt=%f f=l%xr%x cmd=%d pwr=%d q=%d a=",
    fri_msr_.head.sendSeqCount, fri_msr_.intf.timestamp,
    krl_time_, fri_cmd_.krl.boolData, fri_msr_.krl.boolData, fri_cmd_.krl.intData[0], fri_msr_.robot.power, fri_msr_.intf.quality);
  for(int i=0; i < 7; i++)
    fprintf(LOG,"[%d]%5.5f+%5.5f!%5.5f ", i, fri_msr_.data.cmdJntPos[i],
                                             fri_msr_.data.cmdJntPosFriOffset[i],
                                             fri_cmd_.cmd.jntPos[i]);
  fprintf(LOG, "\n");
  // DEBUGGING END

  // send packet
  sendto(socket_, (void*) &fri_cmd_, sizeof(fri_cmd_), 0, (sockaddr*) &remote_addr_, sizeof(remote_addr_));
}


bool FRIComm::postCommand(int iData[16], float rData[16])
{
  fprintf(LOG, "%d (%f): posted command %d\n",
    fri_msr_.head.sendSeqCount, fri_msr_.intf.timestamp, iData[0]);

  if(krl_time_ > fri_msr_.intf.timestamp)
    return false;  // command is being processed

  for(int i=0; i < 16; i++) {
    fri_cmd_.krl.intData[i] = iData[i];
    fri_cmd_.krl.realData[i] = rData[i];
  }
  fri_cmd_.krl.boolData = 1;  // raise flag 0
  krl_time_ = fri_msr_.intf.timestamp + KRL_TIMEOUT;

  return true;
}


void FRIComm::processCommand()
{
  if(fri_msr_.krl.boolData & 1) {
    // processing command, push end time
    fri_cmd_.krl.boolData &= 0xfffe;
    krl_time_ = fri_msr_.intf.timestamp + KRL_GUARD_TIME;
  }

  if(krl_time_ != -1 && krl_time_ < fri_msr_.intf.timestamp) {
    printf("**krl_time: %f < %f\n", krl_time_, fri_msr_.intf.timestamp);
    if((fri_cmd_.krl.boolData & 1)) {
      // timeout, command flag still set
      fri_cmd_.krl.boolData &= 0xfffe;
      krl_time_ = fri_msr_.intf.timestamp + KRL_GUARD_TIME;
    } else {
      printf("command timeout\n");
      // timeout, clear command
      for(int i=0; i < 16; i++) {
        fri_cmd_.krl.intData[i] = 0;
        fri_cmd_.krl.realData[i] = 0.0;
      }
      krl_time_ = -1;
    }    
  }
}

/*! Called from respond.
 */
void FRIComm::stateHandler()
{
  int   intData[16]  = {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  float realData[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

  // check if we should switch to command mode...
  if(fri_msr_.intf.state == 1 && fri_msr_.intf.quality == 3
      && krl_time_ == -1 && fri_msr_.robot.control == 3 && !closing_)   {
    printf("### switching to command mode... %f\n", krl_time_);
    postCommand(intData, realData);
  }

  // reflecting command / monitor state to KRL
  fri_cmd_.krl.boolData &= 0x7f;
  fri_cmd_.krl.boolData |= (fri_msr_.intf.state == 2) << 7;
}


const char* FRIComm::printStatus(char* buffer,
                                 RobotData data,
                                 RobotCommand cmd,
                                 RobotStatus status)
{
  int off=0;
  char qualities[] = {'x', '-', '+', '*'}; char comm_states[] = {'0', 'M', 'C'};
  char quality = qualities[status.quality], state = comm_states[status.state];

  float *p=data.position, *cd=data.commanded, *c=cmd.command;

  //off+=sprintf(buffer+off, "power=%x control=%x error=%x warning=%x\n",fri_msr_.robot.power, fri_msr_.robot.control, fri_msr_.robot.error, fri_msr_.robot.warning);

  off+=sprintf(buffer+off, "t=%7.1f    pos=(% 7.2f % 7.2f % 7.2f % 7.2f % 7.2f % 7.2f % 7.2f)\n",
         status.time, p[0]/M_PI*180.0, p[1] RAD, p[2] RAD,
         p[3] RAD, p[4] RAD, p[5] RAD, p[6] RAD);
  off+=sprintf(buffer+off, "l=%1.5f  cmded=(% 7.2f % 7.2f % 7.2f % 7.2f % 7.2f % 7.2f % 7.2f)\n",
         status.latency, cd[0] RAD, cd[1] RAD, cd[2] RAD,
         cd[3] RAD, cd[4] RAD, cd[5] RAD, cd[6] RAD);
  off+=sprintf(buffer+off, "%c%c           cmd=(% 7.2f % 7.2f % 7.2f % 7.2f % 7.2f % 7.2f % 7.2f)\n\n",
         state, quality, c[0] RAD, c[1] RAD, c[2] RAD, c[3] RAD, c[4] RAD, c[5] RAD, c[6] RAD);

  return buffer;
}


bool FRIComm::runstop()
{
  return runstop_;
}


void FRIComm::setRunstop(bool stopped)
{
  runstop_ = stopped;
}


RobotData FRIComm::data()
{
  RobotData data;

  data.seq = fri_msr_.head.sendSeqCount;

  for(int i=0; i < 7; i++) {
    data.position[i] = fri_msr_.data.msrJntPos[i];
    //data.commanded[i] = fri_msr_.data.cmdJntPos[i] + fri_msr_.data.cmdJntPosFriOffset[i];
    data.commanded[i] = fri_cmd_.cmd.jntPos[i];
    data.torque[i] = fri_msr_.data.estExtJntTrq[i];
  }

  for(int i=0; i < 6; i++) {
    data.torqueTCP[i] = fri_msr_.data.estExtTcpFT[i];
  }

  return data;
}


RobotCommand FRIComm::cmd()
{
  RobotCommand c;
  float *pos = safety_->pos();

  for(int i=0; i < 7; i++) {
    c.command[i]   = (fri_cmd_.cmd.jntPos[i] - pos[i]) / fri_msr_.intf.desiredCmdSampleTime;
    c.stiffness[i] = fri_cmd_.cmd.jntStiffness[i];
    c.damping[i]   = fri_cmd_.cmd.jntDamping[i];
    c.addTorque[i] = fri_cmd_.cmd.addJntTrq[i];
  }
  return c;
}


void FRIComm::setCmd(RobotCommand cmd)
{
  cmd.incrementPosition(fri_cmd_.cmd.jntPos, safety_->pos(), fri_msr_.intf.desiredCmdSampleTime);

  for(int i=0; i < 7; i++) {
      fri_cmd_.cmd.jntStiffness[i] = cmd.stiffness[i];
      fri_cmd_.cmd.jntDamping[i] = cmd.damping[i];
      fri_cmd_.cmd.addJntTrq[i] = cmd.addTorque[i];
  }
}


RobotStatus FRIComm::status()
{
  RobotStatus s;
  s.time = fri_msr_.intf.timestamp;
  s.answerRate = fri_msr_.intf.stat.answerRate;
  s.latency = fri_msr_.intf.stat.latency;
  s.jitter = fri_msr_.intf.stat.jitter;
  s.missRate = fri_msr_.intf.stat.missRate;
  s.missCounter = fri_msr_.intf.stat.missCounter;

  s.state = fri_msr_.intf.state;
  s.quality = fri_msr_.intf.quality;
  s.msrSampleTime = fri_msr_.intf.desiredMsrSampleTime;
  s.cmdSampleTime = fri_msr_.intf.desiredCmdSampleTime;
  s.safety = fri_msr_.intf.safetyLimits;

  s.power = fri_msr_.robot.power;
  s.controlMode = fri_msr_.robot.control;
  s.error = fri_msr_.robot.error;
  s.warning = fri_msr_.robot.warning;
  for(int i=0; i < 7; i++)
    s.temperature[i] = fri_msr_.robot.temperature[i];

  s.runstop = runstop_;

  return s;
}



FRIThread::FRIThread() :
  exitRequested_(false), runstop_buffer_(false)
{

}


void FRIThread::configureNetwork(int local_port, const char* remote_address, int remote_port)
{
  fri.configureNetwork(local_port, remote_address, remote_port);

  //DEBUGGING
  if(!LOG && local_port==7001)
    LOG=fopen("movelog_left.log", "w+");
  if(!LOG && local_port==7002)
    LOG=fopen("movelog_right.log", "w+");
}


FRICheck* FRIThread::limitChecker()
{
  return fri.limitChecker();
}

bool FRIThread::start()
{
  // fri.open() does not block - we can do it here already.
  if(!fri.open())
    return false;

  // setting up mutex
  pthread_mutexattr_t mattr;
  pthread_mutexattr_init(&mattr);
  pthread_mutexattr_setprotocol(&mattr, PTHREAD_PRIO_INHERIT);

  pthread_mutex_init(&mutex_,  &mattr);

  // setting up thread
  pthread_attr_t tattr;
  struct sched_param sparam;
  sparam.sched_priority = 12;
  pthread_attr_init(&tattr);
  pthread_attr_setschedpolicy(&tattr, SCHED_FIFO);
  pthread_attr_setschedparam(&tattr, &sparam);
  pthread_attr_setinheritsched (&tattr, PTHREAD_EXPLICIT_SCHED);

  if(pthread_create(&thread_, &tattr, &FRIThread::run_s, (void *) this) != 0) {
    printf("# ERROR: could not create realtime thread\n");
    return false;
  }

  running_ = true;
  return true;
}

bool FRIThread::runstop()
{
  pthread_scoped_lock lock(&mutex_);
  return runstop_buffer_;
}

void FRIThread::setRunstop(bool stopped)
{
  pthread_scoped_lock lock(&mutex_);
  runstop_buffer_ = stopped;
}


void* FRIThread::run()
{
  bool runstop = false;
  RobotCommand cmd;
  RobotData data;
  RobotStatus status = fri.status();

  printf("# waiting for LWR connection\n");

  bool communicating = false;

  while(!exitRequested_) {
    // retrieve new commands from buffer
    //TODO: -> trylock
    pthread_mutex_lock(&mutex_);
    runstop = runstop_buffer_;
    cmd = cmd_buffer_;
    pthread_mutex_unlock(&mutex_);

    if(!fri.receive()) { // blocks for up to 0.1s
      communicating = false;
      continue;
    } else {
      communicating = true;
    }

    data = fri.data();
    status = fri.status();
    fri.setRunstop(runstop);
    fri.setCmd(cmd);
    fri.respond();
      
    // note: the data is not stored in a queue.
    // read fast, or you will lose data...
    pthread_mutex_lock(&mutex_);
    data_buffer_ = data;
    status_buffer_ = status;
    pthread_mutex_unlock(&mutex_);
  }

  printf("# exiting loop\n");
  fri.close();
  printf("# communication finished\n");

  running_ = false;
  return 0;
}


RobotData FRIThread::data()
{
  pthread_scoped_lock lock(&mutex_);
  return data_buffer_;
}

bool FRIThread::running()
{
  pthread_scoped_lock lock(&mutex_);
  return running_;
}

void FRIThread::finish()
{
  pthread_mutex_lock(&mutex_);
  exitRequested_ = true;
  pthread_mutex_unlock(&mutex_);

  pthread_join(thread_, 0);
}

RobotCommand FRIThread::cmd()
{
  pthread_scoped_lock lock(&mutex_);
  return cmd_buffer_;
}

RobotStatus FRIThread::status()
{
  pthread_scoped_lock lock(&mutex_);
  return status_buffer_;
}


void FRIThread::setCmd(RobotCommand cmd)
{
  pthread_scoped_lock lock(&mutex_);
  cmd_buffer_ = cmd;
}
