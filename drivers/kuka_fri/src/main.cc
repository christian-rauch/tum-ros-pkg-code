#include "FRIComm.hh"

#include "FRICheck.hh"

#include "YARPComm.hh"
#include "ROSComm.hh"

#include <signal.h> // SIGINT signal handling

#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>

int should_exit=0;

//! Ctrl-C handler
void catchsignal(int signo)
{
  should_exit=1;
  printf("\n# caught Ctrl-C, exiting...\n");
}


// This is our setup at TUM.
bool friConfigure(const char* side, FRIThread *kuka, char* yarp_prefix)
{
  if(strcmp(side, "left") != 0 && strcmp(side, "right") != 0)
    return false;

  if(strcmp(side, "left") == 0) {
    strcpy(yarp_prefix, "/lwr/left/robot");
    kuka->configureNetwork(7001, "192.168.139.12", 7010);
    kuka->limitChecker()->setHandSide(FRICheck::LEFT);
  }

  if(strcmp(side, "right") == 0)
  {
    strcpy(yarp_prefix, "/lwr/right/robot");
    kuka->configureNetwork(7002, "192.168.139.10", 7010);
    kuka->limitChecker()->setHandSide(FRICheck::RIGHT);
  }

  return true;
}

int main(int argc, char** argv)
{
  // install Ctrl-C handler
  struct sigaction act;
  act.sa_handler = catchsignal;
  act.sa_flags = 0;
  if ((sigemptyset(&act.sa_mask) == -1) ||
      (sigaction(SIGINT, &act, NULL) == -1)) {
    printf("# Failed to set SIGINT to handle Ctrl-C, oh well\n");
  }

  char yarp_prefix[1024] = "/lwr";
  bool configured = false;

  FRIThread kuka;

  // configure according to command line option
  if(argc >= 2)
    configured = friConfigure(argv[1], &kuka, yarp_prefix);

  // create YARP Comm object
  YARPComm yarp(yarp_prefix);

  // create ROS Comm object
  ros::init(argc, argv, "lwr", ros::init_options::NoSigintHandler);
  ROSComm ros;
  
  // configure according to ROS settings
  configured |= ros.configure(&kuka);


  // must be configured!
  if(!configured)
  {
    printf("usage: fri [left|right]\n");
    return 0;
  }

  //printf("# connecting %d -> %s:%d\n", local_port, remote_ip, remote_port);
  // now start FRI communication
  if(!kuka.start()) {
    printf("# FRI part failed to start, exiting before doing more harm...\n");
    return -1;
  }

  
  yarp.start(&kuka);
  ros.start(&kuka);

  int seq=0;

  bool showOutput = isatty(1);

  while(!should_exit) {
    RobotData data = kuka.data();
    RobotCommand cmd = kuka.cmd();
    RobotStatus status = kuka.status();

    if(seq != data.seq && showOutput) {
      char buffer[4096];
      FRIComm::printStatus(buffer, data, cmd, status);
      printf("%s", buffer);
      seq = data.seq;
    }

    usleep(500000);
  }

  kuka.finish();
  ros.finish();
  yarp.finish();

  printf("# exiting program.\n");
}
