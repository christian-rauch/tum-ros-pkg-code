#include <strings.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>

#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/udp.h>
#include <arpa/inet.h>

#include <time.h>
#include <sys/time.h>
#include <errno.h>

#include <math.h>

#include "friComm.h"


class FRIServer
{
public:
  int sock_;
  struct sockaddr_in recv_addr_, peer_addr_;
  socklen_t recv_addr_len_;


  tFriMsrData msr_msg_;
  tFriCmdData cmd_msg_;

  int counter_;

  void init_msr_packet(tFriMsrData &msr);
  void init_network(const char *peer_address, int peer_port,
                    const char *my_address,   int my_port);

  void communicate();
};

void FRIServer::init_msr_packet(tFriMsrData &msr)
{
  msr.head.packetSize = sizeof(tFriMsrData);
  msr.head.datagramId = FRI_DATAGRAM_ID_MSR;

  msr.intf.state = FRI_STATE_CMD;
  msr.intf.quality = FRI_QUALITY_PERFECT;
  msr.intf.desiredMsrSampleTime = 0.0;
  msr.intf.desiredCmdSampleTime = 0.0;
  msr.intf.safetyLimits = 0.0;

  msr.intf.stat.answerRate = 1.0;
  msr.intf.stat.latency = 0.0;
  msr.intf.stat.jitter = 0.0;
  msr.intf.stat.missRate = 0.0;
  msr.intf.stat.missCounter = 0;

  msr.robot.power = 0;
  msr.robot.control = 0;
  msr.robot.error = 0;
  msr.robot.warning = 0;

  for(int i=0; i < 7; i++)
  {
    msr.data.msrJntPos[i] = 0.0;
    msr.data.cmdJntPosFriOffset[i] = 0.0;
    msr.data.msrJntTrq[i] = 0.0;
    msr.data.estExtJntTrq[i] = 0.0;

    msr.data.cmdJntPos[i] = 0.0;
    msr.data.gravity[i] = 0.0;
    msr.robot.temperature[i] = 0;
  }

  counter_ = 0;
}

void FRIServer::init_network(const char *peer_address, int peer_port,
                             const char *my_address,   int my_port)
{
  sock_ = socket(AF_INET, SOCK_DGRAM, 0);

  bzero((char *) &peer_addr_, sizeof(peer_addr_));
  peer_addr_.sin_family = AF_INET;
  peer_addr_.sin_addr.s_addr = inet_addr(peer_address);
  peer_addr_.sin_port = htons(peer_port);

  struct sockaddr_in my_addr;
  bzero((char *) &my_addr, sizeof(my_addr));
  my_addr.sin_family = AF_INET;
  my_addr.sin_addr.s_addr = inet_addr(my_address);
  my_addr.sin_port = htons(my_port);

  bind(sock_, (sockaddr*) &my_addr, sizeof(sockaddr_in));

  bzero((char *) &recv_addr_, sizeof(recv_addr_));
  recv_addr_len_ = sizeof(recv_addr_);
}

void FRIServer::communicate()
{
  while(1)
  {
    sendto(sock_, (void*) &msr_msg_, sizeof(msr_msg_), 0,
           (sockaddr*) &peer_addr_, sizeof(peer_addr_));
    recvfrom(sock_, (tFriCmdData*) &cmd_msg_, sizeof(cmd_msg_), 0,
             (sockaddr*) &recv_addr_, &recv_addr_len_);

    msr_msg_.head.sendSeqCount = counter_;
    msr_msg_.head.reflSeqCount = counter_ - 1;

    for(int i=0; i < 7; i++)
    {
      //msr_data.data.msrJntPos[i] = sin(0.01*counter)*(i+1)*0.3;
      msr_msg_.data.msrJntPos[i] = cmd_msg_.cmd.jntPos[i];
      msr_msg_.data.cmdJntPos[i] = cmd_msg_.cmd.jntPos[i];
      msr_msg_.data.msrJntTrq[i] = 0.0;
      msr_msg_.data.estExtJntTrq[i] = 0.0;
      msr_msg_.intf.timestamp=double(time(NULL));
    }

    usleep(990);
    counter_++;
  }
}


int main(int argc, char** argv)
{
  const char *localhost="127.0.0.1";

  printf("server address: 127.0.0.1:7010\n");
  printf("peer   address: 127.0.0.1:7001\n");

  FRIServer server;
  server.init_network(localhost, 7001, localhost, 7010);
  server.init_msr_packet(server.msr_msg_);

  server.communicate();
}
