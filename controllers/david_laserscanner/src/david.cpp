/*
 * Copyright (c) 2009 Andreas Holzbach <holzbach -=- cs.tum.edu>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
*/

/**
@mainpage

@htmlinclude manifest.html

\author Andreas Holzbach

@b david_laserscanner connects to a David server which controls the 
    David laser light triangulation software (www.david-laserscanner.com) 
    running on a windows machine.
 **/

//ROS core
#include "ros/ros.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fstream>
#include <cstdlib>

//TCP/IP and Socket
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

//ROS message
#include <perception_srvs/David.h>

#define BUFFSIZE 32

using namespace std;
using namespace perception_srvs;

class DavidService
{

private:
  ros::NodeHandle node_handle_;
  ros::ServiceServer service_; 

  //Variables for IP Connection
  int sock; //<0 if connection fails
  //char* ip; //IP address of server (char*)
  struct sockaddr_in server;
  char buffer[BUFFSIZE];
  unsigned int msglen; //packet message length for TCP connection
  std::string david_ip;  //IP address of server (string)
  //unsigned int port; //Port number of server (unsigned int)
  std::string port_string; //Port number of server (string)
  std::string argument; //Argument to be executed by server


  bool debug_out;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  public:
    DavidService(const ros::NodeHandle& node_handle) : node_handle_(node_handle)
    {
      if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) 
      {
        ROS_WARN("Failed to create socket");
      }
    }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// Reads IP address and port from ROS parameter in david.launch
  void init()
  {	
    debug_out = true;	
    node_handle_.param("~ip",david_ip,string("127.0.0.1"));
    node_handle_.param("~port",port_string,string("19919"));
    service_ = node_handle_.advertiseService("david", &DavidService::david, this);	
    ROS_INFO("DAVID service has been initialized");	
  }
  
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// Send command for: start scanning with DAVID 
  void start()
  {
    char *test = "start\n";
    msglen = strlen(test);

    if (send(sock, test, msglen, 0) != msglen) 
    {
      ROS_INFO("Mismatch in number of sent bytes");
    }
      if (debug_out)
        ROS_INFO("DAVID method: start");
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// Send command for: stop scanning with DAVID 
  void stop()
  {
    char *test = "stop\n";
    msglen = strlen(test);

    if (send(sock, test, msglen, 0) != msglen) 
    {
      ROS_INFO("Mismatch in number of sent bytes");
    }
    if (debug_out)
      ROS_INFO("DAVID mehtod: stop");
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 /// Send command for: Grab Image for texture
  void grabTexture()
  {
    char *test = "grabTexture\n";
    msglen = strlen(test);

    if (send(sock, test, msglen, 0) != msglen) 
    {
      ROS_INFO("Mismatch in number of sent bytes");
    }
    if (debug_out)
      ROS_INFO("DAVID mehtod: grabTexture");
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// Send command for: Save Scan
  void save(std::string arg)
  {
    //std::string temp = "save";
    //temp.append(arg.str);
    //temp.append("\n");
    std::string temp = arg;
    temp.append("\n");
    const char *test = temp.c_str();
    msglen = strlen(test);
		
    //char *test = "save\n";
    //msglen = strlen(test);

    if (send(sock, test, msglen, 0) != msglen) 
    {
      ROS_INFO("Mismatch in number of sent bytes");
    }
    if (debug_out)
      ROS_INFO("DAVID mehtod: save");
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// Send command for: erase Scan
  void erase()
  {
    char *test = "erase\n";
    msglen = strlen(test);

    if (send(sock, test, msglen, 0) != msglen) 
    {
      ROS_INFO("Mismatch in number of sent bytes");
    }
    if (debug_out)
      ROS_INFO("DAVID mehtod: erase");
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// Send command for: erase Texture
  void eraseTexture()
  {
    char *test = "eraseTexture\n";
    msglen = strlen(test);

    if (send(sock, test, msglen, 0) != msglen) 
    {
      ROS_INFO("Mismatch in number of sent bytes");
    }
    if (debug_out)
      ROS_INFO("DAVID mehtod: eraseTexture");
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// Connect to TCP2COM DAVID Server
  void connect2David()
  {		
    ROS_INFO("DAVID: Connecting to DAVID Server...");		
    //getIpPortFromFile("davidIP.conf");
    //std::string ipInfo = "David IP ";
    //cout << david_ip << ":" << port;		
    //david_ip = "127.0.0.1";
    //port = 19919;
    msglen = strlen(david_ip.c_str());
    /* Construct the server sockaddr_in structure */
    memset(&server, 0, sizeof(server));       /* Clear struct */
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = inet_addr(david_ip.c_str());
    server.sin_port = htons(atoi(port_string.c_str()));
    //server.sin_port = htons(atoi("19919"));

    /* Establish connection */
    if (connect(sock,(struct sockaddr *) &server,
      sizeof(server)) < 0) 
    {
      ROS_INFO("Failed to connect with server");
    }

    ROS_INFO("DAVID: Connected to DAVID Server");
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// Disconnect from Socket
	void disconnect()
	{
		close(sock);
	}
	
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	~DavidService()
	{
		close(sock);
	}
	
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /* Function that gets called by other services. The request argument is the 
      method to be called on the David server
  */
	bool david(David::Request &request, David::Response &resp)
	{
		argument = request.david_method;
		if (argument.find("connect")!=-1)
			connect2David();
		if (argument.find("start") != -1)
			start();
		if (argument.find("stop") != -1)
			stop();
		if (argument.find("erase") != -1)
		{		
		    if (argument.find("eraseTexture") != -1)
		    {
			eraseTexture();
		    }
		    else
		    {
		    	erase();
		    }
		}
		if (argument.find("grabTexture") != -1)
			grabTexture();
		if (argument.find("save") != -1)
			save(argument);		
		return true;
	}

  /*
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void getIpPortFromFile(std::string fileName)
  {
    ifstream file;	
    file.open(fileName.c_str());
    if (!file)
    {
      ROS_INFO("Error getIpPortFromFile: no File");
    }
    std::string file_ip;
    std::string file_port;
    std::getline(file, file_ip);
    std::getline(file, file_port);
    david_ip = file_ip;
    port = atoi(file_port.c_str());		
  }
*/
};


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  int main(int argc, char **argv)
  {
    //ROS
    ros::init(argc, argv, "david");
    ros::NodeHandle n;	
    DavidService david(n);
		
    david.init(); //Connect to Server
    ros::spin();
    return 0;
  }

