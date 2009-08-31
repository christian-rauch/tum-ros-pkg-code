#include "ros/ros.h"

#include <iostream>
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <netinet/in.h>
#include <fstream>
#include <cstdlib>

#include <perception_srvs/David.h>

#define BUFFSIZE 32

using namespace std;
using namespace perception_srvs;

class David_service{

private:
	ros::NodeHandle node_handle_;
	ros::ServiceServer service_;
	int sock;
	char* ip;
	struct sockaddr_in server;
	char buffer[BUFFSIZE];
	unsigned int msglen;
	std::string david_ip;
	unsigned int port;
	std::string port_string;
	std::string argument;

public:
	David_service(const ros::NodeHandle& node_handle) : node_handle_(node_handle)
	{
		if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
					 ROS_INFO("Failed to create socket");
				}
	}

	///////////////////////////////////////////////////////////////////////////////////////
	void init()
	{	
	  node_handle_.param("~ip",david_ip,string("127.0.0.1"));
	  node_handle_.param("~port",port_string,string("19919"));
        	service_ = node_handle_.advertiseService("david", &David_service::david, this);	
		ROS_INFO("DAVID service has been initialized");	
	}		
	///////////////////////////////////////////////////////////////////////////////////////
	// start scanning with DAVID 
	void start()
	{
		char *test = "start\n";
		msglen = strlen(test);

		if (send(sock, test, msglen, 0) != msglen) 
		{
		  ROS_INFO("Mismatch in number of sent bytes");
		}
	}
	///////////////////////////////////////////////////////////////////////////////////////
	// stop scanning with DAVID 
	void stop()
	{
		char *test = "stop\n";
		msglen = strlen(test);

		if (send(sock, test, msglen, 0) != msglen) {
		  ROS_INFO("Mismatch in number of sent bytes");
		}
	}
	///////////////////////////////////////////////////////////////////////////////////////
	// grab Image for texture
	void grabTexture()
	{
		char *test = "grabTexture\n";
		msglen = strlen(test);

		if (send(sock, test, msglen, 0) != msglen) {
		  ROS_INFO("Mismatch in number of sent bytes");
		}
	}
	///////////////////////////////////////////////////////////////////////////////////////
	// save Scan
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

		if (send(sock, test, msglen, 0) != msglen) {
		  ROS_INFO("Mismatch in number of sent bytes");
		}
	}
	///////////////////////////////////////////////////////////////////////////////////////
	// erase Scan
	void erase()
	{
		char *test = "erase\n";
		msglen = strlen(test);

		if (send(sock, test, msglen, 0) != msglen) {
		  ROS_INFO("Mismatch in number of sent bytes");
		}
	}
	///////////////////////////////////////////////////////////////////////////////////////
	// erase Texture
	void eraseTexture()
	{
		char *test = "eraseTexture\n";
		msglen = strlen(test);

		if (send(sock, test, msglen, 0) != msglen) {
		  ROS_INFO("Mismatch in number of sent bytes");
		}
	}
	///////////////////////////////////////////////////////////////////////////////////////
	// Connect to TCP2COM DAVID Server
	void connect2David()
	{		
		ROS_INFO("Connecting to DAVID...");		
		//getIpPortFromFile("davidIP.conf");
		std::string ipInfo = "David IP ";
		cout << david_ip << ":" << port;		
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
		if (connect(sock,
					(struct sockaddr *) &server,
					sizeof(server)) < 0) {
		  ROS_INFO("Failed to connect with server");
		}
	}
	///////////////////////////////////////////////////////////////////////////////////////
	void disconnect()
	{
		close(sock);
	}
	
	///////////////////////////////////////////////////////////////////////////////////////
	~David_service()
	{
		close(sock);
	}
	
	///////////////////////////////////////////////////////////////////////////////////////
	//advertised function
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

};




int main(int argc, char **argv)
{
	//ROS
	ros::init(argc, argv, "david");
	ros::NodeHandle n;
	
	David_service david(n);		
	david.init();
	ros::spin();
	return 0;
}

