/*
 * Copyright (C) 2009 by Ulrich Friedrich Klank <klank@in.tum.de>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

 
#include "VM_InterfaceFcns.h"

using namespace Halcon;

bool g_calibTAB=false;

void SendPoseToProlog(Halcon::HTuple ObjectPose, Halcon::HTuple Object, Halcon::HTuple DomAxis, 
yarp::os::BufferedPort < yarp::os::Bottle > & to_prolog, std::string & ImgFName, yarp::os::Bottle &CopsOrigAns)
{
         // YARP: Write to /tracking/in - sending the bottle to COP
//    yarp::os::BufferedPort < yarp::os::Bottle > to_prolog;
 //   to_prolog.open("/middleman-prolog/out");
    //sleep(3);
    //yarp::os::Network::connect("/objintable/out", "/objintable/in");
    //sleep(1);
    yarp::os::Bottle & copbottle = to_prolog.prepare();
    copbottle.clear();

//Setup bottle to send
    copbottle.addString(Object[0].S());
    copbottle.addString(DomAxis[0].S());

    yarp::os::Bottle temp, eulerPoseBot;
		
		double quat[4];
		HTuple EulerPose, EulerHM;
        	EulerPose = ObjectPose;
		//rotate (ObjectPose[4].D(), ObjectPose[5].D(), ObjectPose[3].D(), quat);
		//rotate (D2R(heading), D2R(attitude), D2R(bank), quat);
		//for(int i=0;i < 4; i++)
			//{
				//ObjectPose[3+i]=quat[i];
                        //}
    EulerPose[6]=0;
    pose_to_hom_mat3d(EulerPose, &EulerHM);
    //cerr<<"Homogenous transformation matrix: "<<EulerHM<<endl<<endl;
    //send HM pose
    for (int i = 0; i < EulerHM.Num(); i++)     {
        //cerr << ObjectPose << " " << endl;
		  temp.addDouble(EulerHM[i].D());}
    copbottle.addList() = temp;
    
    //send Euler pose
    for (int i = 0; i < EulerPose.Num(); i++) {
        //cerr << ObjectPose << " " << endl;
      eulerPoseBot.addDouble(EulerPose[i].D());}
    copbottle.addList() = eulerPoseBot;
    copbottle.addString(ImgFName.c_str());
    //cerr<<"Euler pose: "<<endl<<EulerPose<<endl;
    copbottle.addList() = CopsOrigAns;
    printf("\nSending the following bottle (Pose in quaternions): \n%s\n",copbottle.toString().c_str());   
    to_prolog.write();
    usleep(1000);
}

yarp::os::Bottle talkToCop(yarp::os::Bottle * lo_query, char *Object, int NumOfObjects)
{
    // YARP: Write to /tracking/in - sending the bottle to COP
  yarp::os::BufferedPort < yarp::os::Bottle > middlein;
  middlein.open("/middleman-cop/in");
  yarp::os::Bottle received;

  yarp::os::BufferedPort < yarp::os::Bottle > middleman;
	yarp::os::Port LoRPC;
	LoRPC.open("/middleman-lo/rpc");
    yarp::os::Network::connect("/middleman-lo/rpc", "/lo/in");
	yarp::os::Bottle lo_answer;
	cout<<"LO 	query: "<<endl<<lo_query->toString().c_str();
	while (!LoRPC.write(*lo_query, lo_answer));

	cout<<"LO ANSWER: "<<endl<<lo_answer.toString().c_str();
	
    middleman.open("/middleman-cop/out");
    //sleep(3);
    yarp::os::Network::connect("/middleman-cop/out", "/tracking/in");
    sleep(1);
    yarp::os::Bottle & copbottle=middleman.prepare();
    copbottle.clear();

//Setup bottle to send

    copbottle.addString("/locating/out");
    yarp::os::Bottle temp;
    temp.addString(Object);
    //temp.addInt (4);
    copbottle.addList() = temp;	//ClassID
    copbottle.addInt(NumOfObjects);	//NumOfObjects
    copbottle.addInt(0);	// ActionType (0=Locate, 1=Track)    
    temp.clear();
        temp.addDouble(1.0);
        if (g_calibTAB)
        {temp.addInt(12);
        cerr<<"I KNOW IT'S A Calib TAB"<<endl;
        }
        else
	{temp.addInt(lo_answer.get(0).asInt());
	cerr<<"IT'S NOT A Calib TAB"<<endl;
	}
	
	copbottle.addList().addList() = temp;
    printf("Sending the following bottle: \n%s\n",
	   copbottle.toString().c_str());

    middleman.write();
    while (!yarp::os::Network::queryName("/locating/out").isValid())
      {
        //cerr<<"waiting for port to be valid"<<endl;
      }
      
    //cerr<<"waiting for port to be valid"<<endl;
      //usleep(50);
	cerr<<"Port valid."<<endl;
        yarp::os::Network::connect("/locating/out", "/middleman-cop/in");
  usleep(1000);
        cerr<<"Connected. Waiting for answer from COP."<<endl;
        if (g_calibTAB)
        {
               yarp::os::Bottle & copbottle_2=middleman.prepare();
               copbottle_2.clear();
                        
                        //Setup bottle to send
                        
                copbottle_2.addString("/locating/out");
                yarp::os::Bottle temp_2;
                temp_2.addString(Object);
                //temp.addInt (4);
                copbottle_2.addList() = temp_2; //ClassID
                copbottle_2.addInt(NumOfObjects);     //NumOfObjects
                copbottle_2.addInt(0);        // ActionType (0=Locate, 1=Track)
                temp.clear();
                temp_2.addDouble(1.0);
                if (g_calibTAB)
                {
                        temp.addInt(12);
                        cerr<<"I KNOW IT'S A Calib TAB"<<endl;
                }                                                           
                else
                {
                        temp.addInt(lo_answer.get(0).asInt());
                        cerr<<"IT'S NOT A Calib TAB"<<endl;
                }
                middleman.write();
        }
                  
        received = *middlein.read(true);
        printf("Received the following bottle: \n%s\n",received.toString().c_str());
//    printf("Received the following bottle (2): \n%s\n",received2.toString().c_str());
	yarp::os::Network::disconnect("/locating/out", "/middleman-cop/in");
	middlein.close();
  middleman.close();
  cerr << endl << "Middleman's port Closed"<<endl;
    return received;
}

std::vector<RPLCommand> GetCommand(yarp::os::BufferedPort <yarp::os::Bottle> & RPL_in)
	{	
	std::vector<RPLCommand> vector_commands;
	RPLCommand comm("empty", 0, (HTuple)0, (HTuple)0);	
//	RPL_in.open("/middleman-prolog/in");
	
    /*while (!yarp::os::Network::queryName("/cop-rpl/out").isValid())
	{usleep(10);}*/
//	cerr<<"/cop-rpl/out valid. Connecting."<<endl;
//    yarp::os::Network::connect( "/cop-rpl/out","/middleman-prolog/in");
//	cerr<<"Connected."<<endl;
	cerr<<"Waiting for bottle from /cop-rpl/out"<<endl;
        yarp::os::Bottle command = *RPL_in.read(true);
        
        cerr<<"Bottle read." <<endl;
        
        for(int k=0; k < command.size();k+=4){
                if (!command.get(k).isString())
                {
                        cerr<< "I got something WEIRD. First bottle member must be a string."<<endl;
                        printf("Received the following command bottle: \n%s\n",command.toString().c_str());
                }
                cerr<<k<<endl;
                printf("Received the following command bottle: \n%s\n",command.toString().c_str());
                comm.ObjID = command.get(k).asString();
		cerr<<"First: "<<comm.ObjID<<endl;
		comm.NumObjects = command.get(k+1).asInt();
		cerr<<"Second: "<<comm.NumObjects<<endl;
		for (int i=0; i<command.get(k+2).asList()->size();i++)
		{
			comm.Pose[i] = command.get(k+2).asList()->get(i).asDouble();
		}
		cerr<<"Third: "<<comm.Pose<<endl;
		for (int i=0; i<command.get(k+3).asList()->size();i++)			
		{
			comm.Reg[i]= command.get(k+3).asList()->get(i).asDouble();
		}	
		cerr<<"Fourth: "<<comm.Reg<<endl;
		vector_commands.push_back(comm);
	}
	
	command.clear();
	return vector_commands;
	}
