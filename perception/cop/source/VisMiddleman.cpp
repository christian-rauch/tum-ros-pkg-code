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

 
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include "VM_InterfaceFcns.h"
#include "VM_RPLCommand.h"
#include "VM_BasicYarpFcns.h"
#include "YarpRpcComm.h"
#include "VM_InTableCS.h"


Comm* RelPoseFactory::s_loService = NULL;
volatile bool g_stopall = false;

int main()
{
    std::string ObjectToLookFor;

		RelPoseFactory::s_loService = new YarpRpcComm("/middleman-listen/lo");
		yarp::os::BufferedPort < yarp::os::Bottle > command_from_RPL;
		yarp::os::BufferedPort < yarp::os::Bottle > to_prolog_main;

to_prolog_main.open("/middleman-prolog/out");

yarp::os::BufferedPort <yarp::os::Bottle> RPL_in_main;
RPL_in_main.open("/middleman-prolog/in");
HTuple empty6tuple(6,0.0);
	std::vector<RPLCommand > RPLcommand_vector;
	RPLCommand RPL_command((string)"Empty",0,(HTuple)0,(HTuple)0);	
	RPLCommand AddCalTab((string)"CalTab",1,empty6tuple,empty6tuple);

yarp::os::Network n;
//	n.init();
//-0.100142,0.057564,1.166830 /234.282112,5.349121,303.859858

HTuple CaltabPose(7,0.0), TableCSPose(7,0.0);
bool have_caltab_pose = false;
while(true)
{
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
	

				cerr<< "Waiting for command..."<<endl;
                                RPLcommand_vector=GetCommand(RPL_in_main);
                                cerr<<"got command"<<endl;

        if (RPLcommand_vector.size() == 0 )
          continue;
        if( !have_caltab_pose && RPLcommand_vector[0].ObjID != "CalTab")
                        {
                                RPLcommand_vector.push_back(AddCalTab);
				cerr <<endl<< "AUTOMATICALLY ADDING CALTAB SEARCH"<<endl<<endl;
			}
	
//	cerr<<"Received: "<<endl;
//	cerr<<"ID, numofObjects: "<< RPL_command.ObjID<<", "<<RPL_command.NumObjects<<endl;
	while(!RPLcommand_vector.empty()){
		RPL_command=RPLcommand_vector.back();
	
		ObjectToLookFor=RPL_command.ObjID;
		cerr<<"Object2C_str:"<<ObjectToLookFor<<endl;
                if (ObjectToLookFor=="CalTab")
                        {
                        g_calibTAB=true;
                        cerr<<"It's a calib tab. Setting g_calibTAB to true."<<endl;
                        }
                
//RUN THE TABLE CS LOCALIZATION:

//MOVE CALIBPLATE POSE TO TABLE CS

////////////////////////////////////////////////////////////
	
//CREATION OF POSE COVARIANCE MATRIX AND MEAN ON THE TABLE//

////////////////////////////////////////////////////////////
RelPose *RPFPose;
bool lo_there=false;
	while(!lo_there)
	{	
		lo_there=true;
		try {
					RPFPose = RelPoseFactory::FRelPose(5);	
				}
		catch(...)
		{
			cerr<<"Error getting pose from LO."<<endl;
			cerr<<"New attempt in 3 seconds."<<endl;
			sleep(3);
			lo_there=false;
		}
	}
	HTuple PoseHM(12,0.0), Pose(7,0.0), ObjPoseInTableCS(7,0.0),CovTuple(36,0.0);
	HTuple MeanPose;
	HTuple DomAxis2rpl;
		
	if (ObjectToLookFor=="Mug")
		{
		DomAxis2rpl="Y";
		}
	else //(ObjectToLookFor=="Plate")
		{
		DomAxis2rpl="Z";
		}

	cerr<<"TableCSPose:"<<endl<<TableCSPose<<endl;
	CovTuple=get_estd_pose_and_covmatrix(TableCSPose,&MeanPose,RPL_command.Pose,RPL_command.Reg);

	pose_to_hom_mat3d(MeanPose,&PoseHM);
	
	Matrix m(4, 4);	
  Matrix cov(6, 6);
	m=0;
	cov=0;
	
for (int i = 0; i < 3; i++) 
	{
		for (int j = 0; j < 4; j++) 
			{
				m.element(i, j) = PoseHM[i * 4 + j].D();
			}
	}

m.element(3, 3) = 1.0;

for (int i = 0; i < 6; i++) 
	{
		for (int j = 0; j < 6; j++)
			{
				cov.element(i, j) = CovTuple[i*6+j].D();
			}
        }

        //cerr<<""<<<<endl;
//cerr<< "Expected search extents to be used: "<<endl;
Matrix test_eposes;
HTuple empty(3,0.0),meanviewpoint;

RPFPose = RelPoseFactory::FRelPose(RPFPose, m, cov);
    yarp::os::Bottle LoBottle, CopAnswer;
    PutPoseIntoABottle_rok(LoBottle, RPFPose);
    printf("LoBottle: \n%s\n", LoBottle.toString().c_str());
  sleep(2);
	CopAnswer = talkToCop(&LoBottle, (char *)ObjectToLookFor.c_str(),RPL_command.NumObjects);

	//////////////////////GOT ANSWER FROM COP///////////////////////
	
	HTuple COP_PoseHM, COP_CovMat,ObjPoseInTableCSshort;	
	
	
	int nb;
	
	nb=CopAnswer.size();
	
	if (CopAnswer.get(0).asString()=="Locate failed.")
		{
			cerr<<"LOCATE FAILED. BAD IMAGE FOR CALTAB LOCALIZATION. \n "<<
			"PROBABLY HAVE TO RESTART COP.\n IN ANY CASE, CHANGE QUERY OR IMAGE."<<endl;
			break;
		}
      //added Dejan
  if (CopAnswer.get(0).asString()=="No Object Found!")
    {
      cerr<<"No Object Found!"<< endl;
      sleep(2);
      RPLcommand_vector.pop_back(); 
      continue;
    }
    
	cerr<<"RECEIVED BOTTLE: "<<CopAnswer.toString().c_str();
	
	cerr << "Cop answer bottle with "<<nb<<" elements received. Extracting... " << endl;
        string ImgFileName;
        for (int b=0;b<nb;b++)
        {
        yarp::os::Bottle OneAnsBot;
        OneAnsBot=*CopAnswer.get(b).asList();
        ExtractBottle(OneAnsBot,&COP_PoseHM,&ImgFileName);
//      cerr<<"Image FILENAME:\t"<<ImgFileName<<endl;
        
        if (g_calibTAB)
                {
                cerr<<endl<<endl<<"It's a CalTab! Updating the table coordinate system."<<endl<<endl;
                hom_mat3d_to_pose(COP_PoseHM,&CaltabPose);
                cerr<<"OK."<<endl;
                have_caltab_pose=true;
		caltab2table_pose_incam((const HTuple)CaltabPose,&TableCSPose);
		g_calibTAB=false;
                break;
                }
        
        cerr << "Transforming into table coordinates..." << endl;
        hom_mat3d_to_pose(COP_PoseHM, &Pose);

        obj_pose_cam2ref((const HTuple) TableCSPose, (const HTuple) Pose,&ObjPoseInTableCS);
                tuple_select_range(ObjPoseInTableCS,0,5,&ObjPoseInTableCSshort);
        //cerr<<"Object pose in Table CS: "<<endl<< ObjPoseInTableCS<<endl;

        SendPoseToProlog(ObjPoseInTableCSshort, (HTuple) ObjectToLookFor.c_str(), DomAxis2rpl, to_prolog_main, ImgFileName, OneAnsBot);

        OneAnsBot.clear();
    sleep(1); //TODO: LESS SLEEP
        }       
                cerr << "Everything fine" << endl;

	RPLcommand_vector.pop_back();	
}
}
//END OF LOCATING LOOP
	RPL_in_main.close();
    to_prolog_main.close();
 	cerr<<"Closed 1? "<<RPL_in_main.isClosed()<<endl;
	cerr<<"Closed 2? "<<to_prolog_main.isClosed()<<endl;
    usleep(1000);
	cerr<<"Closed ports.\n Exit."<<endl;
 return 0;
}
