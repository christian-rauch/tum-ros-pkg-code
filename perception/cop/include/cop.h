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

 
/************************************************************************
                        cop.h - Copyright klank


**************************************************************************/


#ifndef COP_H
#define COP_H

#include "VisFinder.h"
#include "VisLearner.h"
#include "CameraDriver.h"
#include "SimulatedCamera.h"

#ifdef HALCONIMG

#include "cpp/HalconCpp.h"
//This procedure simply hands the exception object to the C++ exception handling via throw:
void MyHalconExceptionHandler(const Halcon::HException& except)
{
  printf("%s\n ", except.message);
  throw except;
}


#endif /*HALCONIMG*/

#ifdef LOGFILE

#define LOGFILE_ADDED , *s_logFile

#else /*LOGFILE*/

#define LOGFILE_ADDED

#endif /*LOGFILE*/

volatile bool g_stopall = false;

#ifdef BOOST_THREAD
#define BOOST(A) A
#include <signal.h>
#else
#define BOOST(A)
#endif /*BOOST_THREAD*/

/* LO Service*/
#ifdef USE_YARP_COMM

#include "YarpComm.h"

#ifdef NO_LO_SERVICE_AVAILABLE
#else /**NO_LO_SERVICE_AVAILABLE*/
#include "YarpRpcComm.h"
Comm* RelPoseFactory::s_loService = NULL;
#endif /**NO_LO_SERVICE_AVAILABLE*/

#else /**USE_YARP_COMM*/

#include "ROSComm.h"

#ifdef NO_LO_SERVICE_AVAILABLE
#else /**NO_LO_SERVICE_AVAILABLE*/
#include "ROSjloComm.h"
#include <ros/ros.h>
Comm* RelPoseFactory::s_loService = NULL;
#endif /**NO_LO_SERVICE_AVAILABLE*/

#endif /**USE_YARP_COMM*/

#ifdef BOOST_THREAD
void CTRLC(int)
{
  g_stopall = true;
#ifdef BOOST_1_35
  BOOST(boost::system_time t);
#else
  boost::xtime t;
#endif

#ifdef BOOST_1_35
  BOOST(t = get_system_time());
  BOOST(t += boost::posix_time::seconds(1));
#else
  boost::xtime_get(&t, boost::TIME_UTC);
  t.sec += 1;
#endif

#ifdef USE_YARP_COMM
  yarp::os::Network::fini();
#endif /**USE_YARP_COMM*/
  boost::thread::sleep(t);
  exit(-1);

}
#endif /*BOOST_THREAD*/
#ifdef USE_PLAYER
#include <libplayercore/playercore.h>
#include <libplayercore/error.h>
#endif
/********************************************************************
*   class cop_world                                                       */
/** ******************************************************************
*   @brief Loads an initial state for the framework from an xml file
*
*
*   @remarks Several defines can restrict the compiled functionality:
*   HALCONIMG               Enables the use of Halcon images and Halcon image qcquisition,
                            also several algorithms implemented with halcon
*   _DEBUG                  Enables debug messages
*   BOOST_THREAD            Enables multithread image acquisistion and learning
*   LOGFILE                 Logs actions of the VisFinder and VisLearner systems
*   OPENCV_USED             Enables the conversion of Hobjects to IplImages
*   DEFORMSHAPE_AVAILABLE   Enables the usage of the planar matching of hofhausa@in.tum.de
*   DESCRIPTOR_AVAILABLE    Enables the usage of an 3D descriptor matching
*
*********************************************************************/
class cop_world
{
public:
  ImageInputSystem* s_inputSystem;
  SignatureDB*      s_sigDb;
  AttentionManager* s_attManager;
  VisFinder*        s_visFinder;
  VisLearner*       s_visLearner;
  Statistics*       s_stats;
#ifdef LOGFILE
    LogFile*            s_logFile;
#endif /*LOGFILE*/
#ifndef NO_LO_SERVICE_AVAILABLE
#ifndef USE_YARP_COMM
  ros::NodeHandle* s_node;
#endif
#endif
protected:

    virtual int Setup()
  {
    return 0;
  }
    virtual int Shutdown()
  {
    return 0;
  }
public:
  cop_world(XMLTag* config) :
    s_inputSystem(NULL),
    s_sigDb(NULL),
    s_attManager(NULL),
    s_visFinder(NULL),
    s_visLearner(NULL),
    s_stats(NULL)
#ifdef LOGFILE
        ,s_logFile(NULL)
#endif /*LOGFILE*/
#ifndef NO_LO_SERVICE_AVAILABLE
#ifndef USE_YARP_COMM
    ,s_node(NULL)
#endif
#endif
  {
     BOOST(signal(SIGINT, CTRLC));
#ifdef HALCONIMG
      /*Install the exception handler, implemented above*/
      Halcon::HException::InstallHHandler(&MyHalconExceptionHandler);
  #endif
#ifdef NO_LO_SERVICE_AVAILABLE
      RelPoseFactory::LoadList(config->GetChild(XML_NODE_RELPOSELIST));

#else
#ifdef USE_YARP_COMM
      std::string loPortName = STD_LO_RPC_PORT_INTERNAL;
      if(config->GetChild(XML_NODE_RELPOSELIST) != NULL)
      {
        std::string temp = config->GetChild(XML_NODE_RELPOSELIST)->GetProperty(XML_PROPERTY_LO_RPC_PORT);
        if(temp.length() > 0)
          loPortName = temp;
      }
      RelPoseFactory::s_loService = new YarpRpcComm(loPortName);
#else
      s_node = new ros::NodeHandle();
      std::string loPortName = STD_LO_RPC_PORT_INTERNAL;
      if(config->GetChild(XML_NODE_RELPOSELIST) != NULL)
      {
        std::string temp = config->GetChild(XML_NODE_RELPOSELIST)->GetProperty(XML_PROPERTY_LO_RPC_PORT);
        if(temp.length() > 0)
          loPortName = temp;
      }
      try
      {
      RelPoseFactory::s_loService = new ROSjloComm(loPortName);
#endif
#endif
#ifdef LOGFILE
            XMLTag* logfilename = config->GetChild(XML_NODE_LOGFILE);
            s_logFile = new  LogFile(logfilename != NULL ? logfilename->GetCDataST() : "");
#ifdef _DEBUG
      printf("Log file opened\n");
  #endif
    #endif /*LOGFILE*/
      s_inputSystem = new ImageInputSystem( config->GetChild(XML_NODE_IMAGEINPUTSYSTEM));
#ifdef _DEBUG
      printf("Camera System loaded\n");
  #endif
      s_sigDb = new SignatureDB (config->GetChild(XML_NODE_SIGNATUREDB));
#ifdef _DEBUG
      printf("Signature DB loaded\n");
  #endif
      s_attManager = new AttentionManager(config->GetChild(XML_NODE_ATTENTIONMANAGER), *s_inputSystem LOGFILE_ADDED);
#ifdef _DEBUG
      printf("Attention Management loaded\n");
  #endif
      s_visLearner = new VisLearner(config->GetChild(XML_NODE_VISLEARNER), *s_sigDb, *s_inputSystem LOGFILE_ADDED,  true);
#ifdef _DEBUG
      printf("Visial Learner Loaded\n");
  #endif
      XMLTag* visfind = config->GetChild(XML_NODE_VISFINDER);
      if(visfind == NULL)
      {
        throw "Outdated xml file";
      }
        /*visfind = config->GetChild(XML_NODE_ALGORITHMSELECTOR);*/
      s_visFinder = new VisFinder(visfind, *s_inputSystem, *s_sigDb, *s_attManager, *s_visLearner LOGFILE_ADDED);
#ifdef _DEBUG
      printf("Algorithm Selection loaded\n");
  #endif
      }
      catch(const char* text)
      {
        printf("Error loading cop: %s\n", text);
      }
  }

  ~cop_world()
  {
    delete s_inputSystem;
    delete s_visLearner;
    delete s_attManager;
    delete s_visFinder;
    delete s_sigDb;
    delete s_stats;
#ifndef NO_LO_SERVICE_AVAILABLE
#ifndef USE_YARP_COMM
    delete s_node;
#endif
#endif
    RelPoseFactory::DisposeList();
  }
    void SaveCop(std::string filename)
    {
      XMLTag* config = new XMLTag("config");
#ifdef NO_LO_SERVICE_AVAILABLE
      printf("Save relpose list\n");
      config->AddChild(RelPoseFactory::SaveList());
#endif /*NO_LO_SERVICE_AVAILABLE*/
      printf("Save vis finder\n");
      config->AddChild(s_visFinder->Save());
      printf("Save vis learner\n");
      config->AddChild(s_visLearner->Save());
      printf("Save vis input system\n");
      config->AddChild(s_inputSystem->Save());
      printf("Save vis db\n");
      config->AddChild(s_sigDb->Save());
      printf("Save vis att man\n");
      config->AddChild(s_attManager->Save());
      printf("write it\n");
      config->WriteToFile(filename);
    }

#ifdef USE_YARP_COMM
    /**
    *   StartListeningYarpPort
    *
    *    @brief Waits on the specified yarp port for incoming locate or track requests.
    *
    *    @param yarpbottle name of the yarp bottle to listen for new incoming data
    */
    void StartListeningYarpPort(char* yarpbottle)
    {
      printf("Entering StartListeningYarpPort\n");
      yarp::os::Network yarp_network;
      yarp_network.init();
      YarpPortManager port_manager;
      try
      {
        yarp::os::BufferedPort<yarp::os::Bottle>* port = port_manager.OpenCommOnYarpPort(yarpbottle);
        port_manager.Listen(*port, g_stopall, s_visFinder, s_sigDb);
      }
      catch(char const* text)
      {
        printf("StartListeningYarpPort: Port %s could not openend (Error: %s).\n", yarpbottle, text);
      }
      catch(...)
      {
        printf("StartListeningYarpPort: Unknown Exception\n");
      }
      printf("Leaving Function\n");
    }
#else
    /**
    *   StartNodeSubscription
    *
    *    @brief Subscripe a specified ROS topic for incoming locate or track requests.
    *
    *    @param name of the node to listen for new incoming data
    */
    void StartNodeSubscription(char* topic)
    {
      printf("Entering StartNodeSubscription\n");

      ROSTopicManager port_manager(s_visFinder, s_sigDb);
      try
      {
        /*TODO*/
        port_manager.Listen(topic, g_stopall, s_node);
        g_stopall = true;
      }
      catch(char const* text)
      {
        printf("StartListening Failed: Service %s could not be advertised. Error: %s\n", topic, text);
      }
      catch(...)
      {
        printf("StartListeningYarpPort: Unknown Exception\n");
      }
      printf("Leaving Function\n");
    }
#endif /*USE_YARP_COMM*/
    /**
    *   AddElement
    *
    *    @brief Allows to change the configuration file by adding an element.
    *
    *    @remarks The element should be recognized via the noe name passed.
    *    @return if the node name was unknown and could not be interpreted
    *
    *    @param tag a tag with a correct tagname for identifying it as a element used by cop
    */
    bool AddElement(XMLTag* tag)
    {
        try
        {
            Camera* cam = Camera::CamFactory(tag);
            if(cam != NULL)
            {
                s_inputSystem->AddCamera(cam);
                return true;
            }
            Elem* elem = Elem::ElemFactory(tag);
            if(elem != NULL)
            {
                switch(elem->GetType())
                {
                case SIGNATURE:
                    s_sigDb->AddSignature((Signature*)elem);
                    return true;
                case CLASS:
                    s_sigDb->AddClass(((Class*)elem)->GetName(), ((Class*)elem)->m_ID);
                    return true;
                default:
                    break;
                }
            }
            Algorithm<std::vector<RelPose*> >* alg = LocateAlgorithm::AlgFactory(tag);
            if(alg != NULL)
            {
                s_visFinder->AddAlgorithm(alg);
                return true;
            }
            /* TODO: Refine algorithm*/
        }
        catch(char const*
#ifdef _DEBUG
            temp
#endif /*_DEBUG*/
            )
        {
#ifdef _DEBUG
            printf("%s\n", temp);
#endif /*_DEBUG*/
            return false;
        }
        return false;
    }
};
#endif /*COP_H*/
