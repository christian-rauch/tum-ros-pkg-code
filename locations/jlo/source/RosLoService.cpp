/**********************************************************************************************/
/**
*              Jennifer's Located Object
*              Copyright  (C) 2008, U. Klank
*
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*
 **********************************************************************************************/


#include "RosLoService.h"

#include "lo/ServiceLocatedObject.h"

#include <tf/transform_listener.h>
#include <string>
#include <boost/bind.hpp>

#define IDQUERY "idquery"
#define NAMEQUERY "namequery"
#define FRAMEQUERY "framequery"
#define DELETE "del"
#define UPDATE "update"
#define COUTLO "coutlo"
#ifdef _DEBUG
#define PRINTF_DEBUG(A) printf(A)
#else
#define PRINTF_DEBUG(A) printf(A)
#endif

RosLoService::RosLoService(const char* nodename, ros::NodeHandle &n, std::string configFile)
    : jlo::ServiceInterface(configFile.c_str()),
      located_object_service( n.advertiseService("/located_object", &RosLoService::ServiceCallback) )
{
  using namespace std;

  /**
  * Advertise the service that answers to normal lo-quieries
  */
  printf("Advertise Service \"/located_object\"\n");

  /**
  * Subscribe to a tf topic and listening to it.
  */
  if( n.hasParam( "tf_blacklist" ) )
  {
      using namespace XmlRpc;

      XmlRpcValue blacklist_param_value;
      n.getParam( "tf_blacklist", blacklist_param_value );
      if( blacklist_param_value.getType() != XmlRpcValue::TypeArray )
          ROS_WARN("tf_blacklist has invalid type.");
      else
      {
          for(int i=0; i<blacklist_param_value.size(); i++)
          {
              if( blacklist_param_value[i].getType() != XmlRpcValue::TypeString )
              {
                  ROS_WARN("Ignoring blacklist entry. Not a string");
                  continue;
              }
              string blacklist_entry = blacklist_param_value[i];
              tf_blacklist.insert( blacklist_entry );
              ROS_INFO("Adding '%s' to tf blacklist", blacklist_entry.c_str());
          }
      }
  }

  string tf_topic;
  n.param<string>( "tf_topic", tf_topic, "/tf" );
  printf("Subscribe \"%s\"\n", tf_topic.c_str());
  tf_subscription = n.subscribe<tf::tfMessage>( tf_topic, 100, boost::bind(&RosLoService::tf_subscription_callback, this, _1) );
}

RosLoService::~RosLoService()
{
  SaveList()->WriteToFile("bla.ini");
  printf("Wrote data to bla.ini\n");
}

bool PutLoIntoResponse( jlo::ServiceLocatedObject* lo, vision_srvs::srvjlo::Response&  answer)
{
  try
  {
  vision_msgs::partial_lo& out_lo = answer.answer;
  out_lo.id = lo->m_uniqueID;
  out_lo.parent_id = lo->m_parentID;
  int width = 4;
  Matrix m = lo->GetMatrix();
  Matrix cov = lo->GetCovarianceMatrix();
  lo->IncreaseReferenceCounter();
  for(int r = 0; r < width; r++)
  {
    for(int c = 0; c < width; c++)
    {
        out_lo.pose[r * width + c] = m.element(r,c);
    }
  }
  width = 6;
  for(int r = 0; r < width; r++)
  {
    for(int c = 0; c < width; c++)
    {
        out_lo.cov[r * width + c] = cov.element(r,c);
    }
  }
  answer.answer.type = lo->GetLOType();
  answer.answer.name = lo->m_mapstring;
  }
  catch(...)
  {
    printf("Critical Error in PutLoIntoResponse \n");
  }
  return true;
}

 bool RosLoService::ServiceCallback(vision_srvs::srvjlo::Request& request, vision_srvs::srvjlo::Response&  answer)
{
   try
   {
	if(request.command.compare(IDQUERY) == 0)
	{
	       /* case 1: One number can only mean an id: ID-Query*/
              jlo::ServiceLocatedObject* lo = GetServiceLocatedObject(request.query.id);
              /*printf("Requested ID: %d => %p\n", (int)request.query.id, lo);*/
              if (lo == NULL)
              {
                PRINTF_DEBUG("Error Parsing Input: Id does not exist!\n");
                answer.error = "Error Parsing Input: Id does not exist!\n";
                return false;
              }
              PutLoIntoResponse(lo, answer);
	}
	else if(request.command.compare(NAMEQUERY) == 0)
	{
	  /* case 1: One number can only mean an id: ID-Query*/
	  long id = GetServiceLocatedObjectID(request.query.name);
	  if(id != -1)
	  {
            jlo::ServiceLocatedObject* lo = GetServiceLocatedObject(id);
            if (lo == NULL)
            {
              PRINTF_DEBUG("Error Parsing Input: Id does not exist!\n");
              answer.error = "Error Parsing Input: Id does not exist!\n";
              return false;
            }
            PutLoIntoResponse(lo, answer);
	  }
	  else
	  {
	    PRINTF_DEBUG("Error Parsing Input: Name does not exist!\n");
            answer.error = "Error Parsing Input: Name does not exist!\n";
            answer.answer.id = 0;
            return false;
	  }
	}
	else if (request.command.compare(FRAMEQUERY) == 0)
        {
           try
           {
                unsigned long parentID = request.query.parent_id;
                jlo::ServiceLocatedObject* lo = GetServiceLocatedObject(request.query.id);
                if(lo != NULL && request.query.id == parentID)
                {
                   answer.error = "Warning Parsing Input: Self reference ignored!\n";
                   PutLoIntoResponse(lo, answer);
                }
                else
                {
                  if (lo == NULL)
                  {
                    PRINTF_DEBUG("Error Parsing Input: Id does not exist!\n");
                    answer.error = "Error Parsing Input: Id does not exist!\n";
                    return false;
                  }
                  if(lo->m_uniqueID != ID_WORLD && lo->m_relation->m_uniqueID == (unsigned long)parentID)
                  {
                    PutLoIntoResponse(lo, answer);
                  }
                  else
                  {
                    jlo::ServiceLocatedObject* parent = GetServiceLocatedObject(parentID);
                    if (parent == NULL)
                    {
                       PRINTF_DEBUG("Error Parsing Input: Parent Id does not exist!\n");
                       answer.error = "Error Parsing Input: Parent Id does not exist!\n";
                       return false;
                    }
                    Matrix mat = lo->GetMatrix(*parent);
                    Matrix cov = lo->GetCovarianceMatrix(*parent);

                    PutLoIntoResponse(FServiceLocatedObject(parent, mat, cov, 0), answer);
                  }
                }
              }
              catch(...)
              {
                printf("Critical error in FrameQuery\n");
                return false;
              }
            }
            else if(request.command.compare(DELETE) == 0)
            {
                  unsigned long id = request.query.id;
                  if(id != ID_WORLD)
                  {
                    jlo::ServiceLocatedObject* obj  = GetServiceLocatedObject(id);
                    if(obj == NULL)
                    {
                       PRINTF_DEBUG("Error: Can't delete ID_WORLD!\n");
                       answer.error = "Error: Can't delete ID_WORLD!\n";
                       return false;
                    }
                    else
                    {
                      unsigned long ref_count = obj->GetReferenceCounter();
                      FreeServiceLocatedObject(obj);
                      if(ref_count > 1)
                      {
                        answer.error = "Object is still referenced.";
                      }
                      else
                      {
                        answer.error = "Object deleted.";
                      }
     	            }
                  }
                  else
                  {
                    PRINTF_DEBUG("Error: Can't delete ID_WORLD!\n");
                     answer.error = "Error: Can't delete ID_WORLD!\n";
                    return false;
                  }
            }
 /*case error */
/*semi complete*/
          else if(request.command.compare(UPDATE) == 0)
          {
            unsigned long parentID, id;
            bool update = false;
            unsigned long type = request.query.type;
            id =  request.query.id;
            parentID =  request.query.parent_id;
            if(id != 0)
            {
              update = true;
            }
            Matrix mat(4,4), cov(6,6);

            int width = 4;
            for(int r = 0; r < width; r++)
            {
              for(int c = 0; c < width; c++)
              {
                  if((unsigned)(r * width + c) < request.query.pose.size())
                     mat.element(r,c) = request.query.pose[r * width + c];
                  else
                    printf("Wrong length of pose\n");

              }
            }
            width = 6;
            for(int r = 0; r < width; r++)
            {
              for(int c = 0; c < width; c++)
              {
                  if((unsigned)(r * width + c) < request.query.cov.size())
                     cov.element(r,c) = request.query.cov[r * width + c];
                  else
                    printf("Wrong length of cov\n");
              }
            }
            if(update)
            {
              jlo::ServiceLocatedObject* pose = GetServiceLocatedObject(id);
              if(pose == NULL)
              {
                jlo::ServiceLocatedObject* parent = GetServiceLocatedObject(parentID);
                if(parent == NULL)
                {
                  PRINTF_DEBUG("Error Parsing Bottle: Requested parent does not exist!\n");
                  answer.error = "Error Parsing Bottle: Requested parent does not exist!\n";
                  return false;
                }
                jlo::ServiceLocatedObject* pose = FServiceLocatedObject(parent, mat, cov, type);
                PutLoIntoResponse(pose, answer);
                if(request.query.name.length() > 0 && GetServiceLocatedObjectID(request.query.name) == -1)
                {
                   pose->m_mapstring = request.query.name;
                }
              }
              else
              {
                jlo::ServiceLocatedObject* parent = GetServiceLocatedObject(parentID);
                if(parent == NULL || (parent->m_uniqueID == ID_WORLD && pose->m_uniqueID == ID_WORLD))
                {
                  PRINTF_DEBUG("Error Parsing Bottle: Requested parent does not exist!\n");
                   answer.error = "Error Parsing Bottle: Requested parent does not exist!\n";
                  return false;
                }
                if((parent->m_uniqueID == ID_WORLD && pose->m_uniqueID == ID_WORLD))
                {
                    answer.error = "Error: Asked for world in world\n";
                    return false;
                }
                pose->Update(mat, cov, ServiceInterface::FServiceLocatedObjectCopy, ServiceInterface::FreeServiceLocatedObject);
                if(request.query.name.length() > 0 && GetServiceLocatedObjectID(request.query.name) == -1)
                {
                   pose->m_mapstring = request.query.name;
                }
                PutLoIntoResponse(pose, answer);
              }
            }
            else
            {
              jlo::ServiceLocatedObject* parent = GetServiceLocatedObject(parentID);
              if(parent == NULL)
              {
                PRINTF_DEBUG("Error Parsing Bottle: Requested parent does not exist!\n");
                answer.error = "Error Parsing Bottle: Requested parent does not exist!\n";
                return false;
              }
              jlo::ServiceLocatedObject* pose = FServiceLocatedObject(parent, mat, cov, type);
              if(request.query.name.length() > 0 && GetServiceLocatedObjectID(request.query.name) == -1)
              {
                 pose->m_mapstring = request.query.name;
              }
              PutLoIntoResponse(pose, answer);
            }
        }
        else if (request.command.compare(COUTLO) == 0)
        {
          jlo::ServiceLocatedObject* lo = GetServiceLocatedObject(request.query.id);
          /*printf("Requested ID: %d => %p\n", (int)request.query.id, lo);*/
          if (lo == NULL)
          {
            PRINTF_DEBUG("Error Parsing Input: Id does not exist!\n");
            answer.error = "Error Parsing Input: Id does not exist!\n";
            return false;
          }
          PutLoIntoResponse(lo, answer);
          std::string st = lo->SaveComplete()->WriteToString();
          cout << st << endl;
        }
        else
        {
          printf("Command %s not accepted\n", request.command.c_str());
           PRINTF_DEBUG("Error: Requested command does not exist!\n");
           answer.error = "Error Parsing Bottle: Requested parent does not exist!\n";
           return false;
        }
        return true;
      }
      catch(...)
      {
        printf("An exception occured during parsing the message!\n");
        return false;
      }
}

void RosLoService::tf_subscription_callback(const tf::tfMessage::ConstPtr &msg_in_)
{
  for (unsigned int i = 0; i < msg_in_->transforms.size(); i++)
  {
    tf::Stamped<tf::Transform> trans;
    /*Obsolete: TransformStampedMsgToTF, if you get here an error cause the following function is not defined, pls update tf*/
    transformStampedMsgToTF(msg_in_->transforms[i], trans);
    if( tf_blacklist.find( trans.frame_id_ ) != tf_blacklist.end() )
        continue;


 /*   printf("got matrix:= \n[");
    for(int i = 0; i < 16; i++)
    {
      printf("%f ", matrix[i]);
      if(i % 4  == 3)
        printf("\n ");
    }
    printf("]\n");*/
    try
    {
      std::map<std::string, std::string>* msg_header_map = msg_in_->__connection_header.get();
      std::string authority;
      std::map<std::string, std::string>::iterator it = msg_header_map->find("callerid");
      if (it == msg_header_map->end())
      {
        ROS_WARN("Message recieved without callerid");
        authority = "no callerid";
      }
      else
      {
        authority = it->second;
      }
      /*setTransform(trans, authority);*/
    }
    catch(...)
    {
       ROS_ERROR("Failure to set recieved transform\n");
    }
    if(trans.frame_id_.length() == 0)
    {
      printf("Empty tf\n");
      return;
    }
    else
    {
      //printf("Getting tf %s parent %s\n", trans.frame_id_.c_str(), trans.parent_id_.c_str());
    }

    long id = (long)GetServiceLocatedObjectID(trans.frame_id_);
    long parentid = (long)GetServiceLocatedObjectID(trans.parent_id_);
    if(parentid == -1)
    {
      parentid = ID_WORLD;
      /*printf("Error: Requested parent does not exist!\n");*/

      jlo::ServiceLocatedObject* parent = GetServiceLocatedObject(parentid);
      parent->m_mapstring = trans.parent_id_;
    }

    jlo::ServiceLocatedObject* pose = NULL;

    btScalar matrix[16];
    trans.getOpenGLMatrix(matrix);

    Matrix mat(4,4);
    mat << matrix[0] << matrix[4]<< matrix[8]<< matrix[12]
        << matrix[1]<< matrix[5]<< matrix[9]<< matrix[13]
        << matrix[2]<< matrix[6]<< matrix[10]<< matrix[14]
        << matrix[3]<< matrix[7]<< matrix[11]<< matrix[15];

    Matrix cov(6,6);

    cov << 0 << 0 << 0 << 0 << 0 << 0
        << 0 << 0 << 0 << 0 << 0 << 0
        << 0 << 0 << 0 << 0 << 0 << 0
        << 0 << 0 << 0 << 0 << 0 << 0
        << 0 << 0 << 0 << 0 << 0 << 0
        << 0 << 0 << 0 << 0 << 0 << 0;

    int type = 1;

    if(id == -1)
    {
      jlo::ServiceLocatedObject* parent = GetServiceLocatedObject(parentid);
      if(parent == NULL)
      {
          printf("Error tf: Requested parent does not exist! (%s -> %ld)\n", trans.parent_id_.c_str(),  parentid);
          return;
      }
      pose = FServiceLocatedObject(parent, mat, cov, type);
      pose->m_mapstring = trans.frame_id_;
    }
    else
    {
      pose = GetServiceLocatedObject(id);
      if (pose == NULL)
      {
        printf("Error tf: Reject overwritten frame %s -> %ld\n", trans.frame_id_.c_str(), id);
        return;
      }

      if(parentid == pose->m_parentID)
      {
          Matrix m = pose->GetMatrix();
          Matrix diff =  mat - m;
          if((diff.element(0,0) < 0.0001 &&
             diff.element(0,1) < 0.0001 &&
             diff.element(0,3) < 0.0001 &&
              diff.element(0,3) < 0.0001 &&
              diff.element(1,0) < 0.0001 &&
              diff.element(1,1) < 0.0001 &&
              diff.element(1,2) < 0.0001 &&
              diff.element(1,3) < 0.0001 &&
              diff.element(2,0) < 0.0001 &&
              diff.element(2,1) < 0.0001 &&
              diff.element(2,2) < 0.0001 &&
              diff.element(2,3) < 0.0001) || pose->m_uniqueID == 1)
          {
            //printf("no significant change: ignoring tf\n");
            return;
          }
          pose->Update(mat, cov, ServiceInterface::FServiceLocatedObjectCopy, ServiceInterface::FreeServiceLocatedObject);
      }
      else
      {
          pose->m_mapstring = "";
          jlo::ServiceLocatedObject* parent = GetServiceLocatedObject(parentid);
          if(parent == NULL)
          {
              printf("Error tf: Requested parent does not exist! (%s -> %ld)\n", trans.parent_id_.c_str(), parentid);
              return;
          }
          pose = FServiceLocatedObject(parent, mat, cov, type);
          pose->m_mapstring = trans.frame_id_;
      }
    }

    /*catch (TransformException& ex)
    {
      ///\todo Use error reporting
      std::string temp = ex.what();
      ROS_ERROR("Failure to set recieved transform %s to %s with error: %s\n", msg_in_.transforms[i].header.frame_id.c_str(), msg_in_.transforms[i].parent_id.c_st
r(), temp.c_str());
    }*/
  }



};


