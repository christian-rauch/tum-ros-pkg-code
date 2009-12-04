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
                        SignatureDB.cpp - Copyright klank


**************************************************************************/

#include "SignatureDB.h"

#include <time.h>

#ifdef BOOST_THREAD
#include <boost/bind.hpp>
#endif


#define XML_NODE_SIGDB_INTERN_DB	"SigDBRoot"
#define XML_NODE_SIGDB_INTERN_INDEX	"SigDBIndex"
#define XML_NODE_SIGDB_INTERN_IDLIST "SigDBIDList"


#define XML_NODE_SIGDB_INTERN_INDEX_CLASS2ID	"SigDBIndexClass2ID"
#define XML_NODE_SIGDB_INTERN_INDEX_ID2CLASS	"SigDBIndexID2Class"

#define XML_NODE_SIGDB_INTERN_INDEX_CLASS2ID_MEM	"idMem"
#define XML_NODE_SIGDB_INTERN_INDEX_ID2CLASS_MEM	"classMem"

#define XML_ATTRIBUTE_SIGDB_CLASS "Class"
#define XML_ATTRIBUTE_SIGDB_ID "ID"



using namespace cop;


class Class2ID : XMLTag
{
public:
	Class2ID(int idClass, int id) :
		XMLTag(XML_NODE_SIGDB_INTERN_INDEX_CLASS2ID_MEM)
	{
		this->AddProperty(XML_ATTRIBUTE_SIGDB_CLASS, idClass);
		this->AddProperty(XML_ATTRIBUTE_SIGDB_ID, id);
	}
};

// Constructors/Destructors
//

SignatureDB::SignatureDB ( XMLTag* config )
{
	if(config != NULL)
	{
		try
		{
			try
			{
				XMLTag* tagSBRoot = config->GetChild(XML_NODE_SIGDB_INTERN_DB);
				if(tagSBRoot != NULL)
					m_dbStarter = tagSBRoot->Clone();
				else
					m_dbStarter = new XMLTag(XML_NODE_SIGDB_INTERN_DB);
			}
			catch(...)
			{
        printf("Creating Signature DB: Error in node %s\n", XML_NODE_SIGDB_INTERN_DB);
				throw XML_NODE_SIGDB_INTERN_DB;
			}
			try
			{
				XMLTag* tagSBIndex= config->GetChild(XML_NODE_SIGDB_INTERN_INDEX);
				if(tagSBIndex != NULL)
				{
					m_index = tagSBIndex->Clone();
					if(m_index->GetChild(XML_NODE_SIGDB_INTERN_INDEX_CLASS2ID) == NULL)
						m_index->AddChild(new XMLTag(XML_NODE_SIGDB_INTERN_INDEX_CLASS2ID));
				}
				else
				{
					m_index = new XMLTag(XML_NODE_SIGDB_INTERN_INDEX);
					m_index->AddChild(new XMLTag(XML_NODE_SIGDB_INTERN_INDEX_CLASS2ID));
				}
			}
			catch(...)
			{
        printf("Creating Signature DB: Error in node %s\n", XML_NODE_SIGDB_INTERN_INDEX);
				throw XML_NODE_SIGDB_INTERN_INDEX;
			}
			try
			{
				XMLTag* tagSBIdList= config->GetChild(XML_NODE_SIGDB_INTERN_IDLIST);
				if(tagSBIdList != NULL)
				{
					m_ids = XMLTag::Load(tagSBIdList, &m_ids);
					if(m_ids.size() != m_dbStarter->CountChildren())
						UpdateIDList();
				}
				else
					UpdateIDList();
			}
			catch(char const* pListError)
			{
				printf("Error reading Singature DB: %s\n", pListError);
				throw XML_NODE_SIGDB_INTERN_IDLIST;
			}
            catch(...)
            {
				printf("Error reading Singature DB.\n");
				throw XML_NODE_SIGDB_INTERN_IDLIST;
            }
		}
		catch(char const* exception)
		{
			printf("Loading of config file failed: %s\n", exception);
			m_dbStarter = new XMLTag(XML_NODE_SIGDB_INTERN_DB);
			m_index = new XMLTag(XML_NODE_SIGDB_INTERN_INDEX);
			m_index->AddChild(new XMLTag(XML_NODE_SIGDB_INTERN_INDEX_CLASS2ID));
		}
	}
	else
	{
		m_dbStarter = new XMLTag(XML_NODE_SIGDB_INTERN_DB);
		m_index = new XMLTag(XML_NODE_SIGDB_INTERN_INDEX);
		m_index->AddChild(new XMLTag(XML_NODE_SIGDB_INTERN_INDEX_CLASS2ID));
	}

}

SignatureDB::~SignatureDB ( )
{
	delete m_dbStarter;
	delete m_index;
}

//
// Methods
//
#ifdef BOOST_THREAD
void SignatureDB::AddAndShowSignatureAsync(Signature* sig, Sensor* sens)
{
  boost::thread(boost::bind(&SignatureDB::AddSignature, this, sig));
  boost::thread(boost::bind(&Signature::Show, sig, sens));
}
#endif

int SignatureDB::AddSignature(Signature* sig)
{
    int error = -1;
#ifndef WIN32
    sleep(0.01);
#endif
    if(sig != NULL)
    {
        if(!Check(sig->m_ID, error))
        {
            error = m_dbStarter->AddChild(sig->Save());
            m_ids.push_back(sig->m_ID);
            int indizes = m_index->CountChildren();
            for(int i = 0; i < indizes; i++)
            {
                XMLTag* sub_index = m_index->GetChild(i);
                if(sub_index->GetName().compare(XML_NODE_SIGDB_INTERN_INDEX_CLASS2ID)== 0)
                {
                    int id = sig->m_ID;
                    for(unsigned int classes = 0; classes < sig->CountClasses(); classes++)
                    {
                        Class* cl = sig->GetClass(classes);
                        sub_index->AddChild((XMLTag*)new Class2ID(cl->m_ID, id));
                        AddClass(cl->GetName(), cl->m_ID);
                    }
                }
            }
        }
        else
        {
            UpdateNodes(sig, error);
        }
        if(error != -1)
        {
          CleanUpActiveSignatureList();
          AddSignatureToActiveList(sig, error);
        }
    }
    return error;
}

void SignatureDB::AddSignatureToActiveList(Signature* sig, int index)
{
  m_currentlyActiveSignatures.push_back(std::pair<Signature*, int>(sig, 1));
  m_activeMap[index] = m_currentlyActiveSignatures.size() - 1;
}


void SignatureDB::UpdateNodes(Signature* sig, int index)
{
  printf("SignatureDB::UpdateNodes\n");
  m_dbStarter->ReplaceChild(sig->Save(), index);
}

Signature* SignatureDB::GetSignatureByIndex(unsigned int index)
{
  if(index < m_dbStarter->CountChildren())
  {
    if(m_activeMap.find(index) != m_activeMap.end())
    {
      int index_active = m_activeMap[index];
      m_currentlyActiveSignatures[index_active].second++;
      return m_currentlyActiveSignatures[index_active].first;
    }
    else
    {
     try
     {
       Signature* sig = (Signature*)Elem::ElemFactory(m_dbStarter->GetChild(index));
       AddSignatureToActiveList(sig, index);
       return sig;
     }
     catch(char const* error_text)
     {
         printf("Error loading signature: %s\n", error_text);
         return NULL;
     }
    }
  }
  else
    return NULL;
}

#define TIME_OUT_ACTIVE_SIG 30000

void SignatureDB::CleanUpActiveSignatureList()
{
  std::vector<std::pair<Signature*, int> >::iterator it = m_currentlyActiveSignatures.begin();
  unsigned long timeStamp = (unsigned long)time(NULL);
  int index = 0;
  for(;it != m_currentlyActiveSignatures.end(); it++)
  {
    if((*it).second == 0 && timeStamp - (*it).first->date() > TIME_OUT_ACTIVE_SIG)
    {
      delete (*it).first;
      it = m_currentlyActiveSignatures.erase(it);
      m_activeMap.erase(m_activeMap[index]);
    }
    else
      index++;
  }
}


Signature* SignatureDB::GetSignatureByID(int ElemID)
{
  int index;
  Check(ElemID, index);
  return GetSignatureByIndex(index);
}


void SignatureDB::AddClass(std::string stname, int id)
{
  if(CheckClass(id).length() == 0)
    m_classes.push_back(std::pair<std::string, int>(stname, id));
   else
     printf("Already defined class id %d: %s\n", id, CheckClass(id).c_str());
}

Signature* SignatureDB::GetSignature(std::vector<int> class_ids)
{
  size_t size = class_ids.size();
  int score_max = 0, score_temp = 0;
  Signature* sig_max = NULL;
  int index = -1;
  printf("Entering Get Signature\n");
  for(unsigned int i = 0 ; i < size; i++)
  {
    Signature* sig = NULL;
    int offset = 0;
    /** Get a signature for the given class*/
    while((sig = GetSignatureByClass(class_ids[i], offset++)) != NULL)
    {
      /** Check this signature for containing more of the given classes*/
      for(unsigned int j = 0; j < sig->CountClasses(); j++)
      {
        for(unsigned int k = 0 ; k < size; k++)
        {
          if(k == i)
          {
            score_temp++;
            continue;
          }
          if(sig->GetClass(j) != NULL && class_ids[k] == sig->GetClass(j)->m_ID)
          {
            score_temp++;
            break;
          }
        }
      }
      /** Keep the best signature*/
      if(score_temp > score_max)
      {
        score_max = score_temp;
        index = i;
        if(sig_max != NULL)
          FreeActiveSignature(sig_max);
        /** Copy the best*/
        sig_max = (Signature*)sig;
      }
      else
        FreeActiveSignature(sig);
    }
    score_temp = 0;
  }
  /** if there was no signature, put all classes to a new sig*/
 if(sig_max != NULL)
    printf("Signature selected: %p: id: %d\n", sig_max, sig_max->m_ID);

  if(sig_max == NULL && size != 0)
  {
    sig_max = new Signature();
  }
  if(sig_max != NULL)
  {
    for(unsigned int k = 0 ; k < size; k++)
    {
      bool bIn = false;
      for(unsigned int j = 0; j < sig_max->CountClasses(); j++)
      {
        if(sig_max->GetClass(j) != NULL && class_ids[k] == sig_max->GetClass(j)->m_ID)
        {
          bIn = true;
          break;
        }
      }
      if(!bIn)
      {
        try
        {
          /** Add known Descriptior*/
          Elem* elem = FindCreateDescriptor(class_ids[k]);
          if(elem != NULL)
            sig_max->SetElem(elem);
          else
          {
            printf("No descriptor could be found or creted for class %d (=%s)\n", class_ids[k], CheckClass(class_ids[k]).c_str());
          }
        }
        catch(char const* text)
        {
          printf("Problem while building signature: %s\n", text);
        }
      }
    }
  }
  /** Returning the best*/
  return sig_max;
}

Elem* SignatureDB::FindCreateDescriptor(int class_id)
{
  Elem* result = NULL;
  Signature* sample = GetSignatureByClass(class_id);
  if(sample != NULL)
  {
    int count_cont = 1;
    std::vector<Descriptor*> cands;
    while(true)
    {
      for(size_t i = 0; i < sample->CountElems(); i++)
      {
        Descriptor* descr = (Descriptor*)sample->GetElement(i, -1);
        if(descr->GetClass()->m_ID == class_id)
        {
          cands.push_back(descr);
        }
      }
      Signature* sample = GetSignatureByClass(class_id, count_cont++);
      if(sample == NULL)
        break;
    }
    if(cands.size() > 0)
    {
      /*TODO introduce metric for different descriptors for a class or add all...*/
      /*std::sort(cands.begin(), cands.end());*/
      result = cands[0];
    }
  }
  else
  {
    std::string searchstring = CheckClass(class_id);
  }
  return result;
}

void SignatureDB::FreeActiveSignature(Signature* sig)
{
  std::vector<std::pair<Signature*, int> >::iterator it = m_currentlyActiveSignatures.begin();
  //unsigned long timeStamp = (unsigned long)time(NULL);
  /*TODO, decide whats active*/
  for(;it != m_currentlyActiveSignatures.end(); it++)
  {
    if(sig == (*it).first)
      (*it).second--;
  }
}


Class* SignatureDB::GetClassByID(int id)
{
  std::string st = CheckClass(id);
  if(st == "")
    throw "Unknown Class";
  return new Class(st, id);
}


int SignatureDB::CheckClass(std::string name)
{
	std::vector<std::pair<std::string, int> > ::const_iterator it;
	for(it = m_classes.begin(); it != m_classes.end(); it++)
	{
		/*cerr<<(*it).first<<endl;*/
		if((*it).first.compare(name) == 0)
			return (*it).second;

	}
	return -1;
}

std::string SignatureDB::CheckClass(int id_c)
{
	std::vector<std::pair<std::string, int> > ::const_iterator it;
	for(it = m_classes.begin(); it != m_classes.end(); it++)
	{
		if((*it).second == id_c)
			return (*it).first;
	}
	return "";
}

int SignatureDB::GetElemIdByClass(int ClassID, int index)
{
	int id  = -1;
	int count = 0;
	for(unsigned int indizes = 0; indizes< m_index->CountChildren(); indizes++)
	{
		XMLTag* sub_index = m_index->GetChild(indizes);
		if(m_index->GetName().compare(XML_NODE_SIGDB_INTERN_INDEX_CLASS2ID))
		{
			for(unsigned int i = 0; i < sub_index->CountChildren(); i++)
			{
				XMLTag* temp = sub_index->GetChild(i);
				if(temp->GetPropertyInt(XML_ATTRIBUTE_SIGDB_CLASS) == ClassID)
				{
					if(count != index)
					{
						count++;
					}
					else
					{
						id = temp->GetPropertyInt(XML_ATTRIBUTE_SIGDB_ID);
						break;
					}
				}
			}
		}
	}
	return id;
}

Signature* SignatureDB::GetSignatureByClass(int ClassID, int index)
{
	int id = GetElemIdByClass(ClassID, index);
	if(id == -1)
		return NULL;
	return GetSignatureByID(id);
}

XMLTag* SignatureDB::Save()
{
	XMLTag* tag = new XMLTag(XML_NODE_SIGNATUREDB);
	tag->AddChild(m_dbStarter->Clone());
	tag->AddChild(m_index->Clone());
	return tag;
}
// Accessor methods
//
XMLTag* SignatureDB::Query(std::string stQueryString)
{
	//TODO:
	int id = atoi(stQueryString.c_str());
	int index;
	XMLTag* tag = new XMLTag(XML_NODE_SIGNATURE_VEC);
	if(Check(id, index))
	{
		tag->AddChild(m_dbStarter->GetChild(index));
	}
	else
	{
		int classID = CheckClass(stQueryString);
		if(classID == -1)
			if(CheckClass(id).length() != 0)
				classID = id;
		int count = 0;
		int id_elem = -1;
		do
		{
			id_elem = GetElemIdByClass(classID, count);
			if(Check(id_elem, index))
			{
				tag->AddChild(m_dbStarter->GetChild(index));
				count++;
			}
		}
		while(id_elem != -1);
	}
	return tag;
}

bool SignatureDB::Check(int sigID, int& index) const
{
	size_t children = m_ids.size();
	for(unsigned int i = 0; i < children; i++)
	{
		if(m_ids[i] == sigID)
		{
			index = i;
			return true;
		}
	}
	index = -1;
	return false;
}

void SignatureDB::UpdateIDList()
{
	int children = m_dbStarter->CountChildren();
	m_ids.clear();
	for(int i = 0; i < children; i++)
	{
		Signature* sig = GetSignatureByIndex(i);
		if(sig != NULL)
		{
			m_ids.push_back(sig->m_ID);
			int indizes = m_index->CountChildren();
			for(int i = 0; i < indizes; i++)
			{
				XMLTag* sub_index = m_index->GetChild(i);
				if(sub_index->GetName().compare(XML_NODE_SIGDB_INTERN_INDEX_CLASS2ID)== 0)
				{
					int id = sig->m_ID;
					for(unsigned int classes = 0; classes < sig->CountClasses(); classes++)
					{
						Class* cl = sig->GetClass(classes);
						//TODO check if it exists already
						sub_index->AddChild((XMLTag*)new Class2ID(cl->m_ID, id));
						AddClass(cl->GetName(), cl->m_ID);
						printf("Added Class %s / %d\n", cl->GetName().c_str(), cl->m_ID);
					}
				}
			}
      FreeActiveSignature(sig);
		}
	}
}
// Other methods
//


