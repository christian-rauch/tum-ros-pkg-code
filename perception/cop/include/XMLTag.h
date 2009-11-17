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
                        XMLTag.h - Copyright klank


**************************************************************************/


#ifndef XMLTAG_H
#define XMLTAG_H

//#include "RelPoseFactory.h"
#include "Signature.h"
#include "AlgorithmEval.h"
#include "Camera.h"
#include <map>

typedef struct _xmlTextWriter xmlTextWriter;
typedef struct _xmlTextReader xmlTextReader;
typedef struct _xmlDoc xmlDoc;
typedef unsigned char xmlChar;
typedef struct _xmlNode xmlNode;

template<typename T> class AlgorithmEval;

#define XML_NODE_STD_VECTOR "std_vector"
#define XML_NODE_STD_PAIR "std_pair"
#define XML_NODE_INT "int"
#define XML_NODE_DOUBLE "double"
#define XML_NODE_STRING "std_string"
#define XML_NODE_MATRIX "newmat_Matrix"

#define XML_ATTRIBUTE_ROWS "rows"
#define XML_ATTRIBUTE_COLS "cols"

/**********************************************************************************************/
/** class XMLTag    intern class for handling xml transfer
 **********************************************************************************************
 * @brief stores data, to files or memory.
 * Contains functionality to read from files and create any xml representation using libxml2
 *
 **********************************************************************************************
 *
 **********************************************************************************************/
class XMLTag
{
public:

	// Constructors/Destructors
	//


	/**
	* Constructor, sets the tag name
	*/
	XMLTag ( std::string Name);

	/**
	* Empty Destructor
	*/
	virtual ~XMLTag ( );


	///
	/// Methods
	///
/****************************************************************************/
/** WriteToFile: Writes an XMLTag to an File
 ****************************************************************************
 *
 * Writes an XMLTag to an File, can return a XMLTag* that refers the file.
 * Such a file reference will be resolved when it is a child of another node
 * and GetChild is called
 *
 * @param     stFile      absolut filename
 *
 * @paramo    fileReference	a pointer that receives the pointer to the new
 *   						file reference tag
 *
 * @param                    error code
 *
 *
 *****************************************************************************
 *
 * \remark              file name will not be checked
 *
 *****************************************************************************/
	void WriteToFile(const std::string& stFile, XMLTag** fileReference = NULL) const;
/****************************************************************************/
/** SetCData: sets the current content of the tag
 ****************************************************************************
 *
 * sets the current content of a tag
 *
 * @param     value      content of the tag
 * @remarks this function can be only called once by XMLTag before again
 *          FreeAfterWriteToString must be called.
 *
 *****************************************************************************
 *
 *
 *****************************************************************************/
    char* WriteToString();
    void FreeAfterWriteToString();

/****************************************************************************/
/** ReadFromFile: Creates a new XMLTag from a file
 ****************************************************************************
 *
 * As constructor this would have had the same function signature as the std. constructor
 * so its a static function
 *
 *
 * @param     stFile      absolut filename
 *
 *
 *
 *****************************************************************************
 *
 * \remark              can return NULL if the file does not contain a tag
 *
 *****************************************************************************/
	static XMLTag* ReadFromFile(const std::string &stFile);
/****************************************************************************/
/** Clone: clones the current tag and returns it
 ****************************************************************************
 *
 * clone includes recursive copying of all children
 *
 *****************************************************************************
 *
 *
 *****************************************************************************/
	XMLTag* Clone() const;

/****************************************************************************/
/** Write: helping function for WriteToFile
 ****************************************************************************
 *
 * Writes an XMLTag to an File,
 *
 *
 *
 * @param     pWriter			pointer to the xmlWriter
 *
 *
 *****************************************************************************
 *
 *
 *****************************************************************************/
	virtual void Write(xmlTextWriter* pWriter) const;
/****************************************************************************/
/** Read: Reads an XMLTag from a readeer
 ****************************************************************************
 *
 * Supporting function for ReadFromFile
 *
 * @param     pReader			pointer to an libxml2 xmlTextReader
 *
 *
 *
 *****************************************************************************
 *
 *
 *****************************************************************************/
	static XMLTag* Read(xmlTextReader* pReader);
/****************************************************************************/
/** Tag: creates XMLTags from standard types, like vector pair, int, double, Elem
 ****************************************************************************
 *	@param T	contains the element that should be saved in the tag
 *	@param name defines the name of the node,
 *				there are standard names for every implemented tag
 *****************************************************************************
 * \remarks pairs and vectors of any supported Type T are allowed, too
 *
 *****************************************************************************/

	template<typename T> static XMLTag* Tag(std::vector<T> vector, std::string name = "")
	{
		XMLTag* tag = new XMLTag(name.compare("") == 0 ? XML_NODE_STD_VECTOR : name);
#ifdef WIN32
		std::vector<T>::const_iterator iter;
		for(iter = vector.begin(); iter != vector.end(); iter++)
		{
			tag->AddChild(XMLTag::Tag((*iter)));
		}
#else
		for(unsigned int i = 0; i < vector.size(); i++)
		{
			tag->AddChild(XMLTag::Tag(vector[i]));
		}
#endif
		return tag;
	}
	template<typename T1, typename T2> static XMLTag* Tag(std::pair<T1, T2> p, std::string name = "")
	{
		XMLTag* tag = new XMLTag(name.compare("") == 0 ? XML_NODE_STD_PAIR : name);
		tag->AddChild(XMLTag::Tag(p.first));
		tag->AddChild(XMLTag::Tag(p.second));
		return tag;
	}

	template<typename T3> static XMLTag* Tag(AlgorithmEval<T3> eval, std::string name = "")
	{
		return eval.Save();
	}


	static XMLTag* Tag(int n, std::string name = "");
	static XMLTag* Tag(double d, std::string name = "");
	static XMLTag* Tag(std::string value, std::string name = "");
	static XMLTag* Tag(Elem* elem, std::string name = "");
	static XMLTag* Tag(Algorithm<double>* alg, std::string name = "");
  static XMLTag* Tag(Algorithm<std::vector<RelPose* > >* alg, std::string name = "");
	static XMLTag* Tag(Algorithm<Descriptor*>* alg, std::string name = "");
	static XMLTag* Tag(Camera* alg, std::string name = "");
	static XMLTag* Tag(const Matrix& alg, std::string name = "");


/****************************************************************************/
/** Load: Load a Type T from a XMLTag
 ****************************************************************************
 *
 * Supported Types are for example: vector, pair, double, int, string, Matrix
 *									Elem, Algorithm, Camera
 *
 * @param     tag			the tag that was read from a file or created with a Tag function
 * @param		T*			a pointer that specifies the wished class (necessary for resolving the template)
 *
 * @throws char* with an error message in case of failure
 *****************************************************************************
 *
 *
 *****************************************************************************/
	template<typename T>
	static T Load(XMLTag* tag, T* t )
	{
		throw "Load Failed: Expected type has no loading function implemented";
	};

	template<typename T>
	 static std::vector<T> Load(XMLTag* tag, std::vector<T>*  )
	{
		std::vector<T> vectorTemp;
		for(unsigned int i = 0; i < tag->CountChildren(); i++)
		{
			try
			{
				vectorTemp.push_back(Load(tag->GetChild(i), (T*)NULL));
			}
			catch(char const* text)
			{
        printf("Error reading a vector: %s\n", text);
				/* Do nothing, try to get the others*/
			}
			catch(...)
			{
				printf("Error reading a vector.\n");
				/* Do nothing, try to get the others*/
			}
		}
		return vectorTemp;
	};

	template<typename T1, typename T2>
	static std::pair<T1, T2> Load(XMLTag* tag, std::pair<T1, T2>*  )
	{
		std::pair<T1, T2> p;
		int childCount = tag->CountChildren();
		if(childCount > 0)
			p.first = Load(tag->GetChild(0), (T1*)NULL);
		if(childCount > 1)
			p.second = Load(tag->GetChild(1), (T2*)NULL);

		return p;
	};

	template<typename T3> static AlgorithmEval<T3> Load(XMLTag* tag, AlgorithmEval<T3>*)
	{
		return AlgorithmEval<T3>(tag);
	}

	static int Load(XMLTag* tag, int* )
	{
		if(tag == NULL)
			return 0;
		return  tag->GetCDataInt();
	};

	static double Load(XMLTag* tag,double *)
	{
		if(tag == NULL)
			return 0.0;
		return tag->GetCDataDouble();
	};

	static std::string Load(XMLTag* tag, std::string *)
	{
		if(tag == NULL)
			return "";
		return tag->GetCDataST();
	};

	static Elem* Load(XMLTag* tag, Elem**  )
	{
		if(tag == NULL)
			return NULL;
		return Elem::ElemFactory(tag);
	};

  static Algorithm<std::vector<RelPose*> >* Load(XMLTag* tag, Algorithm<std::vector<RelPose*> >** )
	{
		if(tag == NULL)
			return NULL;
		return (Algorithm<std::vector<RelPose*> >*)LocateAlgorithm::LocAlgFactory(tag);
	};

	static Algorithm<double>* Load(XMLTag* tag, Algorithm<double>** )
	{

		if(tag == NULL)
			return NULL;
		return (Algorithm<double>*)ProveAlgorithm::ProveAlgFactory(tag);
	};

	static Algorithm<Descriptor*>* Load(XMLTag* tag, Algorithm<Descriptor*>** )
	{
		if(tag == NULL)
			return NULL;
		return (Algorithm<Descriptor*>*)RefineAlgorithm::RefineAlgFactory(tag);
	};


	static Camera* Load(XMLTag* tag, Camera** )
	{
		if(tag == NULL)
			return NULL;
		return Camera::CamFactory(tag);
	};

	/*static SemMapElement* Load(XMLTag* tag, SemMapElement** )
	{
		if(tag == NULL)
			return NULL;
		return SemMapElement::SemMapElementFactory(tag);
	};*/


	static Matrix Load(XMLTag* tag, Matrix* );

/****************************************************************************/
/** AddProperty: adds a property as an attribute to the current tag
 ****************************************************************************
 *
 * adds a property as an attribute to the current tag
 *
 * @param     name      name of the property, works as a key
 *
 * @param    value			value of the property
 *
 *
 *****************************************************************************
 *
 *
 *****************************************************************************/
	void AddProperty(const std::string& name, const std::string& value);
    void AddProperty(const std::string &name, const unsigned long &value);
	void AddProperty(const std::string& name, const int &value);
	void AddProperty(const std::string& name, const double &value);

/****************************************************************************/
/** SetCData: sets the current content of the tag
 ****************************************************************************
 *
 * sets the current content of a tag
 *
 * @param     value      content of the tag
 *
 *
 *****************************************************************************
 *
 *
 *****************************************************************************/
	void SetCData(const int &value);
	void SetCData(const double &value);
	void SetCData(const std::string &value);

/****************************************************************************/
/** GetCData*: returns the current content
 ****************************************************************************
 *
 * returns the current content
 *
 *
 * @param		the content or empty("", 0, 0.0)
 *
 *
 *****************************************************************************
 *
 *
 *****************************************************************************/
	int			GetCDataInt() const;
	double		GetCDataDouble() const;
	std::string GetCDataST() const;


/****************************************************************************/
/** GetProperty*: returns the value of a property
 ****************************************************************************
 *
 * Returns the current value of a property
 *
 * @param     name      name of the property
 *
 *
 * @param                    the value or empty ("", 0, 0.0)
 *
 *
 *****************************************************************************
 *
 *
 *****************************************************************************/
    std::string GetProperty(const std::string& name, std::string defaultValue = "");
	double GetPropertyDouble(const std::string& name, double defaultValue = 0.0);
	int GetPropertyInt(const std::string& name, int defaultValue = 0);


/****************************************************************************/
/** AddChild: adds a child node to a tag
 ****************************************************************************
 *
 * Adds a child node to a tag, the position where to put it can be influenced
 * if the position is larger or smaller than the possible range the element
 *  will be put at the end of the list.
 *
 *
 * @param     child		pointer to an existing tag, a null child will not be appended
 *
 * @param     position		position where to put the cild in the child list
 *
 * @param                 final position that was selected, or -1
 *
 *
 *****************************************************************************
 *
 * \remark              child = NULL will not be appended and -1 will be returned
 *
 *****************************************************************************/
	int AddChild(XMLTag* child, int position = -1);
	void ReplaceChild(XMLTag* child, int position);
/****************************************************************************/
/** CountChildren: counts the childdren-tags of the current tag
 ****************************************************************************
 *
 * Counts the children
 *
 *
 * @param                 number of children
 *
 *
 *****************************************************************************
 *
 *
 *****************************************************************************/
	unsigned int CountChildren() const;
/****************************************************************************/
/** RemoveChild: removes an XMLTag* from the list
 ****************************************************************************
 *
 * Adds a child node to a tag, the position where to put it can be influenced
 * if the position is larger or smaller than the possible range the element
 *  will be put at the end of the list.
 *
 *
 * @param     position		position where to erase a cild
 *
 *
 *
 *****************************************************************************
 *
 * \remark              the tag will not be deleted!
 *
 *****************************************************************************/
	void RemoveChild(const unsigned int &position);
	void RemoveChild(const std::string &name);

/****************************************************************************/
/** GetChild: returns a child tag
 ****************************************************************************
 *
 * returns a child tag
 *
 * @param position/name   descriptor for the child
 *
 * @param		a  child tag
 *
 *
 *****************************************************************************
 *
 *
 *****************************************************************************/
	XMLTag* GetChild(const unsigned int &position);
	XMLTag* GetChild(const std::string &name);
	XMLTag* GetChild(const std::string& name, int innerindex);

/****************************************************************************/
/** GetName: returns the name of a tag
 ****************************************************************************
 *
 *  The name of the tag (<name></name>)
 *
 *
 *
 *****************************************************************************
 *
 *
 *****************************************************************************/
	std::string& GetName(){return m_name;}

	//static public Methods
	static unsigned int OldTag()
	{
		return 0;
	}

	unsigned long date();
	//
	//Methods
	//
	int getQueryType();


	protected:


	private:
		/**
		*	finction to reset the timestamp
		*/
		void Touch();

      /****************************************************************************/
      /** DocWrite: adds the content to a memory buffer
       ****************************************************************************
       *
       * writes a tag to the given node
       *
       * @param n  bnode
       *
       *****************************************************************************
       *
       *
       *****************************************************************************/
        void DocWrite(xmlNode* n) const;

	// Private attributes
	//
	unsigned long							m_lastChanged;
	std::vector<XMLTag*>					m_children;
	std::string								m_cData;
	std::string								m_name;
	std::map<std::string, std::string>		m_properties;

    xmlDoc*  m_doc;
    xmlChar *m_xmlbuff;
};

#endif // XMLTAG_H
