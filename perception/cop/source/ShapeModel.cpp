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
                        ShapeModel.cpp - Copyright klank

**************************************************************************/

#include "ShapeModel.h"
#include "Camera.h"
#include "DxfReader.h"
#include "DxfWriter.h"
#ifdef HALCONIMG
#include "cpp/HalconCpp.h"
#include "SearchParams3d.h"
#endif


#ifdef BOOST_THREAD
#ifdef BOOST_1_35
#include "boost/date_time/posix_time/posix_time.hpp"
#else
#include <boost/thread/detail/lock.hpp>
#include <boost/bind.hpp>
#endif
#define BOOST(A) A
#else
#define BOOST(A) ;
#endif

// Constructors/Destructors
//
#define XML_NODE_SHAPEMODEL_INTERN_CONFIG "ShapeModelConfig"
#define XML_NODE_SEMANTICMAP "model:SemanticMap"
#define XML_NODE_SHAPEMODELPARAMSET "ShapeModelParamSet"
#define XML_ATTRIBUTE_NAME "name"
#define XML_SHAPEMODEL_INTERN_SM3 "sm3file"
#define XML_SHAPEMODEL_INTERN_DXF "dxffile"

#define XML_NODE_LONGMIN  "longMin"
#define XML_NODE_LONGMAX  "longMax"
#define XML_NODE_LATMIN    "latMin"
#define XML_NODE_LATMAX    "latMax"
#define XML_NODE_CRMIN    "crMin"
#define XML_NODE_CRMAX    "crMax"
#define XML_NODE_REFROTX  "refrotX"
#define XML_NODE_REFROTY  "refrotY"
#define XML_NODE_REFROTZ  "refrotZ"
#define XML_NODE_DISTMIN  "distMin"
#define XML_NODE_DISTMAX  "distMax"
#define XML_NODE_MEASURE  "Measure"
#define XML_NODE_SCALE    "cam_scale"
#define XML_NODE_MINCONTRAST "minContrast"
#define XML_NODE_MINFACEANGLE "minFaceAngle"
#define XML_ATTRIBUTE_INITIALIZATIONLEVEL "initLevel"

using namespace cop;


XMLTag* ShapeModelParamSet::Save()
{
  XMLTag* tag = new XMLTag(XML_NODE_SHAPEMODELPARAMSET);
  tag->AddChild(XMLTag::Tag(m_longitudeMin, XML_NODE_LONGMIN));
  tag->AddChild(XMLTag::Tag(m_longitudeMax, XML_NODE_LONGMAX));
  tag->AddChild(XMLTag::Tag(m_latitudeMin,  XML_NODE_LATMIN ));
  tag->AddChild(XMLTag::Tag(m_latitudeMax,  XML_NODE_LATMAX ));
  tag->AddChild(XMLTag::Tag(m_camRollMin,   XML_NODE_CRMIN  ));
  tag->AddChild(XMLTag::Tag(m_camRollMax,   XML_NODE_CRMAX  ));
  tag->AddChild(XMLTag::Tag(m_initRefX,     XML_NODE_REFROTX));
  tag->AddChild(XMLTag::Tag(m_initRefY,     XML_NODE_REFROTY));
  tag->AddChild(XMLTag::Tag(m_initRefZ,     XML_NODE_REFROTZ));
  tag->AddChild(XMLTag::Tag(m_distMin,      XML_NODE_DISTMIN));
  tag->AddChild(XMLTag::Tag(m_distMax,      XML_NODE_DISTMAX));
  tag->AddChild(XMLTag::Tag(m_measure,      XML_NODE_MEASURE));
  tag->AddChild(XMLTag::Tag(m_scale,        XML_NODE_SCALE));
  tag->AddChild(XMLTag::Tag(m_minContrast,  XML_NODE_MINCONTRAST));
  tag->AddChild(XMLTag::Tag(m_minFaceAngle, XML_NODE_MINFACEANGLE));

  if(m_region != NULL)
  {
    tag->AddChild(m_region->Save(XML_NODE_ROI));
  }
#ifdef _DEBUG
  else
    printf("RegionOI that should be saved is NULL!\n");
#endif
  if(m_calib != NULL)
  {
    XMLTag* calib = new XMLTag(XML_NODE_CALIBRATION);
    m_calib->SaveTo(calib);
    tag->AddChild(calib);
  }
  return tag;

}

ShapeModelParamSet::ShapeModelParamSet(XMLTag* tag, Calibration* calib, bool deleteCalib) :
    m_region(NULL),
    m_deleteCalib(deleteCalib),
    m_gravPointInited(false)
{
  if(tag != NULL)
  {
    m_longitudeMin  = XMLTag::Load(tag->GetChild(XML_NODE_LONGMIN),&m_longitudeMin  );
    m_longitudeMax  = XMLTag::Load(tag->GetChild(XML_NODE_LONGMAX),&m_longitudeMax  );
    m_latitudeMin  = XMLTag::Load(tag->GetChild(XML_NODE_LATMIN ),&m_latitudeMin  );
    m_latitudeMax  = XMLTag::Load(tag->GetChild(XML_NODE_LATMAX ),&m_latitudeMax  );
    m_camRollMin  = XMLTag::Load(tag->GetChild(XML_NODE_CRMIN   ),&m_camRollMin  );
    m_camRollMax  = XMLTag::Load(tag->GetChild(XML_NODE_CRMAX   ),&m_camRollMax  );
    m_initRefX    = XMLTag::Load(tag->GetChild(XML_NODE_REFROTX),&m_initRefX    );
    m_initRefY    = XMLTag::Load(tag->GetChild(XML_NODE_REFROTY),&m_initRefY    );
    m_initRefZ    = XMLTag::Load(tag->GetChild(XML_NODE_REFROTZ),&m_initRefZ    );
    m_distMin    = XMLTag::Load(tag->GetChild(XML_NODE_DISTMIN),&m_distMin    );
    m_distMax    = XMLTag::Load(tag->GetChild(XML_NODE_DISTMAX),&m_distMax    );
    if(tag->GetChild(XML_NODE_MEASURE) != NULL) /* added later*/
    {
      m_measure  = XMLTag::Load(tag->GetChild(XML_NODE_MEASURE),&m_measure);
      if(m_measure == 0.0)
      {
        m_measure = 0.01;
      }
    }
    else
      m_measure = 0.01;
    if(tag->GetChild(XML_NODE_SCALE) != NULL)
      m_scale    = XMLTag::Load(tag->GetChild(XML_NODE_SCALE  ),&m_scale);
    else
      m_scale = 1.0;
    if(tag->GetChild(XML_NODE_MINCONTRAST) != NULL)
      m_minContrast = XMLTag::Load(tag->GetChild(XML_NODE_MINCONTRAST), &m_minContrast );
    else
      m_minContrast = 10;
    if(tag->GetChild(XML_NODE_MINFACEANGLE) != NULL)
      m_minFaceAngle = XMLTag::Load(tag->GetChild(XML_NODE_MINFACEANGLE), &m_minFaceAngle);
    else
      m_minFaceAngle = 0.3;
    XMLTag* tagTemp = tag->GetChild(XML_NODE_ROI);
    if(tagTemp != NULL)
    {
      std::string stFileName = tagTemp->GetProperty("Filename", "");
      if(stFileName.length() == 0)
      {
        RegionRuns type;
        m_region = new RegionOI(XMLTag::Load(tagTemp, &type));
      }
      else
      {
        m_region = new RegionOI(stFileName);
      }
#ifdef _DEBUG
      printf("ROI Loaded! (Size: %d)\n", m_region->GetSize());
#endif /*_DEBUG*/
    }
    else
    {
#ifdef _DEBUG
      printf("No Region contained in the XML, Check the saving!\n");
#endif /*_DEBUG*/
      m_region = NULL;
    }
    XMLTag* tagTemp2 = tag->GetChild(XML_NODE_CALIBRATION);
    if(tagTemp2 != NULL)
    {
      m_calib = new Calibration(tagTemp2);
      m_deleteCalib = true;
    }
    else
    {
      m_calib = calib;
      if(calib == NULL)
        printf("No Calibration provided\n");
    }

  }
  else
  {
#ifdef _DEBUG
    printf("Norm Values for Shape model activated...\n\n");
#endif /*_DEBUG*/
    m_longitudeMin = -3.1415;
    m_longitudeMax = 3.1415;
    m_latitudeMin = -3.1415;
    m_latitudeMax = 3.1415;
    m_camRollMin = -3.1415;
    m_camRollMax = 3.1415;
    m_initRefX = 0;
    m_initRefY = 0;
    m_initRefZ = 0;
    m_distMin = 0.6;
    m_distMax = 0.8;
    m_measure = 0.01;
    m_scale = 1.0;
    m_minContrast  = 10;
    m_minFaceAngle = 0.3;
    m_region = NULL;
  }
}


ShapeModelParamSet::ShapeModelParamSet(Calibration* calib, double lgmi, double lgma,double ltmi , double ltma,double crmi, double crma,double dimi, double dima , double refRoX, double refRoY, double refRoZ, double measure, double scale, int minContrast , double minFaceAngle ) :
    m_longitudeMin(lgmi),
    m_longitudeMax(lgma),
    m_latitudeMin(ltmi),
    m_latitudeMax(ltma),
    m_camRollMin(crmi),
    m_camRollMax(crma),
    m_initRefX(refRoX),
    m_initRefY(refRoY),
    m_initRefZ(refRoZ),
    m_distMin(dimi),
    m_distMax(dima),
    m_measure(measure),
    m_scale(scale),
    m_minContrast(minContrast),
    m_minFaceAngle(minFaceAngle),
    m_region(NULL),
    m_calib(calib),
    m_deleteCalib(false),
    m_gravPointInited(false)
    {
      m_gravPointInited= false;
    }

ShapeModel::ShapeModel (Class* classref ) :
  Descriptor(classref),
  m_initializationLEvel(1.0),
  m_curIndex(-1),
  m_initialized(false),
  m_showNow(true)
{
}

ShapeModel::ShapeModel (XMLTag* tag) :
  Descriptor(tag),
  m_initializationLEvel(1.0),
  m_curIndex(-1),
  m_initialized(false),
  m_showNow(true)
{
  printf("Loading Shape Model\n");
  bool deleteTold = false;
  if(tag != NULL)
  {
    m_initializationLEvel = tag->GetPropertyDouble(XML_ATTRIBUTE_INITIALIZATIONLEVEL, 1.0);
    XMLTag* conf = tag->GetChild(XML_NODE_SHAPEMODEL_INTERN_CONFIG);
    Calibration* calib = NULL;
    if(conf != NULL)
    {
      calib = new Calibration(conf);
      deleteTold = true;
    }
    /*Compatibility with old files*/
    XMLTag* tag_dxf = tag->GetChild(XML_SHAPEMODEL_INTERN_DXF);
    XMLTag* tag_ps = tag->GetChild(XML_NODE_SHAPEMODELPARAMSET);

    if(tag_dxf != NULL && tag_ps !=  NULL)
    {
      try
      {
      /* if we have an old xml that does not say about, if it created its shape model, we are assuming, */
      /* that it was not created and we are passing false to shapeModelCreated*/
        bool shapeModelCreated = false;
        std::string stFileName = tag_dxf->GetProperty(XML_ATTRIBUTE_FILENAME);
        SetShapeModelParamSet(new ShapeModelParamSet(tag_ps, calib, deleteTold), stFileName, shapeModelCreated);
        deleteTold = false;
      }
      catch(...)
      {
        printf("ShapeModel: Error removing config entries\n");
      }
    }

    XMLTag* tag_l_ps = tag->GetChild(XML_NODE_SHAPEPARAM_LIST);
    printf("Loaded tag %s: %p with length %d\n", XML_NODE_SHAPEPARAM_LIST,  tag_l_ps,  tag_l_ps != NULL ?  tag_l_ps->CountChildren() : 0);
    if(tag_l_ps != NULL)
    {
      const unsigned int &num = tag_l_ps->CountChildren();
      for(unsigned int i = 0; i < num; i++)
      {
        try
        {
          XMLTag* child = tag_l_ps->GetChild(XML_NODE_SHAPEPARAM_ENTRY, i);
          printf("Loaded tag %s: %p with length %d\n", XML_NODE_SHAPEPARAM_ENTRY,  child,  child != NULL ?child->CountChildren() : 0);
          if(child == NULL)
            break;
          std::string temp; /*Hack cause ShapeModelParamSet can not be loaeded by XMLTag (lazy TODO)**/
          int temp_b; 				/*Hack cause ShapeModelParamSet can not be loaeded by XMLTag (lazy TODO)**/
          bool btest = XMLTag::Load(child->GetChild(2), &temp_b) != 0;
          temp = XMLTag::Load(child->GetChild(1), &temp);
          SetShapeModelParamSet(new ShapeModelParamSet(
                                        child->GetChild(XML_NODE_SHAPEMODELPARAMSET),
                                        calib, deleteTold),
                                temp,
                                btest);
           printf("With filename %s\n", temp.c_str());
           deleteTold = false;
        }
        catch(...)
        {
          printf("ShapeModel: Error Reading List of shape entries\n");
        }
      }
    }
    else
    {
      if(conf != NULL)/*Old xml file, in here for downwards compatibility*/
      {
      /* if we have an old xml that does not say about, if it created its shape model, we are assuming, */
      /* that it was not created and we are passing false to shapeModelCreated*/
        bool shapeModelCreated = false;
        XMLTag* dxf = conf->GetChild(XML_SHAPEMODEL_INTERN_DXF);
        SetShapeModelParamSet(new ShapeModelParamSet(
                                tag->GetChild(XML_NODE_SHAPEMODELPARAMSET), calib, deleteTold),
                              dxf->GetProperty(XML_ATTRIBUTE_FILENAME), shapeModelCreated);
        deleteTold = false;
      }
    }
    if(deleteTold)
      delete calib;
  }
  else
  {
    printf("ShapeModel: Trying to load empty tag\n");
  }
}

#ifdef HALCONIMG
void ShapeModel::WriteShapeModelThreaded(long shapeModelId, const char* name)
{
  try
  {
    Halcon::write_shape_model_3d(shapeModelId, name);
  }
  catch( Halcon::HException ex )
  {
      m_shapeParams_file[m_curIndex].second.second = false;
      printf("ShapeModel (Writer Thread): %s\n", ex.message);
  }
}
#endif

long ShapeModel::GetShapeModel(double &scale)
{
  scale = 1.0;
  if(m_curIndex != -1)
  {
    printf("Model existing as file: %s\n", m_shapeParams_file[m_curIndex].second.second ? "yes" : "no");
  }
  if(m_initializationLEvel < 0.1)
  {
      return -1;
  }

  if(m_initialized)
  {
    scale = m_shapeParams_file[m_curIndex].first->m_scale;
    return m_ShapeModelID;
  }
  else if (m_curIndex != -1 && !m_shapeParams_file[m_curIndex].second.second)
  {
#ifdef HALCONIMG
    try
    {
      Halcon::HTuple obj, num, empty;
      ShapeModelParamSet& sm = *m_shapeParams_file[m_curIndex].first;
      scale = sm.m_scale;
      std::string stFilename = m_shapeParams_file[m_curIndex].second.first;
#ifdef _DEBUG
      printf("ShapeModel must be created with params:\nRef: %f,%f,%f\nlon: %f,%f\nlat %f, %f\ncamroll %f, %f\ndist %f, %f\nmincontrast %d\nminfaceangle %f\n",
                  sm.m_initRefX,sm.m_initRefY, sm.m_initRefZ,
                  sm.m_longitudeMin, sm.m_longitudeMax,
                  sm.m_latitudeMin, sm.m_latitudeMax,
                  sm.m_camRollMin, sm.m_camRollMax,
                  sm.m_distMin, sm.m_distMax,
                  sm.m_minContrast,sm.m_minFaceAngle);
        //sm.m_measure = 0.0001;
      printf("read_object_model_3d_dxf(%s, %f, ...)\n",stFilename.c_str(), sm.m_measure);

#endif /*_DEBUG*/
      Halcon::read_object_model_3d_dxf(stFilename.c_str(), sm.m_measure,empty,empty, &obj, &num);
      if(sm.m_distMin <= 0.000001)
      {
        printf("Distance input invalid. Inventing values: Min 1m Max 1.2m");
        sm.m_distMin = 1.0;
        sm.m_distMax = 1.2;
      }
      Halcon::create_shape_model_3d(obj,
                                    sm.m_calib->CamParam(scale),
                                    sm.m_initRefX,sm.m_initRefY, sm.m_initRefZ,"gba",
                                    sm.m_longitudeMin, sm.m_longitudeMax,
                                    sm.m_latitudeMin, sm.m_latitudeMax,
                                    sm.m_camRollMin, sm.m_camRollMax,
                                    sm.m_distMin, sm.m_distMax,
                                    sm.m_minContrast,
                                    HTuple("min_face_angle").Concat("model_tolerance"), HTuple(sm.m_minFaceAngle).Concat(0),
                                    &m_ShapeModelID);
#ifdef _DEBUG
      printf("Created ShapeModel: Index: %d / Handle %ld\n", m_curIndex, m_ShapeModelID);
#endif /*_DEBUG*/
      try
      {
#ifdef BOOST_THREAD
          boost::thread(boost::bind(&ShapeModel::WriteShapeModelThreaded, this, m_ShapeModelID, (const char*)GenShapeModelFileName(stFilename, m_curIndex).c_str()));
#else
          Halcon::write_shape_model_3d(m_ShapeModelID , GenShapeModelFileName(stFilename, m_curIndex).c_str());
#endif
          m_shapeParams_file[m_curIndex].second.second = true;
      }
      catch( Halcon::HException ex )
      {
          m_shapeParams_file[m_curIndex].second.second = false;
          printf("ShapeModel: %s\n", ex.message);
      }
      m_initialized = true;
    }
    catch(Halcon::HException ex)
    {
      printf("ShapeModel: %s\n", ex.message);
      throw "Object Model File could not be found";
    }
#endif /*HALCONIMG*/
  }
  else
  {
    if(m_curIndex == -1 || m_curIndex > (signed)m_shapeParams_file.size())
      throw "Shape Model could not be loaded, cause of missing initialization";
    scale = m_shapeParams_file[m_curIndex].first->m_scale;
#ifdef HALCONIMG
    try
    {
#ifdef BOOST_1_35
                  BOOST(boost::system_time t0);
                  BOOST(boost::system_time t1);
                   BOOST(t0 = boost::get_system_time());
#else
                  BOOST(boost::xtime t0);
                  BOOST(boost::xtime t1);
                   BOOST(boost::xtime_get(&t0, boost::TIME_UTC));
#endif
              Halcon::read_shape_model_3d(GenShapeModelFileName(m_shapeParams_file[m_curIndex].second.first,
                                          m_curIndex).c_str(), &m_ShapeModelID);

#ifdef BOOST_1_35
                    BOOST(t1 = get_system_time());
                    BOOST(boost::posix_time::time_duration td = t1 - t0);
                    BOOST(printf("Model load time: %s\n", boost::posix_time::to_simple_string(td).c_str()));

#ifdef _DEBUG
/*                    std::cout << "Calc time: " << t1- t0 << std::endl;*/
#endif/* _DEBUG*/
#else /*BOOST_1_35*/
                    BOOST(boost::xtime_get(&t1, boost::TIME_UTC));
                    BOOST(printf("Nodel load time: %ds %dms\n", t1.sec - t0.sec, (t1.nsec - t0.nsec) / 1000000));
#endif  /*BOOST_1_35*/
      m_initialized = true;
    }
    catch(Halcon::HException )
    {
#ifdef _DEBUG
      printf("Tried to read %s but failed\n", GenShapeModelFileName(m_shapeParams_file[m_curIndex].second.first,
                                        m_curIndex).c_str());
#endif /* _DEBUG*/
      try
      {
        Halcon::HTuple obj, num, empty;
        ShapeModelParamSet& sm = *m_shapeParams_file[m_curIndex].first;
        scale = sm.m_scale;
        std::string stFilename = m_shapeParams_file[m_curIndex].second.first;
#ifdef _DEBUG
        printf("ShapeModel must be created with params:\nRef: %f,%f,%f\nlon: %f,%f\nlat %f %f\ncamroll %f %f\ndist %f %f\nmincontrast %d\nminfaceangle %f\n",
                    sm.m_initRefX,sm.m_initRefY, sm.m_initRefZ,
                    sm.m_longitudeMin, sm.m_longitudeMax,
                    sm.m_latitudeMin, sm.m_latitudeMax,
                    sm.m_camRollMin, sm.m_camRollMax,
                    sm.m_distMin, sm.m_distMax,
                    sm.m_minContrast,sm.m_minFaceAngle);
        //sm.m_measure = 0.0001;
        printf("read_object_model_3d_dxf(%s, %f, ...)\n",stFilename.c_str(), sm.m_measure);
#endif /* _DEBUG*/

        Halcon::read_object_model_3d_dxf(stFilename.c_str(), sm.m_measure,empty,empty, &obj, &num);
        if(sm.m_distMin <= 0.000001)
        {
          printf("Distance input invalid. Inventing values: Min 1m Max 1.2m");
          sm.m_distMin = 1.0;
          sm.m_distMax = 1.2;
        }
        Halcon::create_shape_model_3d(obj,
                                      sm.m_calib->CamParam(scale),
                                      sm.m_initRefX,sm.m_initRefY, sm.m_initRefZ,"gba",
                                      sm.m_longitudeMin, sm.m_longitudeMax,
                                      sm.m_latitudeMin, sm.m_latitudeMax,
                                      sm.m_camRollMin, sm.m_camRollMax,
                                      sm.m_distMin, sm.m_distMax,
                                      sm.m_minContrast,
                                      "min_face_angle", sm.m_minFaceAngle,
                                      &m_ShapeModelID);
#ifdef _DEBUG
        printf("Created ShapeModel: %d\n", m_curIndex);
#endif /*_DEBUG*/

        try
        {
#ifdef BOOST_THREAD
            boost::thread(boost::bind(&ShapeModel::WriteShapeModelThreaded, this, m_ShapeModelID, (const char*)GenShapeModelFileName(stFilename, m_curIndex).c_str()));
#else
            Halcon::write_shape_model_3d(m_ShapeModelID , GenShapeModelFileName(stFilename, m_curIndex).c_str());
#endif
            m_shapeParams_file[m_curIndex].second.second = true;
        }
        catch( Halcon::HException ex )
        {
            m_shapeParams_file[m_curIndex].second.second = false;
            printf("ShapeModel: %s\n", ex.message);
        }
        m_initialized = true;
      }
      catch(Halcon::HException ex)
      {
        printf("ShapeModel: %s\n", ex.message);
        throw "Object Model File could not be found";
      }
    }
#endif /*HALCONIMG*/
  }
  return m_ShapeModelID;
}

Mesh_t ShapeModel::GetMesh(RelPose* pose, double measure_adaption)
{
  ShapeModelParamSet& sm = *m_shapeParams_file[m_curIndex].first;
  int points;
  Matrix m = pose->GetMatrix();
  m.element(0, 3) *= measure_adaption;
  m.element(1, 3) *= measure_adaption;
  m.element(2, 3) *= measure_adaption;

  return ReadMesh(m_shapeParams_file[m_curIndex].second.first, sm.m_measure, m, points);
}


double ShapeModel::IntersectWithSupportingPlane(RelPose* planePose, RelPose* matchPose, RelPose* cameraPose, Calibration* calib)
{
  double eval = 1.0;
  double intersectX, intersectY,diffZmatch, diffZcam;
  printf("Involved Jlo Ids:\n  Cam: %ld (p:%ld)\n  Plane: %ld (p:%ld)\n  Match: %ld (p:%ld)\n", cameraPose->m_uniqueID, cameraPose->m_parentID, planePose->m_uniqueID, planePose->m_parentID, matchPose->m_uniqueID, matchPose->m_parentID);
  printf("Getting Camera and\n");
  RelPose* pose_rel_c = RelPoseFactory::GetRelPose(cameraPose->m_uniqueID, planePose->m_uniqueID);
  Matrix m_camera_in_plane = pose_rel_c->GetMatrix();
  RelPoseFactory::FreeRelPose(pose_rel_c);
  cout << m_camera_in_plane << endl;
  printf("Match in Plane Coordinates\n");
  RelPose* pose_rel_m = RelPoseFactory::GetRelPose(matchPose->m_uniqueID, planePose->m_uniqueID);
  Matrix m_match_in_plane = pose_rel_m->GetMatrix();
  RelPoseFactory::FreeRelPose(pose_rel_m);
  cout << m_match_in_plane << endl;
  intersectX = m_match_in_plane.element(0,3);
  intersectY = m_match_in_plane.element(1,3);
  diffZmatch = m_match_in_plane.element(2,3);
  diffZcam   = m_camera_in_plane.element(2,3);
  /*include offset of object to the plane by infering its most probable connection points.*/
  int points;
  printf("Transforming Mesh in Plane Coordinates\n");
  ShapeModelParamSet* sm = m_shapeParams_file[m_curIndex].first;
  Mesh_t mesh = ReadMesh(m_shapeParams_file[m_curIndex].second.first, sm->m_measure, m_match_in_plane, points);
  double x,y,z,h,w,d;
  CalcMeshCenterBox(mesh, x,y,z,h,w,d);
  diffZmatch -= d/2;
  printf("Distance descrpancy to supporting plane: %f\n", diffZmatch);
  /*Object too close => diffZMatch < 0 */
  /*Object too far away => diffZMatch > 0 */
#ifdef HALCONIMG
  try
  {
    Halcon::HTuple hommat, hommat_old,hommat_inv,hommat_pose, qx, qy, qz, r, c, campar, X, Y, hommat_new;
    planePose->GetHommat(&hommat, 0);
    campar = calib->CamParam(sm->m_scale);
    Halcon::hom_mat3d_invert(hommat, &hommat_inv);
    Halcon::hom_mat3d_to_pose(hommat, &hommat_pose);
    Halcon::hom_mat3d_translate_local(hommat, intersectX, intersectY, diffZmatch, &hommat_old);
    Halcon::project_3d_point(hommat_old[3], hommat_old[7], hommat_old[11], campar, &r, &c);
    printf("P (%f %f %f) proj to cam= (%f %f)\n",hommat_old[3].D(), hommat_old[7].D(), hommat_old[11].D(), r[0].D(), c[0].D());
    Halcon::image_points_to_world_plane(campar, hommat_pose, r, c, 1.0, &X, &Y);
    printf("Point on plane: %f %f \n", X[0].D(), Y[0].D());
    Halcon::hom_mat3d_translate_local(hommat, X, Y, 0, &hommat_new);
    printf("Plnae center in cam: %f %f %f\n", hommat[3].D(), hommat[7].D(), hommat[11].D());
    printf("New Obj center in cam: %f %f %f\n", hommat_new[3].D(), hommat_new[7].D(), hommat_new[11].D());
    printf("Old Obj center in cam: %f %f %f\n", hommat_old[3].D(), hommat_old[7].D(), hommat_old[11].D());
    matchPose->GetHommat(&hommat, 0);
    double scale = sqrt((hommat_new[3].D()*hommat_new[3].D())+(hommat_new[7].D()*hommat_new[7].D())*(hommat_new[11].D()*hommat_new[11].D()))
                   /
                   sqrt((hommat_old[3].D()*hommat_old[3].D())+(hommat_old[7].D()*hommat_old[7].D())*(hommat_old[11].D()*hommat_old[11].D()));
    sm->m_measure *= scale;
    printf("Scaling adapted to: %f\n", sm->m_measure);
    m_initializationLEvel += 0.1;
    hommat[3] = hommat[3].D() * scale;
    hommat[7] = hommat[7].D() * scale;
    hommat[11] = hommat[11].D() * scale;
    printf("Get cov\n");
    Matrix cov = matchPose->GetCovarianceMatrix();
    Halcon::HTuple cov_t(6);
    for(int i = 0; i < 6; i++)
    {
      cov_t[i] = cov.element(i,i) /= scale;
    }
    printf("FRelPose\n");
    matchPose = RelPoseFactory::FRelPose(hommat, cov_t, cameraPose, matchPose->m_uniqueID);
    printf("New corrected pose\n");
    matchPose->Print();
  }
  catch(Halcon::HException ex)
  {
    printf("Error: %s\n", ex.message);
  }
  catch(...)
  {
    printf("Error: in Intersect with plane\n");
  }
#endif
  return eval;
}



void ShapeModel::SaveTo(XMLTag* tag)
{
  Descriptor::SaveTo(tag);

  tag->AddProperty(XML_ATTRIBUTE_INITIALIZATIONLEVEL, m_initializationLEvel);

  XMLTag* tag_spl = new XMLTag(XML_NODE_SHAPEPARAM_LIST);
  for(unsigned int i = 0; i < m_shapeParams_file.size(); i++)
  {
    XMLTag* tag_child = new XMLTag(XML_NODE_SHAPEPARAM_ENTRY);
    tag_child->AddChild(m_shapeParams_file[i].first->Save());
    tag_child->AddChild(XMLTag::Tag(m_shapeParams_file[i].second.first));
    tag_child->AddChild(XMLTag::Tag((int)m_shapeParams_file[i].second.second));
    tag_spl->AddChild(tag_child);
  }
  tag->AddChild(tag_spl);
}

void ShapeModel::Show(RelPose* pose, Camera* cam)
{
#ifdef HALCONIMG
    if(m_showNow && pose != NULL)
    {
      Halcon::HTuple handle = 600;
       if(cam != NULL)
       {
        handle = cam->GetWindow()->WindowHandle();
       }

        Halcon::set_color(handle, "green");
        Halcon::set_line_width(handle, 4);
        Halcon::Hobject xld = GetContour(*pose, cam);
        Halcon::disp_xld(xld, handle);
        //ShapeModelParamSet* pm = GetParamSet();
        ShowRegion(cam);

    }
#endif /*HALCONIMG*/
}

void ShapeModel::ShowRegion(Camera* cam)
{
#ifdef HALCONIMG
    Halcon::HTuple winhandle = 600;
    if( cam != NULL)
    {
        HWindow* hwin = cam->GetWindow();
        winhandle = hwin->WindowHandle();
    }

    Halcon::set_line_width(winhandle, 2);
    ShapeModelParamSet* pm = GetParamSet();
    Halcon::set_draw(winhandle, "margin");
    Halcon::set_color(winhandle, "red");
    if(pm->m_region != NULL)
      Halcon::disp_region(pm->m_region->m_reg, winhandle);
#endif /*HALCONIMG*/
}

ShapeModel::~ShapeModel ( )
{
#ifdef HALCONIMG
  if(m_initialized)
    Halcon::clear_shape_model_3d(m_ShapeModelID);
#endif  /*HALCONIMG*/
  for(unsigned int i = 0; i < m_shapeParams_file.size(); i++)
    delete m_shapeParams_file[i].first;
}

bool ShapeModel::PoseToRange(RelPose* pose, ShapeModelParamSet &pm, double* gravPoint, Calibration* calib)
{
#ifdef HALCONIMG
  Halcon::HTuple pose_s(7,0.0), extents;
  bool cov = true;
  Matrix m,ExtremePoses;
#ifdef _DEBUG
  pose->Print();
  printf("Calculating new Search Ranges\n");
#endif /*_DEBUG*/
  try
  {
      if(pose->m_uniqueID != ID_WORLD)
      {
          Matrix dings = pose->GetMatrix();
          if(dings.element(0,0) >= 0.9999 &&
            dings.element(1,1) >= 0.9999 &&
            dings.element(2,2) >= 0.9999 &&
            fabs(dings.element(0,3)) <= 0.00001 &&
            fabs(dings.element(1,3)) <= 0.00001 &&
            fabs(dings.element(2,3)) <= 0.00001
           )
          {
            printf("Identity transform, no sense to calc search spaces.\n");
            return false;
          }
          m = pose->GetCovarianceMatrix();
          double d = m.trace();
          if (d == 0.0)
          {
            printf("ShapeModel::PoseToRange: Ignoring covariances with trace = 0\n");
            return false;
          }
      }
      else
      {
          m = pose->GetCovarianceMatrix();
      }
  }
  catch(...)
  {
      printf("Error getting cov for this Relation: %ld\n", pose->m_parentID);
      printf("Covariances could not be extracted from a relpose\n");
      cov = false;
      return false;
  }
  pose->GetPose(&pose_s);
#ifdef _DEBUG
  printf("\n\nPOSE S 2  : [%f, %f, %f, %f, %f, %f, %f ]\n\n",pose_s[0].D(), pose_s[1].D(),pose_s[2].D(),
  pose_s[3].D(),pose_s[4].D(),pose_s[5].D(),pose_s[6].D());
#endif /*_DEBUG*/
  //mean pose for camera base view
  pm.m_initRefX =pose_s[3].D()/180.0*PI; // <--- this will be ref. pose
  pm.m_initRefY =pose_s[4].D()/180.0*PI;
  pm.m_initRefZ =pose_s[5].D()/180.0*PI;
  pm.m_gravFinal[0] = gravPoint[0];
  pm.m_gravFinal[1] = gravPoint[1];
  pm.m_gravFinal[2] = gravPoint[2];
  pm.m_gravPointInited = true;
  if(cov)
  {
    ExtremePoses = GetExtremePoses(m);
    extents=GetExtents(ExtremePoses.t(), pose_s, gravPoint, calib, pm.m_region);

    double MeanDist = sqrt(pose_s[0].D()*pose_s[0].D()+pose_s[1].D()*pose_s[1].D()+pose_s[2].D()*pose_s[2].D());
    pm.m_distMin = MeanDist+extents[5].D();
    pm.m_distMax = MeanDist+extents[2].D();
    pm.m_latitudeMin = extents[4].D() *M_PI;
    pm.m_latitudeMax = extents[1].D() *M_PI;
    pm.m_longitudeMin = extents[3].D() *M_PI;
    pm.m_longitudeMax = extents[0].D() *M_PI;

    pm.m_camRollMin = -M_PI / 4.0;
    pm.m_camRollMax = M_PI / 4.0;

    /*pm.m_distMin = pose_s[2].D() - pose_s[2].D() * m.element(2,2);
      pm.m_distMax = pose_s[2].D() + pose_s[2].D() * m.element(2,2);
      pm.m_latitudeMin = M_PI / 2 * m.element(3,3);
      pm.m_latitudeMax = M_PI / 2 * m.element(3,3);
      pm.m_longitudeMin = M_PI / 2 * m.element(4,4);
      pm.m_longitudeMax = M_PI / 2 * m.element(4,4);

      pm.m_camRollMin = M_PI / 2 * m.element(5,5);
      pm.m_camRollMax = M_PI / 2 * m.element(5,5);*/
  }
  else
  {
      pm.m_distMin = pose_s[2].D() - pose_s[2].D();
      pm.m_distMax = pose_s[2].D() + pose_s[2].D();
      pm.m_latitudeMin = M_PI / 2;
      pm.m_latitudeMax = M_PI / 2;
      pm.m_longitudeMin = M_PI / 2;
      pm.m_longitudeMax = M_PI / 2;
      pm.m_camRollMin = M_PI / 2;
      pm.m_camRollMax = M_PI / 2;
      return false;
  }
#endif  /*HALCONIMG*/
  return true;
}

double ShapeModel::RotationError(double rot_x_1, double rot_y_1, double rot_z_1, double rot_x_2, double rot_y_2, double rot_z_2)
{
  double ret = 0.0;
  ret = sqrt((rot_x_1 - rot_x_2)*(rot_x_1 - rot_x_2) + (rot_y_1 - rot_y_2)*(rot_y_1 - rot_y_2)+(rot_z_1 - rot_z_2)*(rot_z_1 - rot_z_2));
  return ret / M_PI;
}

double ShapeModel::CheckOverlapping(ShapeModelParamSet* sm_new, ShapeModelParamSet* sm_old)
{
  double res = 0.0;
  int i = 0;
  /**Camroll*/
  res += fabs(1 - (sm_old->m_camRollMax - sm_old->m_camRollMin) / (sm_new->m_camRollMax - sm_new->m_camRollMin));
  i++;
  /**Distance*/
#ifdef _DEBUG
  printf("Search space comparison camroll %f (%f - %f / %f - %f)\n", res, sm_old->m_camRollMax , sm_old->m_camRollMin ,sm_new->m_camRollMax , sm_new->m_camRollMin);
#endif /*_DEBUG*/
  res += fabs(1 - (sm_old->m_distMax - sm_old->m_distMin) / (sm_new->m_distMax - sm_new->m_distMin));
  i++;
  /**Latitude*/
#ifdef _DEBUG
  printf("Search space comparison dist %f\n", res);
#endif /*_DEBUG*/
  res += fabs(1 - (sm_old->m_latitudeMax - sm_old->m_latitudeMin) / (sm_new->m_latitudeMax - sm_new->m_latitudeMin));
  i++;
  /**Camroll*/
#ifdef _DEBUG
  printf("Search space comparison lat %f\n", res);
#endif /*_DEBUG*/
  res += fabs(1 - (sm_old->m_longitudeMax - sm_old->m_longitudeMin) / (sm_new->m_longitudeMax - sm_new->m_longitudeMin));
  i++;
#ifdef _DEBUG
  printf("Search space comparison long %f\n", res);
#endif /*_DEBUG*/
  res = (1 - fabs(res));
  res *= (1 - RotationError(sm_old->m_initRefX, sm_old->m_initRefY, sm_old->m_initRefZ, sm_new->m_initRefX, sm_new->m_initRefY, sm_new->m_initRefZ ));
  return res;
}

int ShapeModel::SetShapeModelParamSet(ShapeModelParamSet* paramset, std::string st, bool shapeModelSaved)
{
  std::pair<ShapeModelParamSet*, std::pair< std::string, bool> > new_elem;
  new_elem.first = paramset;
  new_elem.second.first = st;
  new_elem.second.second = shapeModelSaved;
  printf("Added ShapeModelParamSet %s\n", st.c_str());
  m_shapeParams_file.push_back(new_elem);
  m_curIndex = (int)(m_shapeParams_file.size() -1);
  return m_curIndex;
}


bool CompareCalib(Calibration* cal1, Calibration* cal2)
{
  if(cal1 == cal2)
    return true;
  if(cal1 != NULL && cal2 != NULL)
  {
    if((cal1->m_width == cal2->m_width ) &&
       (cal1->m_height == cal2->m_height ) &&
       (fabs(cal1->m_calibrationmatrix[0] - cal2->m_calibrationmatrix[0]) <   0.00001) &&
       (fabs(cal1->m_calibrationmatrix[1] - cal2->m_calibrationmatrix[1])  < 0.00001))
          return true;
    else
    {
#ifdef _DEBUG
      printf("ShapeModel: Calibration differs\n");
#endif
      return false;
    }
  }
  return false;
}

double* ShapeModelParamSet::GetGravPoint(std::string st)
{
  if(!m_gravPointInited)
  {
#ifdef HALCONIMG
    Halcon::HObjectModel3D dxf;
    Halcon::HTuple t1, t2, gravPoint;
    Halcon::HTuple measure (m_measure);  //wie centimeter
    Halcon::HTuple name = st.c_str();
    try
    {
      printf("Reading Model for grav point\n");
     dxf.ReadObjectModel3dDxf(name, measure, t1, t2);
     gravPoint = dxf.GetObjectModel3dParams("reference_point");
     m_gravFinal[0] = gravPoint[0].D();
     m_gravFinal[1] = gravPoint[1].D();
     m_gravFinal[2] = gravPoint[2].D();
     printf("%f, %f, %f\n", m_gravFinal[0], m_gravFinal[1], m_gravFinal[2]);
    }
    catch (Halcon::HException ex)
    {
      printf("ShapeParamSet: GetGravPoint: %s\n", ex.message);
     throw "No Filename";
    }
    m_gravPointInited = true;
#endif  /*HALCONIMG*/
  }
  return m_gravFinal;
}


inline void normalize(double &a,double &b, double &c)
{
  double norm = sqrt(a*a + b*b + c*c);
  a /= norm;
  b /= norm;
  c /= norm;
}

inline void CrossProduct(const double b_x, const double b_y,const double b_z,const double c_x,const double c_y,const double c_z,double &a_x,double &a_y,double &a_z)
{
    a_x = b_y*c_z - b_z*c_y;
    a_y = b_z*c_x - b_x*c_z;
    a_z = b_x*c_y - b_y*c_x;
}

inline double Area(Polygon_t poly)
{
  double a_x,a_y,a_z;
  CrossProduct(poly[1].x - poly[0].x , poly[1].y - poly[0].y, poly[1].z - poly[0].z,
                poly[2].x - poly[0].x , poly[2].y - poly[0].y, poly[2].z - poly[0].z ,a_x,a_y,a_z);
  double area_poly =  sqrt(a_x*a_x +a_y*a_y + a_z*a_z) / 2;
  if(poly.size() > 3)
  {
    CrossProduct(poly[2].x - poly[0].x , poly[2].y - poly[0].y, poly[2].z - poly[0].z,
                poly[3].x - poly[0].x , poly[3].y - poly[0].y, poly[3].z - poly[0].z ,a_x,a_y,a_z);
    area_poly += sqrt(a_x*a_x +a_y*a_y + a_z*a_z) / 2;
  }
  return area_poly;
}

inline double scalarproduct(const double &a,const double &b, const double &c, const double &d, const double &e, const double &f)
{
  return a * d + b* e + c*f;
}

void ShapeModel::CalcMeshCenterBox(const Mesh_t &mesh, double &x, double &y, double &z, double &height,double &width, double &depth)
{
  double xt = -1000000000.0,yt = -100000000.0, zt = -100000000.0;
  double xm = 1000000000.0,ym = 10000000000.0, zm = 100000000.0;
  double xa = 0,ya = 0, za = 0;
  double area_acc = 0;
  for(Mesh_t::const_iterator it = mesh.begin(); it != mesh.end(); it++)
  {
    double area_poly = Area((*it).first) / (*it).first.size();
    for(Polygon_t::const_iterator it_poly = (*it).first.begin(); it_poly != (*it).first.end(); it_poly++)
    {
      xt = max((*it_poly).x, xt);
      xm = min((*it_poly).x, xm);
      yt = max((*it_poly).y, yt);
      ym = min((*it_poly).y, ym);
      zt = max((*it_poly).z, zt);
      zm = min((*it_poly).z, zm);
      xa += (*it_poly).x * area_poly;
      ya += (*it_poly).y * area_poly;
      za += (*it_poly).z * area_poly;
      area_acc += area_poly;
    }
  }
  x = xa / area_acc;
  y = ya / area_acc;
  z = za / area_acc;
  width = abs(xt - xm);
  height = abs(yt - ym);
  depth = abs(zt - zm);
}

void ShapeModel::EvalScaling( ShapeModelParamSet* sm,  Calibration* calib, std::string stFileName, RelPose* pose)
{
  /** TODO: implement scaling corrction and center to center movement*/
  int points;
  double xcenter,ycenter,zcenter;
  double width,height,depth;
  Matrix m = pose->GetMatrix();
  double ref_size = m.element(2,3) / 10;
  printf("ref_size : %f \n",ref_size );
  Matrix t(4,4);
  t << 1 << 0 << 0 << 0 <<
       0 << 1 << 0 << 0 <<
       0 << 0 << 1 << 0 <<
       0 << 0 << 0 << 1;

  Mesh_t mesh = ReadMesh(stFileName, 1.0, t, points);
  CalcMeshCenterBox(mesh, xcenter,ycenter,zcenter, width,height,depth);
  double max_ext = max( max (width,height), depth);
#ifdef _DEBUG
  printf("Before\n\n Center: %f, %f, %f\n w : %f\n h: %f\n d: %f\n adapt Scaling by %f\n",xcenter,ycenter,zcenter, width,height,depth, (ref_size / max_ext));
#endif /*DEBUG*/
  t << 1 << 0 << 0 << -xcenter * (ref_size / max_ext) <<
       0 << 1 << 0 << -ycenter * (ref_size / max_ext) <<
       0 << 0 << 1 << -zcenter * (ref_size / max_ext) <<
       0 << 0 << 0 << 1;
  mesh = ReadMesh(stFileName, ref_size / max_ext, t, points);
  CalcMeshCenterBox(mesh, xcenter,ycenter,zcenter, width,height,depth);
#ifdef _DEBUG
  printf("After:\n\n Center: %f, %f, %f\n w : %f\n h: %f\n d: %f\n",xcenter,ycenter,zcenter, width,height,depth);
#endif /*DEBUG*/
  dxfwriter::WriteMesh(mesh, stFileName);
#ifdef _DEBUG
  printf("Overwrote File: %s\n",stFileName.c_str());
#endif /*DEBUG*/

  sm->m_gravPointInited = true;
  sm->m_gravFinal[0] = xcenter;
  sm->m_gravFinal[1] = ycenter;
  sm->m_gravFinal[2] = zcenter;
  sm->m_measure = 1.0;
  if(points > 1000)
  {
    sm->m_minFaceAngle = 3.141;
  }
  else
  {
    sm->m_minFaceAngle = 0.3;
  }
}

bool AdaptRegion(RegionOI* reg, RelPose* pose, Calibration* calib)
{
    if(reg != NULL)
    {
#ifdef HALCONIMG
        Halcon::HTuple area, ro, co,r,c, hom,tup;
        pose->GetPose(&tup, 0);
        Halcon::Hobject temp;
        Halcon::project_3d_point(tup[0].D(), tup[1].D(), tup[2].D(), calib->CamParam(), &r,&c);
        Halcon::area_center(reg->m_reg, &area, &ro, &co);
        Halcon::hom_mat2d_identity(&hom);
        Halcon::hom_mat2d_translate(hom, r - ro, c - co, &hom);
        Halcon::affine_trans_region(reg->m_reg, &reg->m_reg, hom, "none");
#endif
    }
    return false;
}

bool ShapeModel::SetShapeModelParamSet(RelPose* pose, Calibration* calib, double prob)
{
    ShapeModelParamSet* pm = new ShapeModelParamSet(calib);
    bool bReturn = false;
    unsigned int num = m_shapeParams_file.size();
    std::string stChecked = "";
    if(num == 0)
      throw "No ShapeModelParamSet available\n";
    for(unsigned int i = 0; i < num; i++)
    {
      ShapeModelParamSet* shapeParams = m_shapeParams_file[i].first;
      if(m_initializationLEvel < 0.1)
      {
        printf("EvalScaling (%p, %p, %s, %p)", shapeParams, calib, m_shapeParams_file[i].second.first.c_str(), pose);
          EvalScaling(shapeParams, calib, m_shapeParams_file[i].second.first, pose);
          m_initializationLEvel += 0.1;
      }
      std::string st = m_shapeParams_file[i].second.first;
      //bool shapeModelSaved = m_shapeParams_file[i].second.second;
      double *gravPoint = shapeParams->GetGravPoint(st);
      if(st.compare(stChecked) != 0 && i == 0)
      {
#ifdef HALCONIMG
#ifdef _DEBUG
        pose->Print();
#endif
#endif
        bReturn = PoseToRange(pose, *pm, gravPoint, calib);
      }
      double res = CheckOverlapping(pm, shapeParams);
#ifdef _DEBUG
      printf("PROB,RES:    %f, %f\n",prob,res);
#endif /*_DEBUG*/
      if(bReturn && ( res < prob  || res > 1.05))
      {
          if(i < num - 1)
          {
#ifdef _DEBUG
                  printf("Compared but different trying the next\n");
#endif
                  continue;
          }
          pm->m_minContrast = shapeParams != NULL ? shapeParams->m_minContrast : 10;
          pm->m_minFaceAngle = shapeParams != NULL ? shapeParams->m_minFaceAngle : 0.3;
          pm->m_measure = shapeParams != NULL ? shapeParams->m_measure : 0.01;
          pm->m_scale = shapeParams != NULL ? shapeParams->m_scale : 1;
          SetShapeModelParamSet(pm,st, false);
          m_initialized = false;
          break;
      }
      else
      {
#ifdef _DEBUG
        printf("Found an overlapping model (%d)\n", i);
#endif /*_DEBUG*/
        if((unsigned)m_curIndex != i && m_initialized)
        {
          printf("This model was not initialized\n");
#ifdef HALCONIMG
          Halcon::clear_shape_model_3d(m_ShapeModelID);
#endif
          m_initialized = false;
        }
        else if(!m_initialized)
        {
          if(!CompareCalib(shapeParams->m_calib, calib))
          {
            printf("Calibration set to the overlapping model\n");
            if(shapeParams->m_deleteCalib)
            {
              printf("%p\n", shapeParams->m_calib);
              delete shapeParams->m_calib;
              printf("delete did not crash\n");
            }
            printf("Calibration to set: %d %d %f\n", calib->m_height, calib->m_width, calib->m_calibrationmatrix[0]);
            shapeParams->m_calib = calib;
            shapeParams->m_deleteCalib = false;
            AdaptRegion(shapeParams->m_region, pose, calib);
            m_shapeParams_file[i].second.second = false;
          }
          else
          {
#ifdef _DEBUG
            printf("Calibration did  not change, keeping the same ShapeModel\n");
#endif /*_DEBUG*/
          }
        }
        if(shapeParams->m_region == NULL)
        {
          printf("The Region of the Current ShapeModelParamSet is NULL!\n");
        }
        else
        {
          if(shapeParams->m_region->GetSize() == 0)
          {
            printf("The Region of the Current ShapeModelParamSet is empty\n");
            delete shapeParams->m_region;
            shapeParams->m_region = NULL;
          }
#ifdef _DEBUG
          else
            printf("shapeParams->m_region->GetSize() = %d\n", shapeParams->m_region->GetSize());
#endif
        }
        m_curIndex = i;
        delete pm;
        break;
      }
    }
    return bReturn;
}


#ifdef HALCONIMG
Halcon::Hobject ShapeModel::GetContour(RelPose& rpose, Camera* cam)
{
  Halcon::HTuple camparam;
  if(cam != NULL)
  {
    camparam = cam->m_calibration.CamParam();
  }
  else
  {
    if(m_shapeParams_file[m_curIndex].first->m_calib != NULL)
    {
      camparam = m_shapeParams_file[m_curIndex].first->m_calib->CamParam();
    }
    else
    {
     throw "No contour can be showed";
    }
  }
  Halcon::HTuple pose, d;
  Halcon::Hobject obj;
  double scale;
  //if(cam != NULL)
    //rpose.GetPose(&pose, cam->m_relPose);
  //else
    rpose.GetPose(&pose);
  //printf("Pose: %f,%f,%f /%f,%f,%f // %d\n", pose[0].D(), pose[1].D(), pose[2].D(), pose[3].D(), pose[4].D(), pose[5].D(), pose[6].I());
  long shape = GetShapeModel(scale);
  if(shape == -1)
    throw "Shape Model not yet initialized";
  if(pose[2].D() != 0)
  {
    //printf("Projecting Model\n");
    try
    {
        printf("Contour at pose\n");
        rpose.Print();
      /*Testing if scale is needed here, or not! or if a trick is needed*/
       project_shape_model_3d(&obj, shape, camparam ,pose, "true", 0.35);
       return obj;
    }
    catch(Halcon::HException ex)
    {
      printf("Error: %s\n", ex.message);
      gen_empty_obj (&obj);
      return obj;
    }
  }
  else
  {
    gen_empty_obj (&obj);
    return obj;
  }
}
#endif


