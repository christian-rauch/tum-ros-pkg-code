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
                        VisFinder.h - Copyright klank

**************************************************************************/


#ifndef VISFINDER_H
#define VISFINDER_H

#include <string>
#include <vector>

#include "AlgorithmSelector.h"
#include "RelPoseFactory.h"
#include "Image.h"
#include "AttentionManager.h"
#include "VisLearner.h"
#include "TrackAlgorithm.h"

#define XML_NODE_VISFINDER "VisualFinder"


/**
  * class VisFinder
  * @brief Basic class for loacting objects
  */
class VisFinder
{
public:

  // Constructors/Destructors
  //

  /**
   * Constructor VisFinder
   * @brief constructs a visual finder, a derivate of algorithm selector
   * @param configFile configuation of the visual finder, menaing which algorithms are in it
   * @param imageSystem reference to the camera setup ussed to get images from
   * @param db reference the the signature database used for looking up model information
   * @param manager Attention Manager, not specified
   * @param visLearner reference to the model improver can be triggered by the visual finder
#ifdef LOGFILE
   * @param log the central logfile
#endif
   */
  VisFinder ( XMLTag*  configFile, ImageInputSystem& imageSystem, SignatureDB& db, AttentionManager& manager, VisLearner&	visLearner
#ifdef LOGFILE
        , LogFile& log
#endif
        );

  /**
   * Empty Destructor
   */
  virtual ~VisFinder ( );

  // Static Public attributes
  //

  // Public attributes
  //


  // Public attribute accessor methods
  //
  VisFinder& operator=(VisFinder&){throw "Error";}

  // Public attribute accessor methods
  //

  XMLTag* Query(std::string queryString);

  /**
  *	Detect
  *	@brief detects the class or object that is specified by object, updates its signature if it succeeds,
  *			sets the numOfObjects, returns numOfObjects > 0.
  *
   * @return RelPose
   * @param  LastKnownPose
   * @param  Object
   * @param  numOfObjects
   */
  virtual bool Detect (RelPose& lastKnownPose, Signature& object, int &numOfObjects);

  /**
  *	Locate
  *	@brief locates the class or object that is specified by object, updates its signature if it succeeds,
  *			sets the numOfObjects, returns the RelPose* where the first object was found.
  *	@return RelPose
  *	@param  LastKnownPose
  *	@param  Object
  *	@param  numOfObjects
  */
  virtual RelPose* Locate (RelPose& lastKnownPose, Signature& object, int &numOfObjects);
  /**
  *	Locate
  *	@brief locates the class or object that is specified by object, updates its signature if it succeeds,
  *			sets the numOfObjects, returns the RelPose* where the first object was found.
  *	@return the most probably position, rest is stored in the signature
  *	@param  a list of possible positions and the a-prioi probabilitiers for the object beeing at this position
  *	@param
  *	@param  numOfObjects
  */
  virtual SignatureLocations_t Locate (PossibleLocations_t* lastKnownPoses, Signature& object, int &numOfObjects);

  /**
  *  Searches for searchspaces
  */
  bool GetPlaneClusterCall(PossibleLocations_t*, RelPose*, Signature&);

  /**
    *   Starts a thread tracks the object specified with object
   * @param  object contains a trackable signature
   */
  virtual void StartTrack (Signature& object, RelPose* pose);
  /**
    *   Stops the thread that tracks the object specified with object
   * @param  object contains a trackable signature
   */
  virtual void StopTrack (Signature& object);


  /**
   * @return RelPose
   * @param  CurImage
   * @param  Pose
   * @param  Obj1
   * @param  Obj2
   */
  virtual RelPose* RelTwoObjects (const RelPose& pose, Signature& sig1, Signature& sig2);


  /**
   * @param  world
   */
  virtual void UpdateWorldStatus (Signature &world );

  /**
   * @return double
   * @param  DistractionPoints
   * @param  RoIs
   * @param  DisctionLevels
   */
  virtual double GetDistractionPoints (std::vector<RelPose*> &DistractionPoints, std::vector<double> &RoIs, std::vector<double> &DisctionLevels ) const;
  /**
  *	Saves the Visual Finder
  */
  virtual XMLTag* Save();
  /**
  *	Adds a new Algorithm to the visual finder
  */
  void AddAlgorithm(Algorithm<std::vector<RelPose*> >* alg);
  /**
  *	Return num of Algorithms in the visual finder
  */
  int CountAlgorithms(){return m_selLocate.CountAlgorithms();}
private:
    /**
  *	The list of algorithms and their type and evaluation
  */
    AlgorithmSelector<std::vector<RelPose*> > m_selLocate;
    std::map<int, TrackAlgorithm*> m_runningTracks;
public:
  ImageInputSystem& m_imageSys;
  SignatureDB&      m_sigdb;
  VisLearner&       m_visLearner;


protected:
  // Protected attributes
  //
  AttentionManager& m_attentionMan;


};

#endif // VISFINDER_H
