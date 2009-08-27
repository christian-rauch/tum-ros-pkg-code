/*
 *  Calibrate (i.e. find intrinsic and extrinsic calibration parameters) cameras
 *  using left-right sets of images
 *
 *  Copywrong (K) 2008 Radu Bogdan Rusu
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * $Id$
 */
#ifndef CALIBRATION_STANDALONE_H
#define CALIBRATION_STANDALONE_H

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <boost/algorithm/string.hpp>

#include <stdio.h>
#include <vector>
#include <iostream>
#include <string>

#define NR_SQUARE_X 4 
#define NR_SQUARE_Y 6
#define SQ_SIZE     41

#define DEBUG_GUI 1
#define DEBUG_CORNERS_GUI 1

////////////////////////////////////////////////////////////////////////////////
// Parse for a specific given command line argument.
// Returns: the value sent as an int.
int  
  ParseArgument (int argc, char** argv, const char* str, int &val)
{
  for (int i = 1; i < argc; i++)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      val = atoi (argv[i]);
      return (i-1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
// Parse for a specific given command line argument.
// Returns: the value sent as a double.
int  
  ParseArgument (int argc, char** argv, const char* str, double &val)
{
  for (int i = 1; i < argc; i++)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      val = atof (argv[i]);
      return (i-1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
// Parse for specific given command line arguments (range values X-Y).
// Returns: the values sent as ints.
int
  ParseRangeArguments (int argc, char** argv, const char* str, int &s, int &e)
{
  std::vector<std::string> values;
  for (int i = 1; i < argc; i++)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      // look for ',' as a separator
      boost::split (values, argv[i], boost::is_any_of (","), boost::token_compress_on);
      if (values.size () != 2)
        fprintf (stderr, "Number of values for %s different than 2!\n", str);
      s = atoi (values.at (0).c_str ());
      e = atoi (values.at (1).c_str ());
      return (i-1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
// Parse command line arguments for file names.
// Returns: a vector with file names indices.
std::vector<int> 
  ParseFileExtensionArgument (int argc, char** argv, const char *extension)
{
  std::vector<int> indices;
  for (int i = 1; i < argc; i++)
  {
    std::string fname = std::string (argv[i]);
    std::string ext = std::string (extension);
 
    // Needs to be at least 4: .ext
    if (fname.size () <= 4)
      continue;
    
    // For being case insensitive
    std::transform (fname.begin (), fname.end (), fname.begin (), tolower);
    std::transform (ext.begin (), ext.end (), ext.begin (), tolower);
    
    // Check if found
    std::string::size_type it;
    if ((it = fname.find (ext)) != std::string::npos)
    {  
      // Additional check: we want to be able to differentiate between .p and .png
      if ((ext.size () - (fname.size () - it)) == 0)
        indices.push_back (i);
    }
  }  
  return indices;
}

////////////////////////////////////////////////////////////////////////////////
// Display a cvMat on screen
void
  printMatrix (CvMat *matrix, double value = 1.0)
{
  for (int i = 0; i < matrix->height; i++)
  {
    for (int j = 0; j < matrix->width; j++)
      fprintf (stderr, " <<  %10.4f ", cvmGet (matrix, i, j) / value);
    fprintf (stderr, "\n");
  }
}

////////////////////////////////////////////////////////////////////////////////
// Display a cvMat on screen
void
  printTransformation (CvMat *rot, CvMat *tr)
{
  for (int i = 0; i < rot->height; i++)
  {
    for (int j = 0; j < rot->width; j++)
      fprintf (stderr, " <<  %6.4f ", cvmGet (rot, i, j));

    fprintf (stderr, " <<  %6.4f \n", cvmGet (tr, i, 0));
  }
  fprintf (stderr, " <<   0       <<   0       <<   0       <<    1\n");
}

////////////////////////////////////////////////////////////////////////////////
/// Create a checkboard model
/// @note: it doesn't matter what values we have here as long as they are
/// consistently used for all images
std::vector <CvPoint3D64f>
  getObjectPoints (CvSize checkboard, double sq_size, int n_frames)
{
  std::vector <CvPoint3D64f> points (checkboard.height * checkboard.width * n_frames);

  for (int i = 0; i < checkboard.height; ++i)
  {
    for (int j = 0; j < checkboard.width; ++j)
    {
      points[i * checkboard.width + j] = cvPoint3D64f (sq_size * i, sq_size * j, 0);
    }
  }

  int cs = checkboard.height * checkboard.width;
  for (int i = 1; i < n_frames; i++)
    copy (points.begin (), points.begin () + cs, points.begin () + i * cs);
  return points;
}



////////////////////////////////////////////////////////////////////////////////
// Define a sensor data struc to hold all data for a given <sensor>-<camera> entry
struct SensorDataEntry
{
  // All image data for a specified <sensor>-<camera> entry is stored here
  std::vector <IplImage*> images;

  // Bookkeeping the pair numbers and the file paths
  std::vector <int>         pairs;
  std::vector <std::string> paths;

  // String identifiers for the sensor (sr4k, stoc, etc) and camera (left, right, intensity) images used
  std::string sensor;
  std::string camera;

  // OpenCV matrices holding the internal parameters and related stuff
  CvMat *objectPoints;    // The joint matrix of object points, 3xN or Nx3, where N is the total number of points in all views
  CvMat *imagePoints;     // The joint matrix of corresponding image points, 2xN or Nx2, where N is the total number of points in all views
  CvMat *pointNumbers;    // Vector containing numbers of points in each particular view, 1xM or Mx1, where M is the number of a scene views
  CvMat *intrinsic;       // [fx 0 cx; 0 fy cy; 0 0 1]
  CvMat *distortion;      // [k1, k2, p1, p2]

  // Store estimated translations and rotations with respect to a world coordinate framework for a given image
  CvMat *translation;     // optimized translation
  CvMat *rotation;        // optimized rotation
  CvMat *translations;
  CvMat *rotations;       // compact representation of rotation matrices (use cvRodrigues2)

  // Comparator functions for std::find
  bool
    operator < (SensorDataEntry const & b) const
  {
    bool c = this->sensor < b.sensor;
    if (!c)
      return false;
    return this->camera < b.camera;
  }

  bool
    operator > (SensorDataEntry const & b) const
  {
    bool c = this->sensor > b.sensor;
    if (!c)
      return false;
    return this->camera > b.camera;
  }

  bool
    operator == (SensorDataEntry const & b) const
  {
    bool c = this->sensor == b.sensor;
    if (!c)
      return false;
    return this->camera == b.camera;
  }
};

////////////////////////////////////////////////////////////////////////////////
// Release all allocated memory and prepare to exit
void
  finishUp (std::vector<SensorDataEntry> inputData)
{
  /// ---[ Release memory
  for (unsigned int i = 0; i < inputData.size (); i++)
  {
    SensorDataEntry s = inputData.at (i);
    for (unsigned int j = 0; j < s.pairs.size (); j++)
      cvReleaseImage (&s.images.at (j));
    cvmFree (s.objectPoints);
    cvmFree (s.imagePoints);
    cvmFree (s.pointNumbers);
    cvmFree (s.intrinsic);
    cvmFree (s.distortion);
    cvmFree (s.translations);
    cvmFree (s.rotations);
  }
  cvDestroyAllWindows ();
}

////////////////////////////////////////////////////////////////////////////////
// Create and initialize all data structures from a set of given PNG file names
std::vector<SensorDataEntry>
  createSensorEntries (int argc, char** argv, std::vector<int> pPNGFileIndices, CvSize checkboard, int ith)
{
  std::vector<SensorDataEntry> inputData;
  std::vector<std::string> st1, st2, st3;

  // ---[ Phase 1 : create NR-SID-CID entries
  for (unsigned int i = 0; i < pPNGFileIndices.size (); i++)
  {
    SensorDataEntry s;
    // Get the current entry
    std::string entry (argv[pPNGFileIndices.at (i)]);

    // Parse and get the filename out
    boost::split (st1, entry, boost::is_any_of ("/"), boost::token_compress_on);
    std::string file = st1.at (st1.size () - 1);

    // Parse inside the filename for NR-SID-CID
    boost::split (st2, file, boost::is_any_of ("."), boost::token_compress_on);
    boost::split (st3, st2.at (0), boost::is_any_of ("-"), boost::token_compress_on);
    int nr = atoi (st3.at (0).c_str ());
    std::string sid = st3.at (1);
    std::string cid = st3.at (2);
    // Add <sensor>-<camera> entries
    s.sensor = sid;
    s.camera = cid;

    // Check to see if we already have an entry with <sensor>-<camera>
    std::vector<SensorDataEntry>::iterator it = find (inputData.begin (), inputData.end (), s);
    // If we do, add the current image pair number to it
    if (it != inputData.end ())
    {
      SensorDataEntry *pen = &(*it);
      pen->pairs.push_back (nr);
      pen->paths.push_back (entry);
    }
    // Else, create a new entry in inputData
    else
    {
      s.pairs.push_back (nr);
      s.paths.push_back (entry);
      inputData.push_back (s);
    }
  }

  // ---[ Phase 2 : load image data into IplImage structures
  for (unsigned int i = 0; i < inputData.size (); i++)
  {
    SensorDataEntry s = inputData.at (i);
    fprintf (stderr, "Processing data for "); fprintf (stderr, "%s - %s", s.sensor.c_str (), s.camera.c_str ());
    fprintf (stderr, " : ");

    unsigned int nr_pairs = 0;
    std::vector <int> new_pairs;
    std::vector <std::string> new_paths;
    for (unsigned int j = 0; j < s.pairs.size (); j+=ith)
    {
      fprintf (stderr, "%d ", s.pairs.at (j));
      new_pairs.push_back (s.pairs.at (j));
      new_paths.push_back (s.paths.at (j));
      // Load data into IplImage
      inputData.at (i).images.push_back ( cvLoadImage ( s.paths.at (j).c_str () ) );
      nr_pairs++;
    }
    fprintf (stderr, " (%d x %d)\n", inputData.at (i).images.at (0)->width, inputData.at (i).images.at (0)->height);

    inputData.at (i).pairs = new_pairs;
    inputData.at (i).paths = new_paths;

    inputData.at (i).rotation     = cvCreateMat (1, 3, CV_64FC1);
    inputData.at (i).translation  = cvCreateMat (1, 3, CV_64FC1);
    // Initialize OpenCV related stuff
    inputData.at (i).intrinsic    = cvCreateMat (3, 3, CV_64FC1);
    inputData.at (i).distortion   = cvCreateMat (1, 4, CV_64FC1);
    // Allocate a 1D array (1 x MaxPairs) which stores the point indices
    /// @note: do not create the pointNumbers here, but rather after <getCalibrationCheckboardCorners ()>
    ///inputData.at (i).pointNumbers = cvCreateMat (1, nr_pairs, CV_32SC1);
    // Allocate a 3D array (M x 3) for <x,y,z> comprised of M=Cw*Ch*MaxPairs values (alloc more than needed)
    /// @note: do not create the objectPoints here, but rather after <getCalibrationCheckboardCorners ()>
    ///inputData.at (i).objectPoints = cvCreateMat (checkboard.width * checkboard.height * nr_pairs, 3, CV_64FC1);
    // Allocate a 2D array (M x 2) for <u,v> comprised of M=Cw*Ch*MaxPairs values (alloc more than needed)
    /// @note: do not create the imagePoints here, but rather after <getCalibrationCheckboardCorners ()>
    ///inputData.at (i).imagePoints  = cvCreateMat (checkboard.width * checkboard.height * nr_pairs, 2, CV_64FC1);
  }

  // ---[ Phase 3 : check whether the number of pairs is the same across all <sensor>s-<camera>s
  unsigned int nr_pairs = inputData.at (0).pairs.size ();
  for (unsigned int i = 1; i < inputData.size (); i++)
  {
    SensorDataEntry s = inputData.at (i);
    if (s.pairs.size () != nr_pairs)
      fprintf (stderr, "Number of pairs for %s-%s (%d) differs than the standard %d!\n", s.sensor.c_str (), s.camera.c_str (), (int)s.pairs.size (), nr_pairs);
  }

  return inputData;
}

////////////////////////////////////////////////////////////////////////////////
// Given a sensor entry and a number of images, detect and store the imagePoints
// as the calibration checkboard corners
// - also update {good,bad}Pairs, by keeping a list of pairs which are good/bad
/// @note: badPairs contains <sde.pairs.at (i)> and not <i> like goodPairs!
void
  getCalibrationCheckboardCorners (SensorDataEntry &sde, CvSize checkboard, double sq_size,
                                   std::vector<int> &badPairs, std::vector<int> &goodPairs)
{
  CvPoint2D32f *corners = new CvPoint2D32f[checkboard.width * checkboard.height];                         // allocate space for all corners
  std::vector<CvPoint2D32f> imagePoints_temp (checkboard.width * checkboard.height * sde.pairs.size ());  // allocate space for all possible 2D points

  int validPtIdx = 0;  // global point index

  // Iterate over each image
  for (unsigned int i = 0; i < sde.pairs.size (); i++)
  {
    std::vector<int>::iterator bad_it = find (badPairs.begin (), badPairs.end (), sde.pairs.at (i));
    if (bad_it != badPairs.end ())
    {
      fprintf (stderr, "Pair number %d already marked as invalid. Continuing...\n", sde.pairs.at (i));
      continue;
    }

    // Convert each image to grayscale (BGR2GRAY, 1 channel)
    IplImage *grayImage = cvCreateImage (cvGetSize (sde.images.at (i)), IPL_DEPTH_8U, 1);
    cvCvtColor (sde.images.at (i), grayImage, CV_BGR2GRAY);

    int cornerCount = 0;
    // Finds positions of internal corners of the chessboard in the newly created grayscale image
    if (cvFindChessboardCorners (grayImage, checkboard, corners, &cornerCount, CV_CALIB_CB_ADAPTIVE_THRESH))// | CV_CALIB_CB_NORMALIZE_IMAGE)) // | CV_CALIB_CB_FILTER_QUADS))
    {
      fprintf (stderr, "["); fprintf (stderr, "%2d", sde.pairs.at (i)); fprintf (stderr, "/"); fprintf (stderr, "%2d", cornerCount); fprintf (stderr, "] ");
      // Refine the corner locations found using cvFindChessboardCorners
      cvFindCornerSubPix (grayImage, corners, cornerCount,
                          //cvSize (11, 11),
                          cvSize (5, 5),
                          //checkboard,
                          cvSize (-1, -1),
                          cvTermCriteria (CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 0.001));

#if DEBUG_CORNERS_GUI
      char windowTitle[80];
      sprintf (windowTitle, "%s-%s : %d", sde.sensor.c_str (), sde.camera.c_str (), sde.pairs.at (i));
      // Create window
      cvNamedWindow (windowTitle, CV_WINDOW_AUTOSIZE);
      cvDrawChessboardCorners (grayImage, checkboard, corners, cornerCount, 1);
#endif

      // For each corner found in the image, increment the number of corners found for all images and store them in the image/object points
      for (int j = 0; j < cornerCount; j++, validPtIdx++)
      {
        if ( (sde.sensor == "sr4k") || (sde.sensor == "sr3k") || (sde.sensor == "thermal") )
        {
          corners[j].x /= 2.0; corners[j].y /= 2.0;
        }
        imagePoints_temp[validPtIdx] = cvPoint2D32f (corners[j].x, corners[j].y); // Set the image points <u, v> as the freshly detected corners

#if DEBUG_CORNERS_GUI
        if (j == 0) cvCircle (sde.images.at (i), cvPoint ((int) corners[j].x, (int) corners[j].y), 2, CV_RGB (0, 255, 0), 1); // Mark the first circle with green
        else        cvCircle (sde.images.at (i), cvPoint ((int) corners[j].x, (int) corners[j].y), 2, CV_RGB (0, 0, 255), 1); // And the rest with red
#endif
      } // for
#if DEBUG_CORNERS_GUI
      cvShowImage (windowTitle, grayImage);
      //cvWaitKey (0);                    // Wait until key pressed
      cvDestroyWindow (windowTitle);    // Destroy previous window
#endif
      goodPairs.push_back (i);   // push <i>
    } // if (checkboardFound)
    else
    {
      if (cornerCount == checkboard.width * checkboard.height)
        { fprintf (stderr, "["); fprintf (stderr, "%2d", sde.pairs.at (i)); fprintf (stderr, "/"); fprintf (stderr, "0"); fprintf (stderr, "] "); }
      else
        { fprintf (stderr, "["); fprintf (stderr, "%2d", sde.pairs.at (i)); fprintf (stderr, "/"); fprintf (stderr, "%2d", cornerCount); fprintf (stderr, "] "); }
      badPairs.push_back (sde.pairs.at (i));   // push <i=421> and not <i>
    }

    cvReleaseImage (&grayImage);
  }
  fprintf (stderr, "\n");
#if DEBUG
  fprintf (stderr, "Done: "); fprintf (stderr, "%d", (int)goodPairs.size ()); fprintf (stderr, " images.\n\n");
#endif

  /// Prepare to copy data to the Sensor entry
  std::vector<CvPoint3D64f> vObjectPoints = getObjectPoints (checkboard, sq_size, goodPairs.size ());

  sde.objectPoints = cvCreateMat (checkboard.width * checkboard.height * goodPairs.size (), 3, CV_64FC1);
  sde.imagePoints  = cvCreateMat (checkboard.width * checkboard.height * goodPairs.size (), 2, CV_64FC1);
  sde.pointNumbers = cvCreateMat (1, goodPairs.size (), CV_32SC1);

  for (unsigned int i = 0; i < goodPairs.size (); i++)
    cvSet2D (sde.pointNumbers, 0, i, cvScalar (checkboard.width * checkboard.height));

  for (unsigned int i = 0; i < checkboard.width * checkboard.height * goodPairs.size (); i++)
  {
    // Set the 3D points <x, y, z = 0> by copying them from our generated checkboard pattern
    cvmSet (sde.objectPoints, i, 0, vObjectPoints[i].x);
    cvmSet (sde.objectPoints, i, 1, vObjectPoints[i].y);
    cvmSet (sde.objectPoints, i, 2, 0.0);

    cvmSet (sde.imagePoints, i, 0, imagePoints_temp[i].x);
    cvmSet (sde.imagePoints, i, 1, imagePoints_temp[i].y);
  }

  // Dealloc
  delete [] corners;
}

////////////////////////////////////////////////////////////////////////////////
// Creates a 4x4 transformation from a 1x3 R and a 1x3 t
void
  get4x4TransformationMatrix (CvMat *R, CvMat *t, CvMat *T)
{
  cvSetZero (T);
  CvMat *rot3x3 = cvCreateMat (3, 3, CV_64FC1);
  if (R->width == 3 && R->height == 3)
    rot3x3 = R;
  else
    cvRodrigues2 (R, rot3x3);

  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      cvmSet (T, i, j, cvmGet (rot3x3, i, j));

  for (int j = 0; j < 3; j++)
    cvmSet (T, j, 3, cvmGet (t, 0, j));

  cvmSet (T, 3, 3, 1);
  if (R->width != 3 || R->height != 3)
    cvmFree (rot3x3);
}

////////////////////////////////////////////////////////////////////////////////
int
  getExtrinsicBetween (std::string left, std::string right, CvSize checkboard,
                       std::vector<SensorDataEntry> inputData, int nr_pairs, int *nums)
{
  SensorDataEntry sl, sr;
  std::vector<std::string> st;

  boost::split (st, left, boost::is_any_of ("-"), boost::token_compress_on);

  if (st.size () == 2)
  {
    sl.sensor = st.at (0);
    sl.camera = st.at (1);
  }
  else
    sl.sensor = st.at (0);
  std::vector<SensorDataEntry>::iterator left_it = find (inputData.begin (), inputData.end (), sl);
  if (left_it == inputData.end ())
  {
    fprintf (stderr, "%s-%s not found!\n", sl.sensor.c_str (), sl.camera.c_str ());
    return (-1);
  }

  boost::split (st, right, boost::is_any_of ("-"), boost::token_compress_on);
  if (st.size () == 2)
  {
    sr.sensor = st.at (0);
    sr.camera = st.at (1);
  }
  else
    sr.sensor = st.at (0);
  std::vector<SensorDataEntry>::iterator right_it = find (inputData.begin (), inputData.end (), sr);
  if (right_it == inputData.end ())
  {
    fprintf (stderr, "%s-%s not found!\n", sr.sensor.c_str (), sr.camera.c_str ());
    return (-1);
  }

  SensorDataEntry *pleft      = &(*left_it);
  SensorDataEntry *pright     = &(*right_it);

  double R[3][3], T[3], E[3][3], F[3][3];
  CvMat LR_Rot = cvMat (3, 3, CV_64F, R);
  CvMat LR_T   = cvMat (3, 1, CV_64F, T);
  CvMat LR_E   = cvMat (3, 3, CV_64F, E);
  CvMat LR_F   = cvMat (3, 3, CV_64F, F);

  cvStereoCalibrate (pleft->objectPoints, pleft->imagePoints, pright->imagePoints,
                     pleft->pointNumbers,
                     pleft->intrinsic, pleft->distortion, pright->intrinsic, pright->distortion,
                     cvGetSize(inputData.at (0).images.at (0)),
                     &LR_Rot, &LR_T, &LR_E, &LR_F,

                     cvTermCriteria (CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5),
		     CV_CALIB_USE_INTRINSIC_GUESS +
		     CV_CALIB_FIX_ASPECT_RATIO +
		     CV_CALIB_ZERO_TANGENT_DIST +
		     CV_CALIB_SAME_FOCAL_LENGTH);

  /// ---[ Display results
  fprintf (stderr, " Transformation Matrix "); fprintf (stderr, "%s", left.c_str ()); fprintf (stderr, " -> " ); fprintf (stderr, "%s \n", right.c_str ());
  printTransformation (&LR_Rot, &LR_T);

  return (0);
}


////////////////////////////////////////////////////////////////////////////////
/// Obtain the intrinsic camera parameters error for this entry using its associated input images
void
  calculateIntrinsicParametersError (SensorDataEntry sde,
                                     std::vector<int> &badPairs, std::vector<int> goodPairs,
                                     double max_err)
{
  // Calculate the back-projection error: residual
  double err = 0.0;
  CvMat *tmpObjPt = cvCreateMatHeader (2, 2, CV_64FC1);
  CvMat *tmpImgPt = cvCreateMatHeader (2, 2, CV_64FC1);

  CvMat *tmpRotation    = cvCreateMatHeader (1, 3, CV_64FC1);
  CvMat *tmpTranslation = cvCreateMatHeader (1, 3, CV_64FC1);

  CvMat *bestRotation = NULL, *bestTranslation = NULL;
  double bestErr = FLT_MAX;
  int bestPair = 0;

  // For each valid image pair
  int totalPtIdx = 0;
  fprintf (stderr, "Computing back-projection error for pair ");
  for (unsigned int i = 0; i < goodPairs.size (); i++)
  {
    fprintf (stderr, "%d", goodPairs.at (i));
    unsigned int nrPts = sde.pointNumbers->data.i[i];     // this is always going to be equal with cornerCount, see line 995

    // Create a new matrix where the projected 2D <u,v> points will be held
    CvMat *projImgPoints = cvCreateMat (nrPts, 2, CV_64FC1);

    // Get the first nrPts from pair <i>, 3D (resObjPoints) and 2D (resImgPoints)
    cvGetSubRect (sde.objectPoints, tmpObjPt, cvRect (0, totalPtIdx, 3, nrPts));
    cvGetSubRect (sde.imagePoints,  tmpImgPt, cvRect (0, totalPtIdx, 2, nrPts));

    // And get the appropriate rotation and translation values
    cvGetSubRect (sde.rotations,    tmpRotation,    cvRect (0, i, 3, 1));
    cvGetSubRect (sde.translations, tmpTranslation, cvRect (0, i, 3, 1));

    // Use the rotation and translations to project the 3D points from tmpObjPt onto the image plane
    // using the camera intrinsic parameters (sde.intrinsic, sde.distortion) and the current extrinsic (R, t) pair
    /// @note: results are stored in projImgPoints in <u,v>
    cvProjectPoints2 (tmpObjPt, tmpRotation, tmpTranslation, sde.intrinsic, sde.distortion, projImgPoints);

#if DEBUG_GUI
      char windowTitle[80];
      sprintf (windowTitle, "%s-%s : %d", sde.sensor.c_str (), sde.camera.c_str (),  sde.pairs.at (goodPairs.at (i)));
      // Create window
      cvNamedWindow (windowTitle, 1);
#endif
    for (unsigned int j = 0; j < nrPts; j++)
    {
#if DEBUG_GUI
      // Mark the original <u,v> points with green, and the newly projected ones with red
      cvCircle (sde.images.at (goodPairs.at (i)), cvPoint ((int) cvmGet (tmpImgPt, j, 0), (int) cvmGet (tmpImgPt, j, 1)), 3, CV_RGB (0, 255, 0), 2);
      cvCircle (sde.images.at (goodPairs.at (i)), cvPoint ((int) cvmGet (projImgPoints, j, 0), (int) cvmGet (projImgPoints, j, 1)), 3, CV_RGB (255, 0, 0), 2);
#endif
      err += sqrt (
                   (cvmGet (tmpImgPt, j, 0) - cvmGet (projImgPoints, j, 0)) * (cvmGet (tmpImgPt, j, 0) - cvmGet (projImgPoints, j, 0)) +
                   (cvmGet (tmpImgPt, j, 1) - cvmGet (projImgPoints, j, 1)) * (cvmGet (tmpImgPt, j, 1) - cvmGet (projImgPoints, j, 1))
                  );
    }
    err /= (double)nrPts;

    if (err > max_err)
    {
      badPairs.push_back (sde.pairs.at (goodPairs.at (i)));
      fprintf (stderr, "ME");
    }
    fprintf (stderr, "(%.4g) ", err);

    if (err < bestErr)
    {
      bestPair        = sde.pairs.at (goodPairs.at (i));
      bestErr         = err;
      bestRotation    = tmpRotation;
      bestTranslation = tmpTranslation;
    }
#if DEBUG_GUI
    cvShowImage (windowTitle, sde.images.at (goodPairs.at (i)));
    cvWaitKey (0);                   // Wait until key pressed
    cvDestroyWindow (windowTitle);      // Destroy windows
#endif
    totalPtIdx += nrPts;
    cvReleaseMat (&projImgPoints);
  } // for (validImgPairs)
  fprintf (stderr, "\n");

  fprintf (stderr, "Best error is: "); fprintf (stderr, "%g", bestErr); fprintf (stderr, " for pair "); fprintf (stderr, "%d", bestPair); fprintf (stderr, " with: \n");
  fprintf (stderr, "bestRotation :   "); printMatrix (bestRotation);
  fprintf (stderr, "bestTranslation: "); printMatrix  (bestTranslation, 1000.0);
}

////////////////////////////////////////////////////////////////////////////////
/// Obtain the intrisic camera parameters for this entry using its associated input images
int
  computeIntrinsicParameters (SensorDataEntry &sde, CvSize checkboard, double sq_size,
                              std::vector<int> &badPairs,
                              double max_err, int wait)
{
  fprintf (stderr, "\n");
  fprintf (stderr, "Calibrating for "); fprintf (stderr, "%s-%s\n", sde.sensor.c_str (), sde.camera.c_str ());

  // Get the image size and double it
  CvSize imageSize   = cvSize (sde.images.at (0)->width, sde.images.at (0)->height);

  std::vector<int> goodPairs;
  getCalibrationCheckboardCorners (sde, checkboard, sq_size, badPairs, goodPairs);

  if (goodPairs.size () == 0)
    return (-1);

  // Calculate intrinsic camera parameters and estimate rotations and translations (extrinsic)
  sde.translations = cvCreateMat (goodPairs.size (), 3, CV_64FC1);
  sde.rotations    = cvCreateMat (goodPairs.size (), 3, CV_64FC1);

  // Finds intrinsic and extrinsic camera parameters using a calibration pattern
  /// @note: If we set CV_CALIB_USE_INTRINSIC_GUESS, we require initial values
  /// in sde.intrinsic for fx, fy, cx, cy that will be further optimized
  fprintf (stderr, "Estimating intrinsic parameters for %s-%s...\n", sde.sensor.c_str (), sde.camera.c_str ());
  cvCalibrateCamera2 (sde.objectPoints, sde.imagePoints, sde.pointNumbers,
                      imageSize, sde.intrinsic, sde.distortion, sde.rotations, sde.translations,
                      CV_CALIB_USE_INTRINSIC_GUESS);// | CV_CALIB_ZERO_TANGENT_DIST | CV_CALIB_FIX_K3 | CV_CALIB_FIX_K2);
  // Estimate the reprojection errors for each pair
  calculateIntrinsicParametersError (sde, badPairs, goodPairs, max_err);

  return goodPairs.size ();
}

////////////////////////////////////////////////////////////////////////////////
/// Obtain the extrinsic camera parameters for this entry using its associated input images
void
  computeExtrinsicParameters (SensorDataEntry &sde, CvSize checkboard, int nr_pairs)
{
  CvMat *tmpObjPt = cvCreateMatHeader (2, 2, CV_64FC1);
  CvMat *tmpImgPt = cvCreateMatHeader (2, 2, CV_64FC1);
  // Get the first nrPts from pair <i>, 3D (resObjPoints) and 2D (resImgPoints)
  cvGetSubRect (sde.objectPoints, tmpObjPt, cvRect (0, 0, 3, checkboard.width * checkboard.height));
  cvGetSubRect (sde.imagePoints,  tmpImgPt, cvRect (0, 0, 2, checkboard.width * checkboard.height));

  cvFindExtrinsicCameraParams2 (tmpObjPt, tmpImgPt, sde.intrinsic, sde.distortion,
                                sde.rotation, sde.translation);
  fprintf (stderr, "bestRotation :   "); printMatrix (sde.rotation);
  fprintf (stderr, "bestTranslation: "); printMatrix  (sde.translation, 1000.0);

  CvMat *T = cvCreateMat (4, 4, CV_64FC1);
  get4x4TransformationMatrix (sde.rotation, sde.translation, T);
  printMatrix (T);
}

#endif
