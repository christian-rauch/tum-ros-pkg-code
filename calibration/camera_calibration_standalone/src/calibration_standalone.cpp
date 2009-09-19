/*
 * Calibrate (i.e. find intrinsic and extrinsic calibration parameters) cameras
 * using left-right sets of images
 *
 * Copyright (c) 2009 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 */

#include "calibration_standalone.h"


////////////////////////////////////////////////////////////////////////////////
// Set some default values for the intrinsic parameters of one of our cameras
void
  setInitialIntrinsicParameters (std::vector<SensorDataEntry> &inputData)
{
  SensorDataEntry s;
  s.sensor = "stoc";
  s.camera = "left";
  std::vector<SensorDataEntry>::iterator it = find (inputData.begin (), inputData.end (), s);
  if (it != inputData.end ())
  {
    SensorDataEntry *pen = &(*it);
    cvmSet (pen->intrinsic, 0, 0, 1338.77747); cvmSet (pen->intrinsic, 0, 1, 0.0);     cvmSet (pen->intrinsic, 0, 2, 552.81012);
    cvmSet (pen->intrinsic, 1, 0, 0.0);     cvmSet (pen->intrinsic, 1, 1, 1338.77747); cvmSet (pen->intrinsic, 1, 2, 374.58749);
    cvmSet (pen->intrinsic, 2, 0, 0.0);     cvmSet (pen->intrinsic, 2, 1, 0.0);     cvmSet (pen->intrinsic, 2, 2, 1.0);
    cvmSet (pen->distortion, 0, 0, 0.04920);    cvmSet (pen->distortion, 0, 1, -0.20187);    cvmSet (pen->distortion, 0, 2, 0.0);  cvmSet (pen->distortion, 0, 3, 0.0);
  }

  s.sensor = "stoc";
  s.camera = "right";
  it = find (inputData.begin (), inputData.end (), s);
  if (it != inputData.end ())
  {
    SensorDataEntry *pen = &(*it);
    cvmSet (pen->intrinsic, 0, 0, 1339.43933); cvmSet (pen->intrinsic, 0, 1, 0.0);     cvmSet (pen->intrinsic, 0, 2, 503.67599);
    cvmSet (pen->intrinsic, 1, 0, 0.0);     cvmSet (pen->intrinsic, 1, 1, 1339.43933); cvmSet (pen->intrinsic, 1, 2, 394.59033);
    cvmSet (pen->intrinsic, 2, 0, 0.0);     cvmSet (pen->intrinsic, 2, 1, 0.0);     cvmSet (pen->intrinsic, 2, 2, 1.0);
    cvmSet (pen->distortion, 0, 0, -0.00809);    cvmSet (pen->distortion, 0, 1, -0.09067);    cvmSet (pen->distortion, 0, 2, 0.0);  cvmSet (pen->distortion, 0, 3, 0.0);
  }
}

/* ---[ */
int
  main (int argc, char** argv)
{
  std::vector<int> badPairs;   // this vector holds any image pair number that doesn't produce enough corners for a given sensor

  double max_admissible_err = FLT_MAX;
  if (argc < 2)
  {
    fprintf (stderr, "Syntax is: %s <file1..N.{jpg,png}*> <options>\n", argv[0]);
    fprintf (stderr, "  where options are:\n");
    fprintf (stderr, "                     -squares X,Y = use these numbers of squares to define the checkboard pattern\n");
    fprintf (stderr, "                     -sq_size X   = square size in mm\n");
    fprintf (stderr, "\n");
    fprintf (stderr, "                     -ith X   = get only every Xth pair (useful if we have over 100 pairs of recorded images)\n");
    fprintf (stderr, "\n");
    fprintf (stderr, "                     -max_err X = ignore a pair if the pixel reprojection error is larger than X (default: MAX_FLT)\n");
    fprintf (stderr, "\n");
    fprintf (stderr, "                     -wait X   = how much time to wait between each image (default : 0 - wait for a keypress)\n");
    fprintf (stderr, "\n");
    fprintf (stderr, "* note: the number of files will be parsed as folows: NR-SID-CID.png, where:\n");
    fprintf (stderr, "                     - NR  = the actual image pair number\n");
    fprintf (stderr, "                     - SID = a string identifier which defines the sensor used (e.g. stoc, sr4k, etc)\n");
    fprintf (stderr, "                     - CID = a string identifier which defines the camera used (e.g. left, right, intensity, etc)\n");
    return (-1);
  }

  // Parse the command line arguments for options
  int wait = 0;
  ParseArgument (argc, argv, "-wait", wait);
  
  ParseArgument (argc, argv, "-max_err", max_admissible_err);

  int sq_size = SQ_SIZE;
  ParseArgument (argc, argv, "-sq_size", sq_size);

  int ith = 1;
  ParseArgument (argc, argv, "-ith", ith);

  // Create the checkboard pattern
  int nr_sqr_x = NR_SQUARE_X,
      nr_sqr_y = NR_SQUARE_Y;
  ParseRangeArguments (argc, argv, "-squares", nr_sqr_x, nr_sqr_y);
  CvSize checkboard = cvSize (nr_sqr_x, nr_sqr_y);

  fprintf (stderr, "Using %d, %d with %d checkboard.\n", nr_sqr_x, nr_sqr_y, sq_size);

  // Parse the command line arguments for .png files
  std::vector<int> pPNGFileIndices;
  pPNGFileIndices = ParseFileExtensionArgument (argc, argv, ".png");

  if (pPNGFileIndices.size () == 0)
    pPNGFileIndices = ParseFileExtensionArgument (argc, argv, ".jpg");

  std::vector<SensorDataEntry> inputData = createSensorEntries (argc, argv, pPNGFileIndices, checkboard, ith);

  fprintf (stderr, "Maximum admissible pixel reprojection error for an image: "); fprintf (stderr, "%g\n", max_admissible_err);

  setInitialIntrinsicParameters (inputData);

  // Get the extrinsic parameters of each sensor model from the given input images
  int nr_valid_pairs = 0;
  for (unsigned int i = 0; i < inputData.size (); i++)
  {
    SensorDataEntry *s = &inputData.at (i);
    nr_valid_pairs = computeIntrinsicParameters (inputData.at (i), checkboard, sq_size, badPairs,
                                                 max_admissible_err, wait);
    if (nr_valid_pairs == -1)
      return (-1);

    computeExtrinsicParameters (inputData.at (i), checkboard, nr_valid_pairs);

    fprintf (stderr, " >>> INTRINSIC PARAMETERS for "); fprintf (stderr, "%s-%s\n", s->sensor.c_str (), s->camera.c_str ());
    printMatrix (s->intrinsic);
    fprintf (stderr, "   Camera Distortion Coefficients\n");
    printMatrix (s->distortion);
  }

  // Compute the extrinsic parameters between 2 sensors
  int nr_pairs = nr_valid_pairs; 
  int nums[nr_pairs];
  for (int i = 0; i < nr_pairs; i++)
    nums[i] = checkboard.height * checkboard.width;

  getExtrinsicBetween ("stoc-left", "stoc-right", checkboard, inputData, nr_pairs, nums);
  getExtrinsicBetween ("stoc-right", "stoc-left", checkboard, inputData, nr_pairs, nums);

  finishUp (inputData);
  return (0);
}
