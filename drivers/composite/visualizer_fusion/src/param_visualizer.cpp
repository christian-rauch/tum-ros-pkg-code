/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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
 */

/* author: Radu Bogdan Rusu <rusu@cs.tum.edu> */

// ROS core
#include "ros/node_handle.h"
#include "ros/time.h"
#include "ros/common.h"

#include <cfloat>
#include "terminal.h"

#define BOOL2STR_ED(b) ( b ? "enabled" : "disabled" )

using namespace std;

class ParamVisualizer: public ros::node
{
  public:
    ParamVisualizer () : ros::node ("param_visualizer") { }
    ~ParamVisualizer () { }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  if (argc < 2)
  {
    print_error (stderr, "Syntax is: %s [options]\n", argv[0]);
    fprintf (stderr, "where options are:\n");
    fprintf (stderr, "                   -sr_ai X = SwissRanger <Auto-Illumination> parameter\n");
    fprintf (stderr, "                   -sr_it X = SwissRanger <Integration time> parameter\n");
    fprintf (stderr, "                   -sr_mf X = SwissRanger <Modulation frequency> parameter\n");
    fprintf (stderr, "                   -sr_at X = SwissRanger <Amplitude threshold> parameter\n");
    fprintf (stderr, "\n");
    fprintf (stderr, "                   -stoc_sd X = STOC <Minimum disparity region size> parameter\n");
    fprintf (stderr, "                   -stoc_ss X = STOC <Disparity region neighbor diff> parameter\n");
    fprintf (stderr, "                   -stoc_ho X = STOC <Horopter (X Offset)> parameter\n");
    fprintf (stderr, "                   -stoc_cs X = STOC <Correlation window size> parameter\n");
    fprintf (stderr, "                   -stoc_un X = STOC <Uniqueness filter threshold> parameter\n");
    fprintf (stderr, "                   -stoc_tt X = STOC <Texture filter threshold> parameter\n");
    fprintf (stderr, "                   -stoc_nd X = STOC <Number of disparities> parameter\n");
    fprintf (stderr, "                   -stoc_zm X = STOC <Z-max cutoff> parameter\n");
    fprintf (stderr, "\n");
    fprintf (stderr, "                   -sr_rd   X = SwissRanger <Rectify Distance Image> parameter\n");
    fprintf (stderr, "                   -sr_ra   X = SwissRanger <Rectify Amplitude Image> parameter\n");
    fprintf (stderr, "                   -sr_rc   X = SwissRanger <Rectify Confidence Image> parameter\n");
    fprintf (stderr, "                   -sr_fpcd X = SwissRanger <Filter PCD> parameter\n");
    return (-1);
  }
  
  int sr_ai = INT_MIN, sr_it = INT_MIN, sr_mf = INT_MIN, sr_at = INT_MIN;
  int stoc_sd = INT_MIN, stoc_ss = INT_MIN, stoc_ho = INT_MIN, stoc_cs = INT_MIN, stoc_un = INT_MIN, stoc_tt = INT_MIN, stoc_nd = INT_MIN;
  double stoc_zm = DBL_MIN;
  
  parseArgument (argc, argv, "-sr_ai", sr_ai);
  parseArgument (argc, argv, "-sr_it", sr_it);
  parseArgument (argc, argv, "-sr_mf", sr_mf);
  parseArgument (argc, argv, "-sr_at", sr_at);

  parseArgument (argc, argv, "-stoc_sd", stoc_sd);
  parseArgument (argc, argv, "-stoc_ss", stoc_ss);
  parseArgument (argc, argv, "-stoc_ho", stoc_ho);
  parseArgument (argc, argv, "-stoc_cs", stoc_cs);
  parseArgument (argc, argv, "-stoc_un", stoc_un);
  parseArgument (argc, argv, "-stoc_tt", stoc_tt);
  parseArgument (argc, argv, "-stoc_nd", stoc_nd);
  parseArgument (argc, argv, "-stoc_zm", stoc_zm);
  
  ros::init (argc, argv);

  // Here is where we create our ROS node
  ParamVisualizer p;
  
  // Swissranger related settings
  if (sr_ai != INT_MIN)
  {
    print_info (stderr, "Changing the <Auto Illumination> settings on the Swissranger camera to: "); print_value (stderr, "%d\n", sr_ai);
    p.set_param ("/composite/sr_auto_illumination", sr_ai);
  }
  if (sr_it != INT_MIN)
  {
    print_info (stderr, "Changing the <Integration time> settings on the Swissranger camera to: "); print_value (stderr, "%d\n", sr_it);
    p.set_param ("/composite/sr_integration_time", sr_it);
  }
  if (sr_mf != INT_MIN)
  {
    print_info (stderr, "Changing the <Modulation frequency> settings on the Swissranger camera to: "); print_value (stderr, "%d\n", sr_mf);
    p.set_param ("/composite/sr_modulation_freq", sr_mf);
  }
  if (sr_at != INT_MIN)
  {
    print_info (stderr, "Changing the <Amplitude threshold> settings on the Swissranger camera to: "); print_value (stderr, "%d\n", sr_at);
    p.set_param ("/composite/sr_amp_threshold", sr_at);
  }
  
  // STOC related settings
  if (stoc_sd != INT_MIN)
  {
    print_info (stderr, "Changing the <Minimum disparity region size> settings on the STOC camera to: "); print_value (stderr, "%d\n", stoc_sd);
    p.set_param ("/composite/stoc_speckle_size", stoc_sd);
  }
  if (stoc_ss != INT_MIN)
  {
    print_info (stderr, "Changing the <Disparity region neighbor diff> settings on the STOC camera to: "); print_value (stderr, "%d\n", stoc_ss);
    p.set_param ("/composite/stoc_speckle_diff", stoc_ss);
  }
  if (stoc_ho != INT_MIN)
  {
    print_info (stderr, "Changing the <Horopter (X Offset)> settings on the STOC camera to: "); print_value (stderr, "%d\n", stoc_ho);
    p.set_param ("/composite/stoc_horopter", stoc_ho);
  }
  if (stoc_cs != INT_MIN)
  {
    print_info (stderr, "Changing the <Correlation window size> settings on the STOC camera to: "); print_value (stderr, "%d\n", stoc_cs);
    p.set_param ("/composite/stoc_corrsize", stoc_cs);
  }
  if (stoc_un != INT_MIN)
  {
    print_info (stderr, "Changing the <Uniqueness filter threshold> settings on the STOC camera to: "); print_value (stderr, "%d\n", stoc_un);
    p.set_param ("/composite/stoc_unique", stoc_un);
  }
  if (stoc_tt != INT_MIN)
  {
    print_info (stderr, "Changing the <Texture filter threshold> settings on the STOC camera to: "); print_value (stderr, "%d\n", stoc_tt);
    p.set_param ("/composite/stoc_tex_thresh", stoc_tt);
  }
  if (stoc_nd != INT_MIN)
  {
    print_info (stderr, "Changing the <Number of disparities> settings on the STOC camera to: "); print_value (stderr, "%d\n", stoc_nd);
    p.set_param ("/composite/stoc_ndisp", stoc_nd);
  }
  if (stoc_zm != DBL_MIN)
  {
    print_info (stderr, "Changing the <Z-max cutoff> settings on the STOC camera to: "); print_value (stderr, "%g\n", stoc_zm);
    p.set_param ("/composite/stoc_zmax", stoc_zm);
  }
  
  
  int sr_rd, sr_ra, sr_rc, sr_fpcd;
  if (parseArgument (argc, argv, "-sr_rd", sr_rd) != -1)
  {
    print_info (stderr, "Changing the <Rectify Distance Image> settings on the SwissRanger camera to: "); print_value (stderr, "%s\n", BOOL2STR_ED (sr_rd));
    p.set_param ("/composite/sr_rectify_distance", sr_rd);
  }
  
  if (parseArgument (argc, argv, "-sr_ra", sr_ra) != -1)
  {
    print_info (stderr, "Changing the <Rectify Amplitude Image> settings on the SwissRanger camera to: "); print_value (stderr, "%s\n", BOOL2STR_ED (sr_ra));
    p.set_param ("/composite/sr_rectify_amplitude", sr_ra);
  }
  if (parseArgument (argc, argv, "-sr_ra", sr_rc) != -1)
  {
    print_info (stderr, "Changing the <Rectify Confidence Image> settings on the SwissRanger camera to: "); print_value (stderr, "%s\n", BOOL2STR_ED (sr_rc));
    p.set_param ("/composite/sr_rectify_confidence", sr_rc);
  }
  if (parseArgument (argc, argv, "-sr_fpcd", sr_fpcd) != -1)
  {
    print_info (stderr, "Changing the <Filter PCD> settings on the SwissRanger camera to: "); print_value (stderr, "%s\n", BOOL2STR_ED (sr_fpcd));
    p.set_param ("/composite/sr_pcd_filter", sr_fpcd);
  }
  
  // Shutdown
  p.shutdown ();

  ros::fini ();

  return (0);
}
/* ]--- */
