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

#include "std_msgs/PointCloud.h"

//#include "StringTokenizer.h"
#include <stdlib.h>
#include "string_utils/string_utils.h"

#include <fstream>

using namespace std;

class PCDGenerator: public ros::node
{
  public:

    // ROS messages
    std_msgs::PointCloud msg_cloud_;

    PCDGenerator () : ros::node ("PCD_generator")
    {
      // Maximum number of outgoing messages to be queued for delivery to subscribers = 1
      advertise<std_msgs::PointCloud>("cloud_stoc", 1);
    }

    ~PCDGenerator ()
    {
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    // Load point cloud data from a PCD file containing n-D points
    /// @note: all lines besides:
    // - the ones beginning with # (treated as comments
    // - COLUMNS ...
    // - POINTS ...
    // - DATA ...
    // ...are intepreted as data points. Failure to comply with this simple rule might result in errors and warnings! :)
    std_msgs::PointCloud
      LoadPointCloud (const char* fileName)
    {
      std_msgs::PointCloud cloud;
      
      int nr_points = 0;
      std::ifstream fs;
      std::string line;

      int idx = 0;
      // Open file
      fs.open (fileName);
      if (!fs.is_open ())
      {
        fprintf (stderr, "Couldn't open %s for reading!\n", fileName);
        return cloud;
      }
      // Read the header and fill it in with wonderful values
      while (!fs.eof ())
      {
        getline (fs, line);
        if (line == "")
          continue;
          
        vector<string> st;
        string_utils::split (line, st, " ");
//        StringTokenizer st = StringTokenizer (line, " ");

//        std::string lineType = st.nextToken ();
        std::string lineType = st.at (0);

        /// ---[ Perform checks to see what does this line represents
        if (lineType.substr (0, 1) == "#")
          continue;
        // Get the column indices
        if (lineType.substr (0, 7) == "COLUMNS")
        {
//          int remainingTokens = st.countTokens ();
          int remainingTokens = st.size () - (1 + 3);
//          st.nextToken (); st.nextToken (); st.nextToken ();            // x-y-z
          
//          cloud.set_chan_size (remainingTokens - 3);
          cloud.set_chan_size (remainingTokens);

//          for (int i = 0; i < remainingTokens - 3; i++)
          for (int i = 0; i < remainingTokens; i++)
          {
//            std::string colType = st.nextToken ();
            std::string colType = st.at (i + 4);
            cloud.chan[i].name = colType;
          }
          continue;
        }
        // Get the number of points
        if (lineType.substr (0, 6) == "POINTS")
        {
//          nr_points = st.nextIntToken ();
          nr_points = atoi (st.at (1).c_str ());
          cloud.set_pts_size (nr_points);
          
          for (unsigned int d = 0; d < cloud.get_chan_size (); d++)
            cloud.chan[d].set_vals_size (nr_points);
          
          continue;
        }

        // Check DATA type
        if (lineType.substr (0, 4) == "DATA")
          continue;

        // Nothing of the above? We must have points then
        // Convert the first token to float and use it as the first point coordinate
        if (idx >= nr_points)
        {
          fprintf (stderr, "Error: input file %s has more points than advertised (%d)!\n", fileName, nr_points);
          break;
        }
        
        // Assume x-y-z to be the first dimensions in the file
        // @todo: check and improve this for errors
//        cloud.pts[idx].x = atof (lineType.c_str ());
//        cloud.pts[idx].y = st.nextFloatToken ();
//        cloud.pts[idx].z = st.nextFloatToken ();
        cloud.pts[idx].x = atof (st.at (0).c_str ());
        cloud.pts[idx].y = atof (st.at (1).c_str ());
        cloud.pts[idx].z = atof (st.at (2).c_str ());
        for (unsigned int i = 0; i < cloud.get_chan_size (); i++)
//          cloud.chan[i].vals[idx] = st.nextFloatToken ();
          cloud.chan[i].vals[idx] = atof (st.at (i+3).c_str ());

        idx++;
      }
      // Close file
      fs.close ();

      if (idx != nr_points)
        fprintf (stderr, "Warning! Number of points read (%d) is different than expected (%d)\n", idx, nr_points);
        
      return cloud;
    }
    

    ////////////////////////////////////////////////////////////////////////////////
    // Start
    int
      start ()
    {
      msg_cloud_ = LoadPointCloud ("stoc.pcd");
      return (0);
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Spin (!)
    bool spin ()
    {
      while (ok ())
      {
        usleep (100000);
        
        publish ("cloud_stoc", msg_cloud_);
      }

      return true;
    }

  
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);

  PCDGenerator c;

  c.start ();
  c.spin ();

  ros::fini ();

  return (0);
}
/* ]--- */
