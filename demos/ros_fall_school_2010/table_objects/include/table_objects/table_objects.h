/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

/**
  * \author Radu Bogdan Rusu 
  *
  * @b nearest_neighbors loads a FLANN VFH kd-tree from disk, together with a
  * list of model names, and returns the closest K neighbors to a given query
  * model/VFH signature.
  */

#ifndef TABLE_OBJECTS_H_
#define TABLE_OBJECTS_H_

#include <boost/filesystem.hpp>

#include <fstream>
#include <sstream>
#include <iostream>
#include <utility>
#include <cctype>

#include <flann/flann.h>
#include <flann/io/hdf5.h>

#include <terminal_tools/parse.h>
#include <terminal_tools/print.h>

#include <pcl/common/common.h>
#include <pcl/features/feature.h>
#include <pcl/registration/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace pcl;
using namespace terminal_tools;

typedef std::pair<std::string, std::vector<float> > vfh_model;

int metric;
std::string kdtree_idx_file_name;
std::string training_data_h5_file_name;
std::string training_data_list_file_name;
std::vector<vfh_model> models;

////////////////////////////////////////////////////////////////////////////////
/** \brief Load the list of file model names from an ASCII file
  * \param models the resultant list of model name
  * \param filename the input file name
  */
bool
  loadFileList (vector<vfh_model> &models, const string &filename)
{
  ifstream fs;
  fs.open (filename.c_str ());
  if (!fs.is_open () || fs.fail ())
    return (false);

  string line;
  while (!fs.eof ())
  {
    getline (fs, line);
    if (line.empty ())
      continue;
    vfh_model m;
    m.first = line;
    models.push_back (m);
  }
  fs.close ();
  return (true);
}

int
  getParameters (int argc, char** argv)
{
  // Set the tree metric type
  metric = 7;
  parse_argument (argc, argv, "-metric", metric);
  if (metric < 0 || metric > 7)
  {
    print_error ("Invalid metric specified (%d)!\n", metric);
    return (-1);
  }
  flann_set_distance_type ((flann_distance_t)metric, 0);
  print_highlight ("Using distance metric = "); print_value ("%d\n", metric); 

  // --[ Read the kdtree index file
  kdtree_idx_file_name = "kdtree.idx";
  vector<int> idx_indices = parse_file_extension_argument (argc, argv, ".idx");
  if (idx_indices.size () > 1)
  {
    print_error ("Need a single kdtree index file!\n");
    return (-1);
  }
  if (idx_indices.size () == 1)
    kdtree_idx_file_name = argv[idx_indices.at (0)];
  // Check if the tree index has already been saved to disk
  if (!boost::filesystem::exists (kdtree_idx_file_name))
  {
    print_error ("Could not find kd-tree index in file %s!", kdtree_idx_file_name.c_str ());
    return (-1);
  }
  print_highlight ("Using "); print_value ("%s", kdtree_idx_file_name.c_str ()); print_info (" as the kdtree index file.\n");

  // --[ Read the training data h5 file
  training_data_h5_file_name = "training_data.h5";
  vector<int> train_h5_indices = parse_file_extension_argument (argc, argv, ".h5");
  if (train_h5_indices.size () > 1)
  {
    print_error ("Need a single h5 training data file!\n");
    return (-1);
  }
  if (train_h5_indices.size () == 1)
    training_data_h5_file_name = argv[train_h5_indices.at (0)];
  print_highlight ("Using "); print_value ("%s", training_data_h5_file_name.c_str ()); print_info (" as the h5 training data file.\n");

  // --[ Read the training data list file
  training_data_list_file_name = "training_data.list";
  vector<int> train_list_indices = parse_file_extension_argument (argc, argv, ".list");
  if (train_list_indices.size () > 1)
  {
    print_error ("Need a single list training data file!\n");
    return (-1);
  }
  if (train_list_indices.size () == 1)
    training_data_list_file_name = argv[train_list_indices.at (0)];
  print_highlight ("Using "); print_value ("%s", training_data_list_file_name.c_str ()); print_info (" as the list training data file.\n");

  // Check if the data has already been saved to disk
  if (!boost::filesystem::exists (training_data_h5_file_name) || !boost::filesystem::exists (training_data_list_file_name))
  {
    print_error ("Could not find training data models files %s and %s!\n", training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
    return (-1);
  }
  
  return (1);
}

#endif
