/*
 * Copyright (c) 2008 Dejan Pangercic <pangercic -=- cs.tum.edu>
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
 * $Id: file_io.h 21045 2009-08-07 20:52:44Z dejanpan $
 *
 */

/** \author Dejan Pangercic */

#ifndef _PLAYER_LOG_ACTARRAY_FILE_IO_H_
#define _PLAYER_LOG_ACTARRAY_FILE_IO_H_


#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include "boost/filesystem.hpp"

using namespace std;
using namespace boost::filesystem; 

namespace player_log_actarray
{
  
  namespace file_io
  {
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Get a list of log files from a directory
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void get_log_files ( const path & directory, vector <string> &file_list, string suffix=".log", bool recurse_into_subdirs = false )
    {
      if( exists( directory ) )
	{
	  directory_iterator end ;
	  for( directory_iterator iter(directory) ; iter != end ; ++iter )
	    if ( is_directory( *iter ) )
	      {
		ROS_WARN("Directory %s", iter->string().c_str());
		//if( recurse_into_subdirs ) get_log_files(*iter) ;
	      }
	    else 
	      {
		int len = iter->string().length();
		int npos =  iter->string().rfind(suffix);
		if((len - npos) == suffix.length())
		  file_list.push_back(iter->string());
	      }
	}
      ROS_INFO("Found nr %d of files", file_list.size());
      
    }
  }
}
#endif
