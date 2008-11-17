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

#include "terminal.h"

////////////////////////////////////////////////////////////////////////////////
// ---[ Change the text color (on either stdout or stderr) with an attr:fg:bg
void
  changeTextColor (FILE *stream, int attribute, int fg, int bg)
{
  char command[13];
  // Command is the control command to the terminal
  sprintf (command, "%c[%d;%d;%dm", 0x1B, attribute, fg + 30, bg + 40);
  fprintf (stream, "%s", command);
}

////////////////////////////////////////////////////////////////////////////////
// ---[ Change the text color (on either stdout or stderr) with an attr:fg
void
  changeTextColor (FILE *stream, int attribute, int fg)
{
  char command[13];
  // Command is the control command to the terminal
  sprintf (command, "%c[%d;%dm", 0x1B, attribute, fg + 30);
  fprintf (stream, "%s", command);
}

////////////////////////////////////////////////////////////////////////////////
// ---[ Reset the text color (on either stdout or stderr) to its original state
void
  ResetTextColor (FILE *stream)
{
  char command[13];
  // Command is the control command to the terminal
  sprintf (command, "%c[0;m", 0x1B);
  fprintf (stream, "%s", command);
}

////////////////////////////////////////////////////////////////////////////////
// ---[ Print a message on stream with colors
void
  print_color (FILE *stream, int attr, int fg, const char *format, ...)
{
  changeTextColor (stream, attr, fg);
  va_list ap;

  va_start (ap, format);
  vfprintf (stream, format, ap);
  va_end (ap);
  
  ResetTextColor (stream);
}

////////////////////////////////////////////////////////////////////////////////
// ---[ Print an info message on stream without colors
void
  print_info (FILE *stream, const char *format, ...)
{
  changeTextColor (stream, INFO_COLOR);
  fprintf (stream, "> ");
  ResetTextColor (stream);

  va_list ap;

  va_start (ap, format);
  vfprintf (stream, format, ap);
  va_end (ap);
}


////////////////////////////////////////////////////////////////////////////////
// Parse for a specific given command line argument.
// Returns: the value sent as a string.
int
  parseArgument (int argc, char** argv, const char* str, std::string &val)
{
  for (int i = 1; i < argc; i++)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      val = std::string (argv[i]);
      return (i - 1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
// Parse for a specific given command line argument.
// Returns: the value sent as a boolean.
int
  parseArgument (int argc, char** argv, const char* str, bool &val)
{
  for (int i = 1; i < argc; i++)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      val = (bool)atoi (argv[i]);
      return (i - 1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
// Parse for a specific given command line argument.
// Returns: the value sent as a double.
int
  parseArgument (int argc, char** argv, const char* str, double &val)
{
  for (int i = 1; i < argc; i++)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      val = atof (argv[i]);
      return (i - 1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
// Parse for a specific given command line argument.
// Returns: the value sent as an int.
int
  parseArgument (int argc, char** argv, const char* str, int &val)
{
  for (int i = 1; i < argc; i++)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      val = atoi (argv[i]);
      return (i - 1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
// Parse for a specific given command line argument.
// // Returns: the value sent as an unsigned int.
int
  parseArgument (int argc, char** argv, const char* str, unsigned int &val)
{
  for (int i = 1; i < argc; i++)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      val = atoi (argv[i]);
      return (i - 1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
// Parse command line arguments for file names.
// Returns: a vector with file names indices.
std::vector<int>
  parseFileNamesArgument (int argc, char** argv)
{
  std::vector<int> indices;
  for (int i = 1; i < argc; i++)
  {
    std::string fname = std::string (argv[i]);
    
    // Needs to be at least 4: .ext
    if (fname.size () <= 4)
      continue;
      
    // For being case insensitive
    std::transform (fname.begin(), fname.end(), fname.begin(), tolower);
    
    // if OBJ
    if (fname.compare (fname.size () - 3, 3, "obj") == 0)
      indices.push_back (i);
    // or PLY
    else if (fname.compare (fname.size () - 3, 3, "ply") == 0)
      indices.push_back (i);
    // or NCXYZ
    else if (fname.compare (fname.size () - 5, 5, "ncxyz") == 0)
      indices.push_back (i);
    // or CXYZ
    else if (fname.compare (fname.size () - 4, 4, "cxyz") == 0)
      indices.push_back (i);
    // or NXYZ
    else if (fname.compare (fname.size () - 4, 4, "nxyz") == 0)
      indices.push_back (i);
    // or MESH
    else if (fname.compare (fname.size () - 4, 4, "mesh") == 0)
      indices.push_back (i);
    // or XYZR
    else if (fname.compare (fname.size () - 4, 4, "xyzr") == 0)
      indices.push_back (i);
    // or PVTP
    else if (fname.compare (fname.size () - 4, 4, "pvtp") == 0)
      indices.push_back (i);
    // or XYZ
    else if (fname.compare (fname.size () - 3, 3, "xyz") == 0)
      indices.push_back (i);
    // or VTK
    else if (fname.compare (fname.size () - 3, 3, "vtk") == 0)
      indices.push_back (i);
    // or VTP
    else if (fname.compare (fname.size () - 3, 3, "vtp") == 0)
      indices.push_back (i);
  }
  return indices;
}

////////////////////////////////////////////////////////////////////////////////
// Parse command line arguments for file names.
// Returns: a vector with file names indices.
std::vector<int>
  parseFileExtensionArgument (int argc, char** argv, const char *extension)
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
    std::transform(fname.begin(), fname.end(), fname.begin(), tolower);
    std::transform(ext.begin(), ext.end(), ext.begin(), tolower);
    
    // Check if found
    if (fname.find (ext) != std::string::npos)
      indices.push_back (i);
  }
  return indices;
}

////////////////////////////////////////////////////////////////////////////////
// Parse for specific given command line arguments (3x values comma separated).
// Returns: the values sent as doubles.
int
  parse3xArguments (int argc, char** argv, const char* str, double &f, double &s, double &t, bool debug)
{
  for (int i = 1; i < argc; i++)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      // look for ',' as a separator
      std::vector<std::string> values;
      string_utils::split (argv[i], values, ",");
      if ((values.size () != 3) && (debug))
      {
        print_error (stderr, "Number of values for %s different than 3!\n", str);
        return (-2);
      }
      f = atof (values.at (0).c_str ());
      s = atof (values.at (1).c_str ());
      t = atof (values.at (2).c_str ());
      return (i - 1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
// Parse for specific given command line arguments (3x values comma separated).
// Returns: the values sent as ints.
int
  parse3xArguments (int argc, char** argv, const char* str, int &f, int &s, int &t, bool debug)
{
  for (int i = 1; i < argc; i++)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      // look for ',' as a separator
      std::vector<std::string> values;
      string_utils::split (argv[i], values, ",");
      if ((values.size () != 3) && (debug))
      {
        print_error (stderr, "Number of values for %s different than 3!\n", str);
        return (-2);
      }
      f = atoi (values.at (0).c_str ());
      s = atoi (values.at (1).c_str ());
      t = atoi (values.at (2).c_str ());
      return (i - 1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
// Parse for specific given command line arguments (2x values comma separated).
// Returns: the values sent as ints.
int
  parse2xArguments (int argc, char** argv, const char* str, int &f, int &s, bool debug)
{
  for (int i = 1; i < argc; i++)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      // look for ',' as a separator
      std::vector<std::string> values;
      string_utils::split (argv[i], values, ",");
      if ((values.size () != 2) && (debug))
      {
        print_error (stderr, "Number of values for %s different than 2!\n", str);
        return (-2);
      }
      f = atoi (values.at (0).c_str ());
      s = atoi (values.at (1).c_str ());
      return (i - 1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
// Parse for specific given command line arguments (range values X-Y).
// Returns: the values sent as doubles.
int
  parseRangeArguments (int argc, char** argv, const char* str, double &s, double &e)
{
  for (int i = 1; i < argc; i++)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      // look for ',' as a separator
      std::vector<std::string> values;
      string_utils::split (argv[i], values, ",");
      if (values.size () != 2)
        print_error (stderr, "Number of values for %s different than 2!\n", str);
      s = atof (values.at (0).c_str ());
      e = atof (values.at (1).c_str ());
      return (i - 1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
// Parse for specific given command line arguments (range values X-Y).
// Returns: the values sent as ints.
int
  parseRangeArguments (int argc, char** argv, const char* str, int &s, int &e)
{
  for (int i = 1; i < argc; i++)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      // look for ',' as a separator
      std::vector<std::string> values;
      string_utils::split (argv[i], values, ",");
      if (values.size () != 2)
        print_error (stderr, "Number of values for %s different than 2!\n", str);
      s = atoi (values.at (0).c_str ());
      e = atoi (values.at (1).c_str ());
      return (i - 1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
// Parse for specific given command line arguments (multiple occurances of the
// same command line parameter).
// Returns: the values sent as a vector.
bool
  parseMultipleArguments (int argc, char** argv, const char* str, std::vector<int> &values)
{
  for (int i = 1; i < argc; i++)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      int val = atoi (argv[i]);
      values.push_back (val);
    }
  }
  if (values.size () == 0)
    return (false);
  else
    return (true);
}

////////////////////////////////////////////////////////////////////////////////
// Parse for specific given command line arguments (multiple occurances of 3x 
// argument groups, separated by commas)
// Returns: 3 vectors holding the given values
bool
  parseMultiple3xArguments (int argc, char** argv, const char* str,
                            std::vector<double> &valuesF, std::vector<double> &valuesS, std::vector<double> &valuesT)
{
  double f, s, t;
  for (int i = 1; i < argc; i++)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      // look for ',' as a separator
      std::vector<std::string> values;
      string_utils::split (argv[i], values, ",");
      if (values.size () != 3)
        print_error (stderr, "Number of values for %s different than 3!\n", str);
      f = atof (values.at (0).c_str ());
      s = atof (values.at (1).c_str ());
      t = atof (values.at (2).c_str ());
      valuesF.push_back (f);
      valuesS.push_back (s);
      valuesT.push_back (t);
    }
  }
  if (valuesF.size () == 0)
    return (false);
  else
    return (true);
}
