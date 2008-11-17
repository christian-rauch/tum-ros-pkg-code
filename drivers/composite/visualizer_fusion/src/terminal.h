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

#ifndef INCLUDE_COMMONTERMINALROUTINES_H
#define INCLUDE_COMMONTERMINALROUTINES_H

#include <sys/time.h>
#include <cmath>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <algorithm>
#include <string.h>
#include <vector>
#include "string_utils/string_utils.h"

#define RESET           0
#define BRIGHT          1
#define DIM             2
#define UNDERLINE       3
#define BLINK           4
#define REVERSE         7
#define HIDDEN          8
#define BLACK           0
#define RED             1
#define GREEN           2
#define YELLOW          3
#define BLUE            4
#define MAGENTA         5
#define CYAN            6
#define WHITE           7

#define MSG_OUTPUT_STREAM stderr

#define ERROR_COLOR     BRIGHT,RED
#define WARNING_COLOR   BRIGHT,BLUE
#define VALUE_COLOR     BRIGHT,YELLOW
#define INFO_COLOR      BRIGHT,GREEN

#define print_error(stream,...)		print_color(stream, ERROR_COLOR,__VA_ARGS__)
#define print_warning(stream,...)	print_color(stream, WARNING_COLOR,__VA_ARGS__)
#define print_value(stream,...)		print_color(stream, VALUE_COLOR,__VA_ARGS__)

void changeTextColor (FILE *stream, int attribute, int fg, int bg);
void changeTextColor (FILE *stream, int attribute, int fg);
void resetTextColor (FILE *stream);

void print_color (FILE *stream, int attr, int fg, const char *format, ...);
void print_info  (FILE *stream, const char *format, ...);

inline long int print_progressbar (long int counter, long int total, long int decicounter)
{
  if (counter >= decicounter * total / 10.0)
  {
    if (decicounter < 10)
      print_color (stderr, INFO_COLOR, "%ld%%-", decicounter*10);
    else
      print_color (stderr, INFO_COLOR, "%ld%% ", decicounter*10);
    decicounter++;
  }
  return decicounter;
}

std::vector<int> parseFileNamesArgument (int argc, char** argv);
int parseArgument (int argc, char** argv, const char* str, std::string &val);
int parseArgument (int argc, char** argv, const char* str, bool &val);
int parseArgument (int argc, char** argv, const char* str, double &val);
int parseArgument (int argc, char** argv, const char* str, int &val);
int parseArgument (int argc, char** argv, const char* str, unsigned int &val);
int parse3xArguments (int argc, char** argv, const char* str, double &f, double &s, double &t, bool debug = true);
int parse3xArguments (int argc, char** argv, const char* str, int &f, int &s, int &t, bool debug = true);
int parse2xArguments (int argc, char** argv, const char* str, int &f, int &s, bool debug = true);
int parseRangeArguments (int argc, char** argv, const char* str, double &s, double &e);
int parseRangeArguments (int argc, char** argv, const char* str, int &s, int &e);
bool parseMultipleArguments (int argc, char** argv, const char* str, std::vector<int> &values);

bool parseMultiple3xArguments (int argc, char** argv, const char* str, 
                               std::vector<double> &valuesF, std::vector<double> &valuesS, std::vector<double> &valuesT);

std::vector<int> parseFileExtensionArgument (int argc, char** argv, const char* ext);

#endif
