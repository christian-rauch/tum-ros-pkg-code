
/*
 * Copyright (c) 2011, Thomas Ruehr <ruehr@cs.tum.edu>
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
 *     * Neither the name of Willow Garage, Inc. nor the names of its
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

//! Keywords.h - create and pass multiple <string, value> maps conveniently in a association-list style.
//! Defines a type for passing key/value pairs as function parameters in a compact way.
//! the () operator is overloaded in a way to collect multiple key/value pairs.

//! Usage example:
//void test_func(Keywords &kp)
//{
//    ROS_INFO("string %s",kp.lookup_s("blub").c_str());
//    ROS_INFO("double %f",kp.lookup_d("blub"));
//    ROS_INFO("vec %f %f %f",kp.lookup_v("blic").x(),kp.lookup_v("blic").y(),kp.lookup_v("blic").z());
//}
// ...
// test_func(Keywords() ("bla",1) ("blub","blib") ("tic", 5) ("blub",7) ("blic",tf::Vector3(0,0,1)) );
// or
// test_func(Keywords("bla",1) ("blub","blib") ("tic", 5) ("blub",7) ("blic",tf::Vector3(0,0,1)) );

#ifndef __KEYWORDS_H__
#define __KEYWORDS_H__

#include <tf/tf.h>
#include <string>

#include <map>

typedef struct {
    std::string s;
    tf::Vector3 v;
    double d;

    operator std::string() {return s;}
    operator tf::Vector3() {return v;}
    operator double() {return d;}
} KeyValPair;



class Keywords
{
public:

    Keywords() {};

    Keywords(std::string name, double val);

    Keywords &operator()(std::string name, double val);

    Keywords(std::string name, std::string val);

    Keywords &operator()(std::string name, std::string val);

    Keywords(std::string name, tf::Vector3 val);

    Keywords &operator()(std::string name, tf::Vector3 val);

    Keywords &contains();

    //check if key is existing
    bool has_d(const std::string &key) const;
    bool has_s(const std::string &key) const;
    bool has_v(const std::string &key) const;

    //lookup, returns value when key is found else 0/0 vector/empty string
    double lookup_d(const std::string &key) const;
    std::string lookup_s(const std::string &key) const;
    tf::Vector3 lookup_v(const std::string &key) const;

    KeyValPair lookup(const std::string &key) const;

  // one map for each type, allow to have multiple entries with the same key but different value type
    std::map<std::string, double> parameters_d;
    std::map<std::string, std::string> parameters_s;
    std::map<std::string, tf::Vector3> parameters_v;

};

#endif
