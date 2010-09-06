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

#ifndef UNICAP_DRIVER_H
#define UNICAP_DRIVER_H

#include "sensor_msgs/Image.h"

#include <stdlib.h>
#include <string.h>
#include <stdexcept>
#include <unicap/unicap.h>
#include <unicap/unicap_status.h>
#include <unicap/unicap_version.h>

namespace unicap
{
  //! Macro for defining an exception with a given parent (std::runtime_error should be top parent)
  // code borrowed from drivers/laser/hokuyo_driver/hokuyo.h
#define DEF_EXCEPTION(name, parent) \
  class name  : public parent { \
  public: \
    name (const char* msg) : parent (msg) {} \
  }
  
  //! A standard Unicap exception
  DEF_EXCEPTION(Exception, std::runtime_error);

  class FLIR
  {
    public:
      FLIR ();
      ~FLIR ();

      int open ();
      int close ();
      void readData (sensor_msgs::Image &image);

      int device_id_;
      unicap_device_t device_;
    private:
      int color_space_, video_format_;

      unicap_handle_t handle_;
      unicap_format_t format_spec_;
      unicap_format_t format_;
      unicap_data_buffer_t buffer_;
      unicap_data_buffer_t *returned_buffer_;

      bool debug_;
  };

};

#endif
