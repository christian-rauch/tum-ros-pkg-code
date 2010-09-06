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

#include "flir.h"

//! Macro for throwing an exception with a message
#define FLIR_EXCEPT(except, msg) \
  { \
    char buf[100]; \
    snprintf(buf, 100, "[FLIR::%s]: " msg, __FUNCTION__); \
    throw except(buf); \
  }

//! Macro for throwing an exception with a message, passing args
#define FLIR_EXCEPT_ARGS(except, msg, ...) \
  { \
    char buf[100]; \
    snprintf(buf, 100, "[FLIR::%s]: " msg, __FUNCTION__, __VA_ARGS__); \
    throw except(buf); \
  }

unicap::FLIR::FLIR () : device_id_(1), color_space_(0), video_format_(0), debug_(false)
{
}

unicap::FLIR::~FLIR ()
{
}

////////////////////////////////////////////////////////////////////////////////
// Set up the device
int
  unicap::FLIR::open ()
{
  // Device ID number
  if (!SUCCESS (unicap_enumerate_devices (NULL, &device_, device_id_)))
    FLIR_EXCEPT_ARGS(unicap::Exception, "Could not get info for device %i!", device_id_);

  if ( !SUCCESS (unicap_open (&handle_, &device_)) )
  {
    FLIR_EXCEPT_ARGS(unicap::Exception, "Failed to open camera port %s!", device_.identifier);
    return (-1);
  }

  // Set the desired format
  unicap_void_format (&format_spec_);
//  format_spec.fourcc = FOURCC('U','Y','V','Y');

  // Get the list of video formats of the given colorformat
  if (debug_)
  {
    fprintf (stderr, "[CompositeNode::FLIR] Available color spaces:");
    for ( int i = 0; SUCCESS (unicap_enumerate_formats (handle_, &format_spec_, &format_, i)); i++)
      fprintf (stderr, " %d(%s) ", i, format_.identifier);
    fprintf (stderr, "\n");
  }

  if (!SUCCESS (unicap_enumerate_formats (handle_, &format_spec_, &format_, color_space_) ) )
  {
    FLIR_EXCEPT_ARGS(unicap::Exception, "Failed to set color space to %d!", color_space_);
    return (-1);
  }

  // If a video format has more than one size, ask for which size to use
  if (format_.size_count)
  {
    if (debug_)
    {
      fprintf (stderr, "[CompositeNode::FLIR] Available video formats:");
       for (int i = 0; i < format_.size_count; i++)
         fprintf (stderr, " %d(%dx%d) ", i, format_.sizes[i].width, format_.sizes[i].height);
      fprintf (stderr, "\n");
    }
    format_.size.width = format_.sizes[video_format_].width;
    format_.size.height = format_.sizes[video_format_].height;
  }

  if (!SUCCESS (unicap_set_format (handle_, &format_) ) )
  {
    FLIR_EXCEPT_ARGS(unicap::Exception, "Failed to set video format to %d!", video_format_);
    return (-1);
  }

  // Start the capture process on the device
  if (!SUCCESS (unicap_start_capture (handle_) ) )
  {
    FLIR_EXCEPT_ARGS(unicap::Exception, "Failed to capture on device: %s!", device_.identifier);
    return (-1);
  }

  // Initialize the image buffer
  memset (&buffer_,   0, sizeof (unicap_data_buffer_t));

  // Allocate buffer data
  buffer_.data = (unsigned char*)(malloc (format_.size.width * format_.size.height * format_.bpp / 8));
  buffer_.buffer_size = format_.size.width * format_.size.height * format_.bpp / 8;

  return (0);
}

////////////////////////////////////////////////////////////////////////////////
// Shutdown the device
int
  unicap::FLIR::close ()
{
  // Stop the device
  if ( !SUCCESS (unicap_stop_capture (handle_) ) )
    FLIR_EXCEPT_ARGS(unicap::Exception, "Failed to stop capture on device: %s!", device_.identifier);

  free (buffer_.data);

  // Close the device
  if ( !SUCCESS (unicap_close (handle_) ) )
  {
    FLIR_EXCEPT_ARGS(unicap::Exception, "Failed to close device: %s!", device_.identifier);
    return (-1);
  }

  return (0);
}

////////////////////////////////////////////////////////////////////////////////
// Read data from the device
void
//  unicap::FLIR::readData (camera_data_t* data)
  unicap::FLIR::readData (sensor_msgs::Image &image)
{
  // Queue the buffer
  // The buffer now gets filled with image data by the capture device
  if (!SUCCESS (unicap_queue_buffer (handle_, &buffer_) ) )
    return;

  // Wait until the image buffer is ready
  if (!SUCCESS (unicap_wait_buffer (handle_, &returned_buffer_) ) ) {}
//    return;

  // To do: implement the code for different formats later
  image.width  = buffer_.format.size.width;
  image.height = buffer_.format.size.height;
  image.encoding = "raw";
  //  image.colorspace  = "mono16";
  // image.label       = "flir-thermal";
  //  image.set_data_size (buffer_.buffer_size);
  memcpy (&(image.data[0]), buffer_.data, buffer_.buffer_size);  

  return;
}
