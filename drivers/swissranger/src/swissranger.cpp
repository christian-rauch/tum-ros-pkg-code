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

#include "swissranger.h"

//! Macro for throwing an exception with a message
#define SR_EXCEPT(except, msg) \
  { \
    char buf[100]; \
    snprintf(buf, 100, "[SwissRanger::%s]: " msg, __FUNCTION__); \
    throw except(buf); \
  }

//! Macro for throwing an exception with a message, passing args
#define SR_EXCEPT_ARGS(except, msg, ...) \
  { \
    char buf[100]; \
    snprintf(buf, 100, "[SwissRanger::%s]: " msg, __FUNCTION__, __VA_ARGS__); \
    throw except(buf); \
  }

swissranger::SwissRanger::SwissRanger () : srCam_ (NULL)
{
}

swissranger::SwissRanger::~SwissRanger ()
{
}

////////////////////////////////////////////////////////////////////////////////
// Set up the device
int
  swissranger::SwissRanger::open ()
{
  // ---[ Open the camera ]---
  int res = SR_OpenUSB (&srCam_, 0);          //returns the device ID used in other calls
  if (res <= 0)
  {
    SR_EXCEPT(swissranger::Exception, "Failed to open camera port!");
    return (-1);
  }
  
  device_id_   = getDeviceString ();
  lib_version_ = getLibraryVersion ();
  // ---[ Get the number of rows, cols, ... ]---
  rows_ = SR_GetRows (srCam_);
  cols_ = SR_GetCols (srCam_);
  inr_  = SR_GetImageList (srCam_, &imgEntryArray_);
  ROS_INFO ("[SwissRanger::open] Number of images available: %d", inr_);
  modulation_freq_  = SR_GetModulationFrequency (srCam_);
  integration_time_ = SR_GetIntegrationTime (srCam_);

  if ( (cols_ != SR_COLS) || (rows_ != SR_ROWS) || (inr_ < 1) || (imgEntryArray_ == 0) )
  {
    SR_Close (srCam_);
    SR_EXCEPT_ARGS(swissranger::Exception, "Invalid data image (%dx%d) received from camera!", cols_, rows_);
    return (-1);
  }

  // ---[ Set the acquisition mode ]---
  SR_SetMode (srCam_, MODE);

  // Points array
  size_t buffer_size = rows_ * cols_ * 3 * sizeof (float);
  buffer_ = (float*)malloc (buffer_size);
  memset (buffer_, 0xaf, buffer_size);

  xp_ = buffer_;
  yp_ = &xp_[rows_*cols_];
  zp_ = &yp_[rows_*cols_];

  return (0);

}

////////////////////////////////////////////////////////////////////////////////
// Shutdown the device
int
  swissranger::SwissRanger::close ()
{
  if (srCam_ == NULL)
    return (-1);

  // ---[ Close the camera ]---
  int res = SR_Close (srCam_);

  // ---[ Free the allocated memory buffer ]---
  free (buffer_);

  if (res != 0)
    return (-1);
  return (0);
}

////////////////////////////////////////////////////////////////////////////////
// Read data from the device
void
  swissranger::SwissRanger::readData (robot_msgs::PointCloud &cloud, deprecated_msgs::ImageArray &images)
{
  if (srCam_ == NULL)
    SR_EXCEPT(swissranger::Exception, "Read attempted on NULL camera port!");

  int res;

  inr_  = SR_GetImageList (srCam_, &imgEntryArray_);
  res = SR_Acquire (srCam_);

  size_t image_size = imgEntryArray_->width * imgEntryArray_->height * 2;

  // Pointers to data
  uint8_t *distance_image   = (unsigned char*)SR_GetImage (srCam_, 0);//(unsigned char*)imgEntryArray_->data;
  uint8_t *amplitude_image  = (unsigned char*)SR_GetImage (srCam_, 1);//(unsigned char*)imgEntryArray_->data + (image_size + sizeof (ImgEntry));
  uint8_t *confidence_image = (unsigned char*)SR_GetImage (srCam_, 2);//(unsigned char*)imgEntryArray_->data + (image_size + sizeof (ImgEntry)) * 2;
  
  // Points array
  res = SR_CoordTrfFlt (srCam_, xp_, yp_, zp_, sizeof (float), sizeof (float), sizeof (float));

  // Filter points
  cloud.set_pts_size (imgEntryArray_->width * imgEntryArray_->height);
  cloud.set_chan_size (2);
  cloud.chan[0].name = "con";
  cloud.chan[0].set_vals_size (imgEntryArray_->width * imgEntryArray_->height);
  cloud.chan[1].name = "i";
  cloud.chan[1].set_vals_size (imgEntryArray_->width * imgEntryArray_->height);
  
  // Fill in the ROS PointCloud message
  for (int i = 0; i < imgEntryArray_->width * imgEntryArray_->height; i++)
  {
    cloud.pts[i].x = xp_[i];
    cloud.pts[i].y = yp_[i];
    cloud.pts[i].z = zp_[i];
    cloud.chan[0].vals[i] = (confidence_image[i * 2 + 0] << 0) + (confidence_image[i * 2 + 1] << 8);
    cloud.chan[1].vals[i] = (amplitude_image[i * 2 + 0] << 0) + (amplitude_image[i * 2 + 1] << 8);
  }
  
  images.set_images_size (3);

  images.images[0].width  = cols_;
  images.images[0].height = rows_;
  images.images[0].colorspace  = "mono16";
  images.images[0].compression = "raw";
  images.images[0].label       = "sr4k-distance";
  images.images[0].set_data_size (image_size);
  memcpy (&(images.images[0].data[0]), distance_image, images.images[0].get_data_size ());

  images.images[1].width  = cols_;
  images.images[1].height = rows_;
  //if (MODE & AM_CONV_GRAY)
  images.images[1].colorspace  = "mono16";
  images.images[1].compression = "raw";
  images.images[1].label       = "sr4k-intensity";
  images.images[1].set_data_size (image_size);
  memcpy (&(images.images[1].data[0]), amplitude_image, images.images[1].get_data_size ());

  images.images[2].width  = cols_;
  images.images[2].height = rows_;
  images.images[2].colorspace  = "mono16";
  images.images[2].compression = "raw";
  images.images[2].label       = "sr4k-confidence";
  images.images[2].set_data_size (image_size);
  memcpy (&(images.images[2].data[0]), confidence_image, images.images[2].get_data_size ());

  return;
}

////////////////////////////////////////////////////////////////////////////////
int
  swissranger::SwissRanger::setAutoIllumination (bool on)
{
  int res;
  if (on)
//    res = SR_SetAutoIllumination (srCam_, 5, 255, 10, 45);
    res = SR_SetAutoExposure (srCam_, 20, 20, 5, 100);
  else
    res = SR_SetAutoExposure (srCam_, 255, 0, 0, 0);
  return (res);
}

////////////////////////////////////////////////////////////////////////////////
int
  swissranger::SwissRanger::setIntegrationTime (int time)
{
  // ---[ Set integration time
  return (SR_SetIntegrationTime (srCam_, time));
}

////////////////////////////////////////////////////////////////////////////////
int
  swissranger::SwissRanger::getIntegrationTime ()
{
  // ---[ Set integration time
  return (SR_GetIntegrationTime (srCam_));
}

////////////////////////////////////////////////////////////////////////////////
int
  swissranger::SwissRanger::setModulationFrequency (int freq)
{
  // ---[ Set modulation frequency
  return (SR_SetModulationFrequency (srCam_, (ModulationFrq)freq));
}

////////////////////////////////////////////////////////////////////////////////
int
  swissranger::SwissRanger::getModulationFrequency ()
{
  // ---[ Set modulation frequency
  return (SR_GetModulationFrequency (srCam_));
}

////////////////////////////////////////////////////////////////////////////////
int
  swissranger::SwissRanger::setAmplitudeThreshold (int thresh)
{
  // ---[ Set amplitude threshold
  return (SR_SetAmplitudeThreshold (srCam_, thresh));
}

////////////////////////////////////////////////////////////////////////////////
int
  swissranger::SwissRanger::getAmplitudeThreshold ()
{
  // ---[ Set amplitude threshold
  return (SR_GetAmplitudeThreshold (srCam_));
}

////////////////////////////////////////////////////////////////////////////////
// Obtain the device product name
std::string
  swissranger::SwissRanger::getDeviceString ()
{
  char *buf = new char[256];
  int *buflen = new int;
  SR_GetDeviceString (srCam_, buf, *buflen);

  // VendorID:0x%04x, ProductID:0x%04x, Manufacturer:'%s', Product:'%s'
  std::string sensor (buf);
  std::string::size_type loc = sensor.find ("Product:", 0);
  if (loc != std::string::npos)
  {
    sensor = sensor.substr (loc + 9, *buflen);
    loc = sensor.find ("'", 0);
    if (loc != std::string::npos)
      sensor = sensor.substr (0, loc);
  }
  else
    sensor = "";

  delete buflen;
  delete [] buf;
  return (sensor);
}

////////////////////////////////////////////////////////////////////////////////
// Obtain the libusbSR library version
std::string
  swissranger::SwissRanger::getLibraryVersion ()
{
  unsigned short version[4];
  char buf[80];
  SR_GetVersion (version);
  snprintf (buf, sizeof (buf), "%d.%d.%d.%d", version[3], version[2], version[1], version[0]);
  return (std::string (buf));
}
