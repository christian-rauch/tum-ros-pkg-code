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

#ifndef SWISSRANGER_HH
#define SWISSRANGER_HH

#include <stdexcept>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string.h>

#include <opencv/cv.h>

// For some reason 1.0.10-541 does not provide this
#define DWORD unsigned int

#include <libusbSR.h>

// ROS include
#include "std_msgs/PointCloud.h"
#include "std_msgs/ImageArray.h"
#include "ros/common.h"

// Older library: #define MODE (AM_COR_FIX_PTRN | AM_COR_LED_NON_LIN | AM_MEDIAN)
#define MODE (AM_CONF_MAP | AM_COR_FIX_PTRN | AM_SW_ANF | AM_MEDIAN | AM_DENOISE_ANF | AM_MEDIANCROSS | AM_CONV_GRAY)// | AM_SHORT_RANGE)
using namespace std;

namespace swissranger
{
  //! Macro for defining an exception with a given parent (std::runtime_error should be top parent)
  // code borrowed from drivers/laser/hokuyo_driver/hokuyo.h
#define DEF_EXCEPTION(name, parent) \
  class name  : public parent { \
  public: \
    name (const char* msg) : parent (msg) {} \
  }

  //! A standard SwissRanger exception
  DEF_EXCEPTION(Exception, std::runtime_error);

  const unsigned int SR_COLS = 176;
  const unsigned int SR_ROWS = 144;

  class SwissRanger
  {
    public:
      SwissRanger ();
      ~SwissRanger ();

      int open ();
      int close ();

      void readData (std_msgs::PointCloud &cloud, std_msgs::ImageArray &images);

      int setAutoIllumination (bool on);
      int setIntegrationTime (int time);
      int getIntegrationTime ();
      int setModulationFrequency (int freq);
      int getModulationFrequency ();
      int setAmplitudeThreshold (int thresh);
      int getAmplitudeThreshold ();

      // SwissRanger specific values
      unsigned int rows_, cols_, inr_;
      string device_id_;
      string lib_version_;

    private:
      // device identifier
      CMesaDevice* srCam_;

      ImgEntry* imgEntryArray_;
      float *buffer_, *xp_, *yp_, *zp_;

      int integration_time_, modulation_freq_;

      string getDeviceString ();
      string getLibraryVersion ();

      // Used for rotating images with 180 deg around their centre internally
      void rotateImage180 (uint8_t *img, uint8_t *rot_img, int width, int height);
      
      // Stores the intrinsic camera matrix and the distortion coefficients after calibration
      CvMat *intrinsic_, *distortion_;
      void undistort (uint8_t *img, uint8_t *un_img, int width, int height);
      void contours (uint8_t *img, uint8_t *con_img, int width, int height, int threshold);
      double getAngle (float px, float py, float pz, float qx, float qy, float qz);
  };
};

#endif
