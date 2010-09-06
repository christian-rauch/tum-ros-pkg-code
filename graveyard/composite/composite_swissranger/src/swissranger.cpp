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
  // Set the intrinsic camera parameters (found after calibration)
  intrinsic_  = cvCreateMat (3, 3, CV_64FC1);
  distortion_ = cvCreateMat (1, 4, CV_64FC1);
  cvmSet (intrinsic_, 0, 0, 250.8230); cvmSet (intrinsic_, 0, 1, 0.0);      cvmSet (intrinsic_, 0, 2, 83.3768);
  cvmSet (intrinsic_, 1, 0, 0.0);      cvmSet (intrinsic_, 1, 1, 252.9221); cvmSet (intrinsic_, 1, 2, 72.6264);
  cvmSet (intrinsic_, 2, 0, 0.0);      cvmSet (intrinsic_, 2, 1, 0.0);      cvmSet (intrinsic_, 2, 2, 1.0);
  cvmSet (distortion_, 0, 0, -0.0612); cvmSet (distortion_, 0, 1, 0.1706);  cvmSet (distortion_, 0, 2, 0); cvmSet (distortion_, 0, 3, 0);
  
  pcd_filter_ = false;
  undistort_distance_ = undistort_amplitude_ = undistort_confidence_ = false;
}

swissranger::SwissRanger::~SwissRanger ()
{
  cvReleaseMat (&intrinsic_);
  cvReleaseMat (&distortion_);
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
// Rotate an image buffer with 180 degrees
void
  swissranger::SwissRanger::rotateImage180 (uint8_t *img, uint8_t *rot_img, int width, int height)
{
  for (int u = 0; u < height; u++)
  {
    int nu_ = (height - u - 1) * width;
    int u_  =           u      * width;
    for (int v = 0; v < width; v++)
    {
      int nv = width - v - 1;
      rot_img[(nu_ + nv) * 2 + 0] = img[(u_ + v) * 2 + 0];
      rot_img[(nu_ + nv) * 2 + 1] = img[(u_ + v) * 2 + 1];
    }
  }
}


////////////////////////////////////////////////////////////////////////////////
// Undistort an image based on the intrinsic camera parameters
void
  swissranger::SwissRanger::undistort (uint8_t *img, uint8_t *un_img, int width, int height)
{
  CvMat *src = cvCreateMatHeader (height, width, CV_8UC2);
  cvSetData (src, img, width * 2);
  CvMat *dst = cvCreateMatHeader (height, width, CV_8UC2);
  cvSetData (dst, un_img, width * 2);
  
  cvUndistort2 (src, dst, intrinsic_, distortion_);
}


////////////////////////////////////////////////////////////////////////////////
#define _sqr(x) ((x)*(x))
#define _sqr_sum_3d(x) (_sqr(x[0])+_sqr(x[1])+_sqr(x[2]))
#define _dot(x,y) ((x[0])*(y[0])+(x[1])*(y[1])+(x[2])*(y[2]))
double
  swissranger::SwissRanger::getAngle (float px, float py, float pz, float qx, float qy, float qz)
{
  float dir_a[3], dir_b[3];
  dir_a[0] = cvmGet (intrinsic_, 0, 2) - px; dir_a[1] = cvmGet (intrinsic_, 1, 2) - py; dir_a[2] = -pz;
  dir_b[0] = cvmGet (intrinsic_, 0, 2) - qx; dir_b[1] = cvmGet (intrinsic_, 1, 2) - qy; dir_b[2] = -qz;
  
  double norm_a = sqrt (_sqr_sum_3d (dir_a));
  double norm_b = sqrt (_sqr_sum_3d (dir_b));
  return (acos (_dot (dir_a, dir_b) / (norm_a * norm_b)));
}

////////////////////////////////////////////////////////////////////////////////
// Get contours
void
  swissranger::SwissRanger::contours (uint8_t *img, uint8_t *con_img,
                                      int width, int height, int threshold)
{
  CvMat *src = cvCreateMatHeader (height, width, CV_8UC1);
  cvSetData (src, img + width * height, width);

  CvMat *dst = cvCreateMatHeader (height, width, CV_8UC1);
  cvSetData (dst, con_img + width * height, width);
  
  cvCanny (src, dst, 5, 2);

  // Convert source to GRAY
//  IplImage *monoImg = cvCreateImage (cvGetSize (srcImg), IPL_DEPTH_8U, 1);
//  cvCvtColor (srcImg, monoImg, CV_RGB2GRAY);

  // Clone source
//  IplImage *tmpImg = cvCloneImage (monoImg);

  // Threshold the source image. This needful for cvFindContours ()
/*  cvThreshold (monoImg, tmpImg, threshold, 255, CV_THRESH_BINARY);

//  cvReleaseImage (&monoImg);

  // Find all contours
  CvMemStorage* stor = cvCreateMemStorage (0);
  CvSeq* cont = cvCreateSeq (CV_SEQ_ELTYPE_POINT, sizeof (CvSeq), sizeof (CvPoint), stor);
  cvFindContours (tmpImg, stor, &cont, sizeof (CvContour),
                  CV_RETR_LIST, CV_CHAIN_APPROX_NONE, cvPoint (0, 0));
//  cvZero (tmpImg);

  // This cycle draw all contours and approximate it by ellipses
  for (;cont; cont = cont->h_next)
  {
    cvDrawContours (dstImg, cont, CV_RGB (255, 255, 255), CV_RGB (255, 255,255), 0, 1, 8, cvPoint (0, 0));
  }

//  cvShowImage ("test", tmpImg);
//  cvWaitKey (0);
  cvReleaseImage (&tmpImg);*/
}

////////////////////////////////////////////////////////////////////////////////
// Read data from the device
void
  swissranger::SwissRanger::readData (std_msgs::PointCloud &cloud, std_msgs::ImageArray &images)
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
  
  // Stores a copy of the rotated distance image
  uint8_t *img_tmp = (uint8_t*)malloc (image_size);
  memcpy (img_tmp, distance_image, image_size);

  if (undistort_distance_)
  {
    // Rotate the distance image with 180 on spot
    rotateImage180 (distance_image, img_tmp, imgEntryArray_->width, imgEntryArray_->height);
    // Undistort the distance image
    undistort (img_tmp, distance_image, imgEntryArray_->width, imgEntryArray_->height);
  }
  else
    // Rotate the distance image with 180 on spot
    rotateImage180 (img_tmp, distance_image, imgEntryArray_->width, imgEntryArray_->height);

  memcpy (img_tmp, amplitude_image, image_size);
  
  if (undistort_amplitude_)
  {
    // Rotate the amplitude image with 180 on spot
    rotateImage180 (amplitude_image, img_tmp, imgEntryArray_->width, imgEntryArray_->height);
    // Undistort the amplitude image
    undistort (img_tmp, amplitude_image, imgEntryArray_->width, imgEntryArray_->height);
  }
  else
    // Rotate the amplitude image with 180 on spot
    rotateImage180 (img_tmp, amplitude_image, imgEntryArray_->width, imgEntryArray_->height);

  memcpy (img_tmp, confidence_image, image_size);
  
  if (undistort_confidence_)
  {
    // Rotate the confidence image with 180 on spot
    rotateImage180 (confidence_image, img_tmp, imgEntryArray_->width, imgEntryArray_->height);
    //contours (img_tmp, confidence_image, imgEntryArray_->width, imgEntryArray_->height, 1);
    // Undistort the confidence image
    undistort (img_tmp, confidence_image, imgEntryArray_->width, imgEntryArray_->height);
  }
  else
    // Rotate the confidence image with 180 on spot
    rotateImage180 (img_tmp, confidence_image, imgEntryArray_->width, imgEntryArray_->height);
  
  // Points array
  res = SR_CoordTrfFlt (srCam_, xp_, yp_, zp_, sizeof (float), sizeof (float), sizeof (float));

  // Filter points
  cloud.set_pts_size (imgEntryArray_->width * imgEntryArray_->height);
  cloud.set_chan_size (2);
  if (pcd_filter_)                    // if in-driver filtering is enabled, do not send the point confidence anymore, but rather the original point index
    cloud.chan[0].name = "pid";
  else
    cloud.chan[0].name = "con";
  cloud.chan[0].set_vals_size (imgEntryArray_->width * imgEntryArray_->height);
  cloud.chan[1].name = "i";
  cloud.chan[1].set_vals_size (imgEntryArray_->width * imgEntryArray_->height);
  
  int nr_pts = 0;
  for (int u = 0; u < imgEntryArray_->height; u++)
  {
    int nu = imgEntryArray_->width * u;
    for (int v = 0; v < imgEntryArray_->width; v++)
    {
      int i = nu + v;
      
      if (pcd_filter_)                // is in-driver filtering enabled ?
      {
        unsigned short conf_val = (confidence_image[i * 2 + 0] << 0) + (confidence_image[i * 2 + 1] << 8);
        unsigned short dist_val = (distance_image[i * 2 + 0] << 0) + (distance_image[i * 2 + 1] << 8);
        if ( (conf_val > 0xffff * 3 / 4) && (dist_val < 0xff00) )
        {
          if (getAngle (xp_[i], yp_[i], zp_[i], xp_[i+1], yp_[i+1], zp_[i+1]) > 0.0004)
  	    continue;
        }
      }

      // Save the XYZ coordinates
      cloud.pts[nr_pts].x = xp_[i];
      cloud.pts[nr_pts].y = yp_[i];
      cloud.pts[nr_pts].z = zp_[i];

      if (pcd_filter_)                // is in-driver filtering enabled ?
        cloud.chan[0].vals[nr_pts] = i;
      else
        cloud.chan[0].vals[nr_pts] = (confidence_image[i * 2 + 0] << 0) + (confidence_image[i * 2 + 1] << 8);
      
      cloud.chan[1].vals[nr_pts] = (amplitude_image[i * 2 + 0] << 0) + (amplitude_image[i * 2 + 1] << 8); //amplitude_image[i*2 + 1];
      nr_pts++;
    }
  }
  cloud.pts.resize (nr_pts);
  cloud.chan[0].vals.resize (nr_pts);
  cloud.chan[1].vals.resize (nr_pts);
  
  // Fill in the ROS PointCloud message
  /*for (int i = 0; i < imgEntryArray_->width * imgEntryArray_->height; i++)
  {
    cloud.pts[i].x = xp_[i];
    cloud.pts[i].y = yp_[i];
    cloud.pts[i].z = zp_[i];
    cloud.chan[0].vals[i] = amplitude_image[i*2 + 1];
  }*/
  
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

  free (img_tmp);
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
