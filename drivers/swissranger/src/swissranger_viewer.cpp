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

#include <cstdio>
#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <ros/node.h>
#include <deprecated_msgs/ImageArray.h>

using namespace std;
using namespace deprecated_msgs;

class SwissRangerViewer
{
  protected:
    ros::Node& node_;
  public:
    ImageArray images_sr;
    char key;
    int imgnum;

    IplImage *sr_img1, *sr_img2, *sr_img3;
  
    SwissRangerViewer (ros::Node& anode) : node_ (anode)
    {
      cvNamedWindow ("sr4k - distance", CV_WINDOW_AUTOSIZE);
      cvNamedWindow ("sr4k - amplitude", CV_WINDOW_AUTOSIZE);
      cvNamedWindow ("sr4k - confidence", CV_WINDOW_AUTOSIZE);
      node_.subscribe ("images_sr", images_sr, &SwissRangerViewer::image_sr_cb, this, 1);
      sr_img1 = cvCreateImage (cvSize (1, 1), IPL_DEPTH_16U, 1);
      sr_img2 = cvCreateImage (cvSize (1, 1), IPL_DEPTH_16U, 1);
      sr_img3 = cvCreateImage (cvSize (1, 1), IPL_DEPTH_16U, 1);
      imgnum = 0;
    }

    void
      image_sr_cb ()
    {
      cvReleaseImage (&sr_img1);
      cvReleaseImage (&sr_img2);
      cvReleaseImage (&sr_img3);

      fprintf (stderr, "Received %d images of size:", images_sr.get_images_size ());
      for (unsigned int j = 0; j < images_sr.get_images_size (); j++)
      {
        fprintf (stderr, " %dx%d and type %s,", images_sr.images[j].width, images_sr.images[j].height, images_sr.images[j].colorspace.c_str ());
        if (images_sr.images[j].width == 0 || images_sr.images[j].height == 0)
         return;
      }
      fprintf (stderr, "\n");

      sr_img1 = cvCreateImage (cvSize (images_sr.images[0].width, images_sr.images[0].height), IPL_DEPTH_16U, 1);
      sr_img2 = cvCreateImage (cvSize (images_sr.images[1].width, images_sr.images[1].height), IPL_DEPTH_16U, 1);
      /// 3rd image
      sr_img3 = cvCreateImage (cvSize (images_sr.images[2].width, images_sr.images[2].height), IPL_DEPTH_16U, 1);

      memcpy (sr_img1->imageData, &(images_sr.images[0].data[0]), sr_img1->imageSize);
      memcpy (sr_img2->imageData, &(images_sr.images[1].data[0]), sr_img2->imageSize);
      memcpy (sr_img3->imageData, &(images_sr.images[2].data[0]), sr_img3->imageSize);

      cvShowImage ("sr4k - distance", sr_img1);

      IplImage* sr_img2_rot = cvCloneImage (sr_img2);
      sr_img2 = cvCreateImage (cvGetSize (sr_img2_rot), IPL_DEPTH_8U, 3);
      if (sr_img2_rot->depth == IPL_DEPTH_16U)
        cvConvertImage (sr_img2_rot, sr_img2, CV_CVTIMG_SWAP_RB);

      IplImage *sr_img2_a = cvCreateImage (cvGetSize (sr_img2_rot), IPL_DEPTH_8U, 1);
      cvCvtColor (sr_img2, sr_img2_a, CV_RGB2GRAY);
        
      IplImage *sr_img2_rot_hist = cvCloneImage (sr_img2_a);
      cvEqualizeHist (sr_img2_a, sr_img2_rot_hist);

      cvReleaseImage (&sr_img2);
        
      sr_img2 = cvCreateImage (cvSize (sr_img2_rot_hist->width << 1, sr_img2_rot_hist->height << 1), IPL_DEPTH_8U, 1);
      cvResize (sr_img2_rot_hist, sr_img2, CV_INTER_NN);
      cvShowImage ("sr4k - amplitude", sr_img2);
              
      cvReleaseImage (&sr_img2_rot_hist);
      cvReleaseImage (&sr_img2_rot);

      cvShowImage ("sr4k - confidence", sr_img3);
      cvWaitKey (20);
    }

};

int
  main (int argc, char **argv)
{
  ros::init (argc, argv);
  ros::Node ros_node ("swissranger_viewer");

  SwissRangerViewer c (ros_node);

  ros_node.spin ();

  return (0);
}
