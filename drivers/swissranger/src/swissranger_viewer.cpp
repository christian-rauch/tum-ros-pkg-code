#include <cstdio>
#include <vector>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "ros/node.h"
#include "std_msgs/Image.h"
#include "std_msgs/ImageArray.h"

using namespace std;
using namespace ros;

class CvView:public node
{
public:
  std_msgs::Image      image_flir_msg;
  std_msgs::ImageArray images_stoc_msg, images_sr_msg;
  char key;
  int imgnum;

  bool rotate;
  
  IplImage *sr_img1, *sr_img2, *sr_img3;
  IplImage *stoc_img1, *stoc_img2, *stoc_img3;
  IplImage *flir_img;
  
  bool stereo, sr4k, flir;

  CvView ():node ("cam_viewer"), rotate (false), stereo (false), sr4k (true), flir (false)
  {
    if (stereo)
    {
      cvNamedWindow ("stoc - left", CV_WINDOW_AUTOSIZE);
      cvNamedWindow ("stoc - right", CV_WINDOW_AUTOSIZE);
      cvNamedWindow ("stoc - disparity", CV_WINDOW_AUTOSIZE);
      subscribe ("images_stoc", images_stoc_msg, &CvView::image_stoc_cb);
      stoc_img1 = cvCreateImage (cvSize (1, 1), IPL_DEPTH_16U, 1);
      stoc_img2 = cvCreateImage (cvSize (1, 1), IPL_DEPTH_16U, 1);
      stoc_img3 = cvCreateImage (cvSize (1, 1), IPL_DEPTH_16U, 1);
    }
    
    if (sr4k)
    {
      cvNamedWindow ("sr4k - distance", CV_WINDOW_AUTOSIZE);
      cvNamedWindow ("sr4k - amplitude", CV_WINDOW_AUTOSIZE);
      cvNamedWindow ("sr4k - confidence", CV_WINDOW_AUTOSIZE);
      subscribe ("images_sr", images_sr_msg, &CvView::image_sr_cb);
      sr_img1 = cvCreateImage (cvSize (1, 1), IPL_DEPTH_16U, 1);
      sr_img2 = cvCreateImage (cvSize (1, 1), IPL_DEPTH_16U, 1);
      sr_img3 = cvCreateImage (cvSize (1, 1), IPL_DEPTH_16U, 1);
    }
    
    if (flir)
    {
      cvNamedWindow ("flir", CV_WINDOW_AUTOSIZE);
      subscribe("image_flir", image_flir_msg, &CvView::image_flir_cb);
      flir_img = cvCreateImage (cvSize (1, 1), IPL_DEPTH_16U, 1);
    }

    imgnum = 0;
  }

  void printMatrix (CvMat *mat)
  {
    for (int i = 0; i < mat->rows; i++)
    {
      for (int j = 0; j < mat->cols; j++)
        fprintf (stderr, "%12.6f", cvmGet (mat, i, j));

      fprintf (stderr, "\n" );
    }
  }

  void
    image_flir_cb ()
  {
    cvReleaseImage (&flir_img);
    fprintf (stderr, "Received image of size: %dx%d and type %s\n", image_flir_msg.width, image_flir_msg.height, image_flir_msg.colorspace.c_str ());
    
    if (image_flir_msg.width == 0 || image_flir_msg.height == 0)
     return;
    
    flir_img = cvCreateImage (cvSize (image_flir_msg.width, image_flir_msg.height), IPL_DEPTH_16U, 1);
    memcpy (flir_img->imageData, &(image_flir_msg.data[0]), flir_img->imageSize);
    
    CvSize imageSize2x = cvSize (flir_img->width << 1, flir_img->height << 1);
    IplImage *img2 = cvCreateImage (imageSize2x, IPL_DEPTH_16U, 1);
    cvResize (flir_img, img2, CV_INTER_NN);
    
    IplImage *img1_rot = cvCloneImage (img2);
    CvPoint2D32f center = cvPoint2D32f (img1_rot->width / 2.0, img1_rot->height / 2.0);
    CvMat *rot_mat = cvCreateMat (2, 3, CV_32F);
    cv2DRotationMatrix (center, 180, 1.0, rot_mat);
    printMatrix (rot_mat);
    cvWarpAffine (img2, img1_rot, rot_mat, CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS);
      
    cvShowImage ("flir", img1_rot);
    
    cvReleaseImage (&img2);
    cvReleaseImage (&img1_rot);
    cvWaitKey (10);
  }
  
  void
    image_sr_cb ()
  {
    cvReleaseImage (&sr_img1);
    cvReleaseImage (&sr_img2);
    cvReleaseImage (&sr_img3);

    fprintf (stderr, "Received %d images of size:", images_sr_msg.get_images_size ());
    for (unsigned int j = 0; j < images_sr_msg.get_images_size (); j++)
    {
      fprintf (stderr, " %dx%d and type %s,", images_sr_msg.images[j].width, images_sr_msg.images[j].height, images_sr_msg.images[j].colorspace.c_str ());
      if (images_sr_msg.images[j].width == 0 || images_sr_msg.images[j].height == 0)
       return;
    }
    fprintf (stderr, "\n");

    sr_img1 = cvCreateImage (cvSize (images_sr_msg.images[0].width, images_sr_msg.images[0].height), IPL_DEPTH_16U, 1);
    sr_img2 = cvCreateImage (cvSize (images_sr_msg.images[1].width, images_sr_msg.images[1].height), IPL_DEPTH_16U, 1);
    /// 3rd image
    sr_img3 = cvCreateImage (cvSize (images_sr_msg.images[2].width, images_sr_msg.images[2].height), IPL_DEPTH_16U, 1);

    memcpy (sr_img1->imageData, &(images_sr_msg.images[0].data[0]), sr_img1->imageSize);
    memcpy (sr_img2->imageData, &(images_sr_msg.images[1].data[0]), sr_img2->imageSize);
    memcpy (sr_img3->imageData, &(images_sr_msg.images[2].data[0]), sr_img3->imageSize);

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

  void
    image_stoc_cb ()
  {
    cvReleaseImage (&stoc_img1);
    cvReleaseImage (&stoc_img2);
//    cvReleaseImage (&stoc_img3);
    
    fprintf (stderr, "Received %d images of size:", images_stoc_msg.get_images_size ());
    for (unsigned int j = 0; j < images_stoc_msg.get_images_size (); j++)
    {
      fprintf (stderr, " %dx%d and type %s,", images_stoc_msg.images[j].width, images_stoc_msg.images[j].height, images_stoc_msg.images[j].colorspace.c_str ());
      if (images_stoc_msg.images[j].width == 0 || images_stoc_msg.images[j].height == 0)
       return;
    }
    fprintf (stderr, "\n");

    if (images_stoc_msg.images[0].colorspace == "mono16")
      stoc_img1 = cvCreateImage (cvSize (images_stoc_msg.images[0].width, images_stoc_msg.images[0].height), IPL_DEPTH_16U, 1);
    else if (images_stoc_msg.images[0].colorspace == "rgb24")
      stoc_img1 = cvCreateImage (cvSize (images_stoc_msg.images[0].width, images_stoc_msg.images[0].height), IPL_DEPTH_8U, 3);
    else if (images_stoc_msg.images[0].colorspace == "mono8")
      stoc_img1 = cvCreateImage (cvSize (images_stoc_msg.images[0].width, images_stoc_msg.images[0].height), IPL_DEPTH_8U, 1);

    if (images_stoc_msg.images[1].colorspace == "mono16")
      stoc_img2 = cvCreateImage (cvSize (images_stoc_msg.images[1].width, images_stoc_msg.images[1].height), IPL_DEPTH_16U, 1);
    else if (images_stoc_msg.images[1].colorspace == "rgb24")
      stoc_img2 = cvCreateImage (cvSize (images_stoc_msg.images[1].width, images_stoc_msg.images[1].height), IPL_DEPTH_8U, 3);
    else if (images_stoc_msg.images[1].colorspace == "mono8")
      stoc_img2 = cvCreateImage (cvSize (images_stoc_msg.images[1].width, images_stoc_msg.images[1].height), IPL_DEPTH_8U, 1);

    /// 3rd image - usually disparity
/*    if (images_stoc_msg.images[2].colorspace == "mono16")
      stoc_img3 = cvCreateImage (cvSize (images_stoc_msg.images[2].width, images_stoc_msg.images[2].height), IPL_DEPTH_16U, 1);
    else if (images_stoc_msg.images[2].colorspace == "rgb24")
      stoc_img3 = cvCreateImage (cvSize (images_stoc_msg.images[2].width, images_stoc_msg.images[2].height), IPL_DEPTH_8U, 3);
    else if (images_stoc_msg.images[2].colorspace == "mono8")
      stoc_img3 = cvCreateImage (cvSize (images_stoc_msg.images[2].width, images_stoc_msg.images[2].height), IPL_DEPTH_8U, 1);*/

    memcpy (stoc_img1->imageData, &(images_stoc_msg.images[0].data[0]), stoc_img1->imageSize);
    memcpy (stoc_img2->imageData, &(images_stoc_msg.images[1].data[0]), stoc_img2->imageSize);
///    memcpy (stoc_img3->imageData, &(images_stoc_msg.images[2].data[0]), stoc_img3->imageSize);

    cvShowImage ("stoc - left", stoc_img1);
    cvShowImage ("stoc - right", stoc_img2);
//    cvShowImage ("stoc - disparity", stoc_img3);
    cvWaitKey (5);
  }
};

int
  main (int argc, char **argv)
{
  ros::init (argc, argv);
  CvView view;
  view.spin ();
  ros::fini ();
  return 0;
}
