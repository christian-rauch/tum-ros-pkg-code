#include <ros/ros.h>
#include <ros/node_handle.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <string.h>
#include "math.h"
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <string.h>

using namespace std;

class DetectCircle2D 
{
public:
  int c_;
  int px_[0], py_[0], radius_;
  //for CvThreshold
  double threshold_, max_value_;
  //for CvSmooth
  int aperture_width_, aperture_height_;
  // for cvHoughCircles
  int accumulator_, canny_high_threshold_, accumulator_center_, min_radius_, max_radius_;
  int max_circles_;
  double sleep_;
  std::string image_topic_;
  CvMemStorage* cstorage_;
  bool hough_circle_;
  int ellipse_width_, ellipse_height_;
  DetectCircle2D(ros::NodeHandle &n) :
    n_(n), it_(n_)
  {
    px_[0] = 0; py_[0] = 0; radius_ = 0;
    n_.param("threshold", threshold_, 100.00);
    n_.param("max_value", max_value_, 255.00);
    n_.param("aperture_height_", aperture_height_, 9);
    n_.param("aperture_width_", aperture_width_, 9);
    n_.param("accumulator", accumulator_, 2);
    n_.param("canny_high_threshold", canny_high_threshold_, 5);
    n_.param("accumulator_center", accumulator_center_, 35);
    n_.param("min_radius", min_radius_, 80);
    n_.param("max_radius", max_radius_, 150);
    n_.param("max_circles", max_circles_, 2);
    n_.param("sleep", sleep_, 0.1);
    n_.param("image_topic", image_topic_, std::string("/narrow_stereo/right/image_rect"));
    n_.param("hough_circle", hough_circle_, false);
    n_.param("ellipse_width", ellipse_width_, 80);
    n_.param("ellipse_height", ellipse_height_, 80);
    image_sub_ = it_.subscribe(
                               image_topic_, 1, &DetectCircle2D::imageCallback, this);
    ROS_INFO("DetectCircle2D Ready!!!");
    cstorage_ = cvCreateMemStorage(0);
    if (hough_circle_)
    {
      cvNamedWindow("src",1);
      cvNamedWindow("gray",1);
    }
    else 
    {
      cvNamedWindow("Source", 1);
      cvNamedWindow("Result", 1);
    }
  }

  ~DetectCircle2D()
  {
    //release all windows
    cvDestroyAllWindows();
  }

  
  // Define trackbar callback functon. This function find contours,
// draw it and approximate it by ellipses.
  void process_image(int h, IplImage *image02, IplImage *image03, IplImage *image04)
    {
      CvMemStorage* storage;
      CvSeq* contour;
      
      // Create dynamic structure and sequence.
      storage = cvCreateMemStorage(0);
      contour = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint) , storage);
      
      // Threshold the source image. This needful for cvFindContours().
      cvThreshold( image03, image02, threshold_, 255, CV_THRESH_BINARY );
      
      // Find all contours.
      cvFindContours( image02, storage, &contour, sizeof(CvContour),
                      CV_RETR_LIST, CV_CHAIN_APPROX_NONE, cvPoint(0,0));
      
      // Clear images. IPL use.
      //cvZero(image02);
      cvZero(image04);
      // This cycle draws all contours and approximates them by ellipses.
      int count_contours = 0;
      for(;contour;contour = contour->h_next)
      {
        count_contours++;
        int count = contour->total; // This is number of points in contour
        CvPoint center;
        CvSize size;
        CvBox2D box;
        
        // Number points must be greater or equal 6 (for cvFitEllipse_32f).
        if( count < 6 )
          continue;
        
        CvMat* points_f = cvCreateMat( 1, count, CV_32FC2 );
        CvMat points_i = cvMat( 1, count, CV_32SC2, points_f->data.ptr );
        cvCvtSeqToArray( contour, points_f->data.ptr, CV_WHOLE_SEQ );
        cvConvert( &points_i, points_f );
        
        // Fits ellipse to current contour.
        box = cvFitEllipse2( points_f );
        
        // Draw current contour.
        cvDrawContours(image04,contour,CV_RGB(255,255,255),CV_RGB(255,255,255),0,1,8,cvPoint(0,0));
        
        // Convert ellipse data from float to integer representation.
        center = cvPointFrom32f(box.center);
        size.width = cvRound(box.size.width*0.5);
        size.height = cvRound(box.size.height*0.5);
        // Draw ellipse.
	if (size.width > ellipse_width_ && size.height > ellipse_height_ && size.width < 2.0 * ellipse_width_ && size.height < 2.0 * ellipse_height_)
	//	if (size.width > ellipse_width_ && size.height > ellipse_height_)
        {
          ROS_INFO_STREAM("size: " << size.width << " " << size.height);
          cvEllipse(image03, center, size,
                    -box.angle, 0, 360,
                    CV_RGB(0,0,0), 1, CV_AA, 0);
        }
        cvReleaseMat(&points_f);
      }
      ROS_INFO_STREAM("Number of contours " << count_contours);
      
      // Show image. HighGUI use.
      cvShowImage( "Result", image03 );
      //cvShowImage( "Result", image04 );
    }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
    {
      ROS_INFO("New image received");
      IplImage *cv_image = NULL;
      try
      {
        cv_image = bridge_.imgMsgToCv(msg_ptr, "passthrough");
      }
      catch (sensor_msgs::CvBridgeException error)
      {
        ROS_ERROR("error");
      }

      if (hough_circle_)
      {
        IplImage *gray_ = NULL;
        gray_=cvCloneImage(cv_image);
        //color threshold
        cvThreshold(gray_, gray_, threshold_, max_value_, CV_THRESH_BINARY);
        //smooth the image to reduce unneccesary results
        cvSmooth( gray_, gray_, CV_GAUSSIAN, aperture_width_, aperture_height_);
        // cerr <<  accumulator_ << " " << gray_->height/5 << " " << canny_high_threshold_ << " " << accumulator_center_ << " " << min_radius_ << " " << max_radius_ << endl;
        //get circles
        CvSeq* circles =  cvHoughCircles( gray_, cstorage_, CV_HOUGH_GRADIENT, accumulator_, gray_->height/5, 
                                          canny_high_threshold_, accumulator_center_, min_radius_, max_radius_);
        //output all the circle detected
        ROS_INFO_STREAM("circles total: " << circles->total);
        //start drawing all the circles
        int i;
        for( i = 0; circles->total>=max_circles_?i<max_circles_:i < circles->total; i++ )
        { 
          //just make a filter to limit only <=2 circles to draw
          float* p = (float*)cvGetSeqElem( circles, i );
          cvCircle( cv_image, cvPoint(cvRound(p[0]),cvRound(p[1])), 3, CV_RGB(255,0,0), -1, 8, 0 );
          cvCircle( cv_image, cvPoint(cvRound(p[0]),cvRound(p[1])), cvRound(p[2]), CV_RGB(200,0,0), 1, 8, 0 );
          px_[i]=cvRound(p[0]); py_[i]=cvRound(p[1]); radius_ = cvRound(p[2]);
        }
        
        //output two circles' center position
        ROS_INFO_STREAM("center x: "<<px_[0]<<"  center y: "<<py_[0] << " radius: " << radius_);
        cvSetImageROI(gray_, cvRect(0,0,cv_image->width,cv_image->height));
        cvShowImage("gray",gray_);
        cvShowImage("src",cv_image);
        //release memory
        cvClearSeq(circles);
        cvReleaseImage(&gray_);
        cvClearMemStorage( cstorage_ );
        //ready to exit loop
        
        c_=cvWaitKey(10);
        if(c_==27)
          exit(2);
        sleep(sleep_);
      }
      else //fit ellipse
      {
//        const char* filename = argc == 2 ? argv[1] : (char*)"stuff.jpg";
        
        // load image and force it to be grayscale

        
        // Create the destination images
        IplImage * image02 = cvCloneImage( cv_image );
        IplImage * image04 = cvCloneImage( cv_image );
        
        // Create windows.
//        cvNamedWindow("Source", 1);
        //      cvNamedWindow("Result", 1);
        
        // Show the image.
        cvShowImage("Source", cv_image);
        
        // Create toolbars. HighGUI use.
//        cvCreateTrackbar( "Threshold", "Result", &slider_pos, 255, process_image );
        
        process_image(0, image02, cv_image, image04);
        
        // Wait for a key stroke; the same function arranges events processing
        //cvWaitKey(0);
        c_=cvWaitKey(10);
        if(c_==27)
          exit(2);
        sleep(sleep_);

        cvReleaseImage(&image02);
        cvReleaseImage(&image04);
        
//        cvDestroyWindow("Source");
        //cvDestroyWindow("Result");
      }
    }
  
protected:
  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  sensor_msgs::CvBridge bridge_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detect_circle2D");
  ros::NodeHandle nh("~");
  DetectCircle2D dc(nh);
  ros::spin();
  return 0;
}
