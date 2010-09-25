#include <ros/ros.h>
#include <ros/node_handle.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "cvaux.h"
#include "cxmisc.h"
#include "highgui.h"
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

class BGFGCodeBook {

public:
  BGFGCodeBook(ros::NodeHandle &n) :
    nh_(n), it_(nh_)
  {
    
    n.param("input_image_topic", input_image_topic_, std::string("/narrow_stereo/left/image_color_rect"));
    n.param("modMin", modMin_, 35);
    n.param("modMax", modMax_, 35);
    n.param("cbBounds_", cbBounds_, 10);
    n.param("nframesToLearnBG", nframesToLearnBG_, 10);
    n.param("nChannels", nChannels_, 3);
    image_sub_ = it_.subscribe(
                               input_image_topic_, 1, &BGFGCodeBook::imageCallback, this);

    //VARIABLES for CODEBOOK METHOD:
    model_ = 0;
    ch_={true,true,true}; // This sets what channels should be adjusted for background bounds
    rawImage_ = 0;
    yuvImage_ = 0; //yuvImage is for codebook method
    ImaskCodeBook_ = 0;
    ImaskCodeBookCC_ = 0;
    c_ =  n_ = nframes_ = 0;
    model_ = cvCreateBGCodeBookModel();
    
    //Set color thresholds to default values
    model_->modMin[0] = modMin_;
    model_->modMin[1] = model_->modMin[2] = modMin_;
    model_->modMax[0] = modMax_;
    model_->modMax[1] = model_->modMax[2] = modMax_;
    model_->cbBounds[0] = model_->cbBounds[1] = model_->cbBounds[2] = cbBounds_;

    pause_ = false;
    singlestep_ = false;
  }

  ~BGFGCodeBook()
  {
    cvDestroyWindow( "Raw" );
    cvDestroyWindow( "ForegroundCodeBook");
    cvDestroyWindow( "CodeBook_ConnectComp");
  }

  void help(void)
  {
    printf("\nLearn background and find foreground using simple average and average difference learning method:\n"
	   "\nUSAGE:\nbgfg_codebook_node _input_image_topic:=/image_topic\n"
	   "***Keep the focus on the video windows, NOT the console***\n\n"
	   "INTERACTIVE PARAMETERS:\n"
	   "\tESC,q,Q  - quit the program\n"
	   "\th    - print this help\n"
	   "\tp    - pause toggle\n"
	   "\ts    - single step\n"
	   "\tr    - run mode (single step off)\n"
	   "=== AVG PARAMS ===\n"
	   "\t-    - bump high threshold UP by 0.25\n"
	   "\t=    - bump high threshold DOWN by 0.25\n"
	   "\t[    - bump low threshold UP by 0.25\n"
	   "\t]    - bump low threshold DOWN by 0.25\n"
	   "=== CODEBOOK PARAMS ===\n"
	   "\ty,u,v- only adjust channel 0(y) or 1(u) or 2(v) respectively\n"
	   "\ta    - adjust all 3 channels at once\n"
	   "\tb    - adjust both 2 and 3 at once\n"
	   "\ti,o  - bump upper threshold up,down by 1\n"
	   "\tk,l  - bump lower threshold up,down by 1\n"
	   "\tSPACE - reset the model\n"
	   );
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
  {

    IplImage *cv_image = NULL;
    try
      {
        cv_image = bridge_.imgMsgToCv(msg_ptr, "bgr8");
      }
    catch (sensor_msgs::CvBridgeException error)
      {
        ROS_ERROR("error");
      }

    if( !pause_ )
      {
	//rawImage = cvQueryFrame( capture );
	rawImage_ = cv_image;
	++nframes_;
	if(!rawImage_) 
	  {
	    ROS_WARN("[BGFGCodeBook:] No image received");
	    return;
	  }
      }
    if( singlestep_ )
      pause_ = true;
        
    //First time:
    if( nframes_ == 1 && rawImage_ )
      {
	ROS_DEBUG("[BGFGCodeBook:] First image received");
	// CODEBOOK METHOD ALLOCATION
	yuvImage_ = cvCloneImage(rawImage_);
	ImaskCodeBook_ = cvCreateImage( cvGetSize(rawImage_), IPL_DEPTH_8U, 1 );
	ImaskCodeBookCC_ = cvCreateImage( cvGetSize(rawImage_), IPL_DEPTH_8U, 1 );
	cvSet(ImaskCodeBook_,cvScalar(255));
            
	cvNamedWindow( "Raw", 1 );
	cvNamedWindow( "ForegroundCodeBook",1);
	cvNamedWindow( "CodeBook_ConnectComp",1);
      }

    // If we've got an rawImage and are good to go:                
    if( rawImage_ )
      {
	cvCvtColor( rawImage_, yuvImage_, CV_BGR2YCrCb );//YUV For codebook method
	//This is where we build our background model
	if( !pause_ && nframes_-1 < nframesToLearnBG_  )
	  {
	    cvBGCodeBookUpdate( model_, yuvImage_ );
	    ROS_DEBUG("[BGFGCodeBook:] Learning BG, nframes: %d", nframes_);
	  }
	if( nframes_-1 == nframesToLearnBG_  )
	  cvBGCodeBookClearStale( model_, model_->t/2 );
            
	//Find the foreground if any
	if( nframes_-1 >= nframesToLearnBG_  )
	  {
	    ROS_DEBUG("[BGFGCodeBook:] Finding FG (modMin: %d , modMax: %d)",model_->modMin[0],model_->modMax[0]);
	    // Find foreground by codebook method
	    cvBGCodeBookDiff( model_, yuvImage_, ImaskCodeBook_ );
	    // This part just to visualize bounding boxes and centers if desired
	    cvCopy(ImaskCodeBook_,ImaskCodeBookCC_);	
	    cvSegmentFGMask( ImaskCodeBookCC_ );
	  }
	//Display
	cvShowImage( "Raw", rawImage_ );
	cvShowImage( "ForegroundCodeBook",ImaskCodeBook_);
	cvShowImage( "CodeBook_ConnectComp",ImaskCodeBookCC_);
      }
    // User input:
    c_ = cvWaitKey(10)&0xFF;
    c_ = tolower(c_);

    if(c_ == 27 || c_ == 'q')
      {
	ros::shutdown();
	return;
      }
    //Else check for user input
    switch( c_ )
      {
      case 'h':
	help();
	break;
      case 'p':
	pause_ = !pause_;
	break;
      case 's':
	singlestep_ = !singlestep_;
	pause_ = false;
	break;
      case 'r':
	pause_ = false;
	singlestep_ = false;
	break;
      case ' ':
	cvBGCodeBookClearStale( model_, 0 );
	nframes_ = 0;
	break;
	//CODEBOOK PARAMS
      case 'y': case '0':
      case 'u': case '1':
      case 'v': case '2':
      case 'a': case '3':
      case 'b': 
	ch_[0] = c_ == 'y' || c_ == '0' || c_ == 'a' || c_ == '3';
	ch_[1] = c_ == 'u' || c_ == '1' || c_ == 'a' || c_ == '3' || c_ == 'b';
	ch_[2] = c_ == 'v' || c_ == '2' || c_ == 'a' || c_ == '3' || c_ == 'b';
	printf("CodeBook YUV Channels active: %d, %d, %d\n", ch_[0], ch_[1], ch_[2] );
	break;
      case 'i': //modify max classification bounds (max bound goes higher)
      case 'o': //modify max classification bounds (max bound goes lower)
      case 'k': //modify min classification bounds (min bound goes lower)
      case 'l': //modify min classification bounds (min bound goes higher)
	{
	  uchar* ptr = c_ == 'i' || c_ == 'o' ? model_->modMax : model_->modMin;
	  for(n_=0; n_<nChannels_; n_++)
            {
	      if( ch_[n_] )
                {
		  int v = ptr[n_] + (c_ == 'i' || c_ == 'l' ? 1 : -1);
		  ptr[n_] = CV_CAST_8U(v);
                }
	      printf("%d,", ptr[n_]);
            }
	  printf(" CodeBook %s Side\n", c_ == 'i' || c_ == 'o' ? "High" : "Low" );
	}
	break;
      }

  }


protected:

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  sensor_msgs::CvBridge bridge_;
  std::string input_image_topic_;

  //VARIABLES for CODEBOOK METHOD:
  CvBGCodeBookModel* model_;
  int modMin_, modMax_, cbBounds_;
  //int NCHANNELS_;
  bool ch_[3]; // This sets what channels should be adjusted for background bounds
  IplImage* rawImage_, *yuvImage_; //yuvImage is for codebook method
  IplImage *ImaskCodeBook_, *ImaskCodeBookCC_;  
  int c_, n_, nframes_, nChannels_;
  int nframesToLearnBG_;
  bool pause_;
  bool singlestep_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bgfg_codebook");
  ros::NodeHandle n("~");
  BGFGCodeBook bg(n);
  ros::spin();
  return 0;
}
