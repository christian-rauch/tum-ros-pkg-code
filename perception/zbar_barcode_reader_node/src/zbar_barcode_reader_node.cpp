
#include <ros/ros.h>
#include <ros/node_handle.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <string.h>
#include "std_msgs/String.h"
#include <sstream>

//Magick++ lib
#include <Magick++.h>
//zbar
#include <zbar.h>

using namespace std;
using namespace zbar;

class BarcodeReaderNode {

public:
  BarcodeReaderNode(ros::NodeHandle &n) :
    n_(n), it_(n_)
  {
    n_.param ("input_image_topic", input_image_topic_, std::string("/stereo/left/image_rect"));
    n_.param ("outout_barcode_topic", output_barcode_topic_, std::string("barcode"));
    image_sub_ = it_.subscribe(
                               input_image_topic_, 1, &BarcodeReaderNode::imageCallback, this);
    barcode_pub_ = n_.advertise<std_msgs::String>(output_barcode_topic_, 1);
  }

  ~BarcodeReaderNode()
  {

  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
  {

    IplImage *cv_image = NULL;
    ROS_INFO("[BarcodeReaderNode: ] Image received");
    try
      {
        cv_image = bridge_.imgMsgToCv(msg_ptr, "passthrough");
      }
    catch (sensor_msgs::CvBridgeException error)
      {
        ROS_ERROR("error");
      }
    
    // create a reader
    ImageScanner scanner;

    // configure the reader
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

    // obtain image data
    //    IplImage * cv_image = cvLoadImage(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
    Magick::Blob blob(cv_image->imageData, cv_image->imageSize);

    int width = cv_image->width;   // extract dimensions
    int height = cv_image->height;
    const void *raw = blob.data();

    // wrap image data
    Image image(width, height, "Y800", raw, width * height);

    // scan the image for barcodes
    int n = scanner.scan(image);

    // extract results
    for(Image::SymbolIterator symbol = image.symbol_begin();
        symbol != image.symbol_end();
        ++symbol) 
    {
      // do something useful with results
      ROS_INFO_STREAM("Publishing: barcode type: " << symbol->get_type_name()
                      << " barcode value " << symbol->get_data());

      std_msgs::String msg;
      std::stringstream ss;
      ss << symbol->get_type_name() << "," << symbol->get_data();
      msg.data = ss.str();
      barcode_pub_.publish(msg);
    }
    if (n == 0)
      ROS_WARN("Barcode not found");

    if (n < 0)
      ROS_ERROR("Error occured while finding barcode");
    // clean up
    image.set_data(NULL, 0);

  }

protected:

  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher barcode_pub_;
  sensor_msgs::CvBridge bridge_;
  std::string input_image_topic_, output_barcode_topic_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "barcode_reader_node");
  ros::NodeHandle n("~");
  BarcodeReaderNode br(n);
  ros::spin();
  return 0;
}
