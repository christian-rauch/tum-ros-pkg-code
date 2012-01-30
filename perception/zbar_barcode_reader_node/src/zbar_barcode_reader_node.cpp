
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
#include "curl/curl.h" 
#include "tinyxml.h"
#include <highgui.h>

//Magick++ lib
#include <Magick++.h>
//zbar
#include <zbar.h>

using namespace std;
using namespace zbar;

 static int writer(char *data, size_t size, size_t nmemb,  
		      std::string *buffer)  
    {  
      // What we will return  
      int result = 0;  
  
      // Is there anything in the buffer?  
      if (buffer != NULL)  
	{  
	  // Append the data to the buffer  
	  buffer->append(data, size * nmemb);  
  
	  // How much did we write?  
	  result = size * nmemb;  
	}  
  
      return result;  
    }  

static void* CURL_realloc(void *ptr, size_t size)
{
  /* There might be a realloc() out there that doesn't like reallocing
     NULL pointers, so we take care of it here */
  if(ptr)
    return realloc(ptr, size);
  else
    return malloc(size);
}

struct memoryStruct {
  char *memory;
  size_t size;
};

  size_t WriteMemoryCallback
  (void *ptr, size_t size, size_t nmemb, void *data)
  {
    size_t realsize = size * nmemb;
    struct memoryStruct *mem = (struct memoryStruct *)data;
    
    mem->memory = (char *)
      CURL_realloc(mem->memory, mem->size + realsize + 1);
    if (mem->memory) {
      memcpy(&(mem->memory[mem->size]), ptr, realsize);
      mem->size += realsize;
      mem->memory[mem->size] = 0;
    }
    return realsize;
  }


class BarcodeReaderNode {

  // struct memoryStruct {
  //   char *memory;
  //   size_t size;
  // };
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
  // void* CURL_realloc(void *ptr, size_t size)
  // {
  //   /* There might be a realloc() out there that doesn't like reallocing
  //      NULL pointers, so we take care of it here */
  //   if(ptr)
  //     return realloc(ptr, size);
  //   else
  //     return malloc(size);
  // }

  // size_t WriteMemoryCallback
  // (void *ptr, size_t size, size_t nmemb, void *data)
  // {
  //   size_t realsize = size * nmemb;
  //   struct memoryStruct *mem = (struct memoryStruct *)data;
    
  //   mem->memory = (char *)
  //     CURL_realloc(mem->memory, mem->size + realsize + 1);
  //   if (mem->memory) {
  //     memcpy(&(mem->memory[mem->size]), ptr, realsize);
  //     mem->size += realsize;
  //     mem->memory[mem->size] = 0;
  //   }
  //   return realsize;
  // }
    // This is the writer call back function used by curl  
  // int writer(char *data, size_t size, size_t nmemb,  
  // 		    std::string *buffer)  
  // {  
  //   // What we will return  
  //   int result = 0;  
  
  //   // Is there anything in the buffer?  
  //   if (buffer != NULL)  
  //     {  
  // 	// Append the data to the buffer  
  // 	buffer->append(data, size * nmemb);  
  
  // 	// How much did we write?  
  // 	result = size * nmemb;  
  //     }  
  
  //   return result;  
  // }  
  int getImage(std::string & image)
  {
    CURL *curl;       // CURL objects
    CURLcode res;
    cv::Mat imgTmp; // image object
    memoryStruct buffer; // memory buffer
    
    curl = curl_easy_init(); // init CURL library object/structure
    
    if(curl) 
      {
      // set up the write to memory buffer
      // (buffer starts off empty)
      buffer.memory = NULL;
      buffer.size = 0;
      
      // (N.B. check this URL still works in browser in case image has moved)
      curl_easy_setopt(curl, CURLOPT_URL, image.c_str());
      curl_easy_setopt(curl, CURLOPT_VERBOSE, 1); // tell us what is happening
      
      // tell libcurl where to write the image (to a dynamic memory buffer)

      curl_easy_setopt(curl,CURLOPT_WRITEFUNCTION, WriteMemoryCallback);
      curl_easy_setopt(curl,CURLOPT_WRITEDATA, (void *) &buffer);
      
      // get the image from the specified URL
      res = curl_easy_perform(curl);
      // decode memory buffer using OpenCV
      imgTmp = cv::imdecode(cv::Mat(1, buffer.size, CV_8UC1, buffer.memory), CV_LOAD_IMAGE_UNCHANGED);
      // display image (if we got / decoded it correctly)
      
    
      if (!(imgTmp.empty()))
	{
	  imshow("Image from URL", imgTmp);
	}
      cv::waitKey(3);
      
      // always cleanup

      curl_easy_cleanup(curl);
      free(buffer.memory);
  }
  return 1;
  }

  void findElement( TiXmlNode* pParent, std::string & picture)
  {
    if ( !pParent ) return;
    
    TiXmlNode* pChild;
    //std::string picture ("Picture Not Found");
    std::string pstring = pParent->Value();
    //    std::cerr << "pstring: " << pstring << std::endl;
    size_t found=pstring.find("picture_high");
    if (found!=std::string::npos)
      {
	ROS_INFO_STREAM("First child: " << pParent->FirstChild()->Value());
	picture =  pParent->FirstChild()->Value();
	return;
      }
    size_t found2=pstring.find("picture_low");
    if (found2!=std::string::npos && picture == "")
      {
	ROS_INFO_STREAM("First child: " << pParent->FirstChild()->Value());
	picture =  pParent->FirstChild()->Value();
	return;
      }
    for ( pChild = pParent->FirstChild(); pChild != 0; pChild = pChild->NextSibling()) 
      {	
	findElement(pChild, picture);
	//	std::cerr << "in for " << std::endl;
      }
  }


  int visBarcooPicture(std::string buffer)
  {
    //    std::cerr << "buffer: " << buffer << std::endl;
    TiXmlDocument doc;
    doc.Parse((const char*)buffer.c_str(), 0, TIXML_ENCODING_UTF8);
    doc.SaveFile ("text.xml");
    std::string picture;
    findElement (&doc, picture);
    ROS_INFO_STREAM ("Picture link: " << picture);
    if (picture == "")
      return -1;
    getImage (picture);
    return 1;
  }


  int callBarcoo(std::string bar_code, std::string & buffer)
  {
    char errorBuffer[CURL_ERROR_SIZE];
    // Our curl objects  
    CURL *curl;  
    CURLcode result;  
    // Create our curl handle  
    curl = curl_easy_init();  
    std::string full_url = "http://www.barcoo.com/api/get_product_complete?pi=" + bar_code + "&pins=ean&format=xml&source=ias-tum";
    ROS_INFO_STREAM("full_url: " << full_url);
   
    if (curl)  
      {  
	// Now set up all of the curl options  
	curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, errorBuffer);  
	curl_easy_setopt(curl, CURLOPT_URL, full_url.c_str());  
	curl_easy_setopt(curl, CURLOPT_HEADER, 0);  
	curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1);  
	//curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &BarcodeReaderNode::writer);  
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writer);  
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer);  

	// Attempt to retrieve the remote page  
	result = curl_easy_perform(curl);  

	// Always cleanup  
	curl_easy_cleanup(curl);

	// Did we succeed?  
	if (result == CURLE_OK)  
	  {  
	    ROS_INFO_STREAM ("CURLE_OK");
	    return 1;
	  }  
	else  
	  {  
	    ROS_INFO_STREAM ( "CURLE_NOT OK");
	    ROS_ERROR_STREAM ("Error: [" << result << "] - " << errorBuffer);
	    return -1;
	  }  
      }
    return -1;
  }
  
  void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
  {

    IplImage *cv_image = NULL;
    ROS_INFO("[BarcodeReaderNode: ] Image received");
    try
      {
        cv_image = bridge_.imgMsgToCv(msg_ptr, "mono8");
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
    std::stringstream ss;
    for(Image::SymbolIterator symbol = image.symbol_begin();
        symbol != image.symbol_end();
        ++symbol) 
    {
      // do something useful with results
      ROS_INFO_STREAM("Publishing: barcode type: " << symbol->get_type_name()
                      << " barcode value " << symbol->get_data());

      std_msgs::String msg;

      //ss << symbol->get_type_name() << "," << symbol->get_data();
      ss << symbol->get_data();
      msg.data = ss.str();
      barcode_pub_.publish(msg);
    }
    if (n == 0)
      {
	ROS_WARN("Barcode not found");
	return;
      }

    if (n < 0)
      {
	ROS_ERROR("Error occured while finding barcode");
	return;
      }

    std::string buffer;
    if (callBarcoo(ss.str(), buffer) == 1)
      {
	//	std::cerr << "buffer after callBarcoo " << buffer << std::endl;
	visBarcooPicture (buffer);
      }
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
