
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

class BarcodeReaderNode {
	std:: string link1_;// = "http://www.barcoo.com/api/get_product_complete?pi=";
	std:: string link2_;// = "&pins=ean&format=xml&source=ias-tum";
	std:: string tag1_;// = "answer";
	std:: string tag2_;// = "picture_high";
	std:: string tag3_;// = "picture_low";
	std:: string pattern_;// = "<meta property=\"og:image\" content=\"";
	TiXmlDocument doc;

public:
  struct memoryStruct {
    char *memory;
    size_t size;
  };

  struct UserData {
    memoryStruct *memory;
    BarcodeReaderNode *self;
    UserData(memoryStruct *memory, BarcodeReaderNode *self)
      : memory(memory), self(self) {}
  };

  BarcodeReaderNode(ros::NodeHandle &n) :
    n_(n), it_(n_)
  {
	  //link1 = "http://www.barcoo.com/api/get_product_complete?pi=";
	  //link2 = "&pins=ean&format=xml&source=ias-tum";
	  //tag1 = "answer";
	  //tag2 = "picture_high";
	  //tag3 = "picture_low";

    n_.param ("input_image_topic", input_image_topic_, std::string("/image_raw"));
    n_.param ("outout_barcode_topic", output_barcode_topic_, std::string("barcode"));
    n_.param ("link1", link1_, std::string("http://www.barcoo.com/api/get_product_complete?pi="));
    n_.param ("link2", link2_, std::string("&pins=ean&format=xml&source=ias-tum"));
    n_.param ("tag2", tag2_, std::string("picture_high"));
    n_.param ("tag3", tag3_, std::string("picture_low"));
    n_.param ("image_pattern", pattern_, std::string("<meta property=\"og:image\" content=\""));
    image_sub_ = it_.subscribe(
                               input_image_topic_, 1, &BarcodeReaderNode::imageCallback, this);
    barcode_pub_ =
    n_.advertise<std_msgs::String>(output_barcode_topic_, 1);
    cv::namedWindow ("Barcoo img");
  }

  ~BarcodeReaderNode()
  {
    cv::destroyAllWindows();
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

  static size_t WriteMemoryCallback
  (void *ptr, size_t size, size_t nmemb, void *data)
  {
    UserData *userdata = reinterpret_cast<UserData *>(data);
    struct memoryStruct *mem = userdata->memory;
    //BarcodeReaderNode *self = userdata->self;
    size_t realsize = size * nmemb;
    
    mem->memory = (char *)
      CURL_realloc(mem->memory, mem->size + realsize + 1);
    if (mem->memory) {
      memcpy(&(mem->memory[mem->size]), ptr, realsize);
      mem->size += realsize;
      mem->memory[mem->size] = 0;
    }
    return realsize;
  }

    // This is the writer call back function used by curl  
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
 
  int getImage(std::string image, cv::Mat * imgTmp)
  {
    CURL *curl;       // CURL objects
    CURLcode res;
    memoryStruct buffer; // memory buffer
    UserData userdata(&buffer, this);
    
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

      curl_easy_setopt(curl,CURLOPT_WRITEFUNCTION, &BarcodeReaderNode::WriteMemoryCallback);
      curl_easy_setopt(curl,CURLOPT_WRITEDATA, (void *) &userdata);
      
      // get the image from the specified URL
      res = curl_easy_perform(curl);
      // decode memory buffer using OpenCV
      *imgTmp = cv::imdecode(cv::Mat(1, buffer.size, CV_8UC1, buffer.memory), CV_LOAD_IMAGE_UNCHANGED);
      // always cleanup

      curl_easy_cleanup(curl);
      free(buffer.memory);
  }
  return 1;
  }

  void findElement( TiXmlNode* pParent, std::string & picture, std:: string tag)
  {
    //if ( !pParent ) return;
    
    TiXmlNode* pChild;
    std::string pstring = pParent->Value();
    
    //Search for tag
    size_t found=pstring.find(tag);
    if (found!=std::string::npos)
    {
    	ROS_INFO_STREAM("First child: " << pParent->FirstChild()->Value());
    	picture =  pParent->FirstChild()->Value();
    	return;
    }
    
    for( pChild = pParent->FirstChild(); pChild != 0; pChild = pChild->NextSibling()) 
    {	
    	findElement(pChild, picture,tag);
    }
  }


  int getPictureLink(std::string buffer,std::string & picture)
  {
    doc.Parse((const char*)buffer.c_str(), 0, TIXML_ENCODING_UTF8);
    
    //Check if there is a result
    std:: string result;
    cout << "ohhh my goooooood\n\n";
    findElement(&doc,result,tag1_);
    if(result.compare("0") == 0) //This condition checks if there is a result in the XML file if not returns 0
    {
    	cout << "heloooooo barcode does not correspond to any object in the barcoo database";
    	return 0;
    }
    else
    	cout << "ELSE ELSE ELSE\n";
    
    //Search for first tag
    findElement (&doc, picture,tag2_);

    if(picture == "")
       	findElement (&doc, picture,tag3_); 	//Search for second tag

    ROS_INFO_STREAM ("Picture link: " << picture);
    if (picture == "")
      return -1;
    
    return 1;
  }


  int getBarcooXML(std::string bar_code, std::string & buffer)
  {
    char errorBuffer[CURL_ERROR_SIZE];
    // Our curl objects  
    CURL *curl;  
    CURLcode result;  
    // Create our curl handle  
    curl = curl_easy_init();  
    std::string full_url = link1_ + bar_code + link2_;
    ROS_INFO_STREAM("full_url: " << full_url);
   
    if (curl)  
      {  
	// Now set up all of the curl options  
	curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, errorBuffer);  
	curl_easy_setopt(curl, CURLOPT_URL, full_url.c_str());  
	curl_easy_setopt(curl, CURLOPT_HEADER, 0);  
	curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1);  
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &BarcodeReaderNode::writer);  
    //	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writer);  
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
  
  int getHTMLpage(std::string url, std::string & buffer)
  {
	  char errorBuffer[CURL_ERROR_SIZE];
	  // Our curl objects
	  CURL *curl;
	  CURLcode result;
	  // Create our curl handle
	  curl = curl_easy_init();
      if (curl)
      {
    	  // Now set up all of the curl options
    	  curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, errorBuffer);
    	  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    	  curl_easy_setopt(curl, CURLOPT_HEADER, 0);
    	  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1);
    	  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &BarcodeReaderNode::writer);
	      //	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writer);
    	  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer);

    	  // Attempt to retrieve the remote page
    	  result = curl_easy_perform(curl);

    	  // Always cleanup
    	  curl_easy_cleanup(curl);

    	  // Did we succeed?
    	  if (result == CURLE_OK)
	  	  {

	  	    return 1;
	  	  }
    	  else
	  	  {
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
    //Get xml file from barcoo database
    int res = getBarcooXML(ss.str(), buffer);
    if(res != 1)
    	return;
    																						//	std::cerr << "buffer after callBarcoo " << buffer << std::endl;
    //Search for info in the xml file
    std::string pictureLink;
    res = getPictureLink (buffer,pictureLink);


    if(res == 0) //If barcode does not exist in the database return
    {
    	cout << "Barcode does not exist in the barcoo datbase\n";
    	return;
    }
    if(res == -1) //This condition is true when the image link is not given
    {
    	//Look for an image in the HTML file

    	//First find the link to the barcoo website of the file
    	std::string htmlPage;
    	std::string htmlLink;
    	findElement(&doc,htmlLink,"back_link");
    	cout << "html link:  "+htmlLink+"\n";
    	//Download HTML page
    	getHTMLpage(htmlLink,htmlPage);
    	std::string ss  = pattern_;
    	long position = htmlPage.find(ss, ss.length()) + ss.length();

    	std::string link;
    	while(htmlPage.at(position) !='"')
    	{
    		link += htmlPage.at(position);
    		position++;
    	}
    	pictureLink = link;

    }
    //Get Image
    if(!pictureLink.empty())
    {
    	cv::Mat  imgTmp; // image object
    	getImage (pictureLink ,& imgTmp);
    	// display image
    	if (!(imgTmp.empty()))
    	{
    		imshow("Barcoo img", imgTmp);
    	}
    }
    cv::waitKey(3);
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
