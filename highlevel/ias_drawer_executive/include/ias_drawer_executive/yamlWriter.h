#include <iostream>
//yaml reading and writing
#include <opencv2/core/core.hpp>
#include <fstream>
#include <tf/tf.h>

class yamlWriter 
{
public:
  int writeTrajectory(const std::string & filename, const std::vector<tf::Stamped<tf::Pose> > & opening_trajectory);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Get a string representation of the name of this class. */
  std::string getName () const { return ("[yamlWriter: ]"); }
};


int yamlWriter::writeTrajectory(const std::string & filename, const std::vector<tf::Stamped<tf::Pose> > & opening_trajectory)
{

    cv::FileStorage fs;
    fs.open(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened())
    {
      std::cerr << "bad yml file!" << std::endl;
      return -1;
    }

    cv::Mat mat_trajectory;
    mat_trajectory = cv::Mat::zeros(opening_trajectory.size(), 7, CV_32F);
    for (uint i = 0; i < opening_trajectory.size(); i++)
      {
	mat_trajectory.at<float>(i, 0) = opening_trajectory[i].getOrigin().x();
	mat_trajectory.at<float>(i, 1) = opening_trajectory[i].getOrigin().y();
	mat_trajectory.at<float>(i, 2) = opening_trajectory[i].getOrigin().z();
	mat_trajectory.at<float>(i, 3) = opening_trajectory[i].getRotation().x();
	mat_trajectory.at<float>(i, 4) = opening_trajectory[i].getRotation().y();
	mat_trajectory.at<float>(i, 5) = opening_trajectory[i].getRotation().z();
	mat_trajectory.at<float>(i, 6) = opening_trajectory[i].getRotation().w();
      }
    fs << "trajectory " << mat_trajectory;
    //cv::FileNode fn = fs["cabinet" + ss.str ()];
    /* std::stringstream ss; */
    /* ss << 100; */
    /* cv::FileNode fn = fs["cabinet" + ss.str()]; */
    /* cv::FileNodeIterator it = fn.begin(); */
    /* while (it != fn.end()) */
    /*   { */
    /* 	cv::FileNode n = *it; */
    /* 	std::cerr << "n.name: " << n.name() << std::endl; */
    /* 	if (n.name() == "type" && n.size() == 1) */
    /* 	  { */
    /* 	    //	    unique_id++; */
    /* 	    // cabinet.id = unique_id; */
    /* 	    std::cerr << "writting my string" << std::endl; */
    /* 	    std::string(n) == "my_string"; */
    /* 	  } */
    /*   } */
    fs.release();
    return 1;
}
