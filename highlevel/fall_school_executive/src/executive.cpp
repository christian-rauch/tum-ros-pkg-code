#include <ros/ros.h>

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "executive");
  ros::spin();
  return 0;
}
