#include <pluginlib/class_list_macros.h>
#include <image_algos/image_algos.h>
#include <image_algos/color_find_hsv.h>
#include <image_algos/pcd_to_image_projector_algo.h>

using namespace image_algos;

PLUGINLIB_DECLARE_CLASS(image_algos, ColorFindHSV, image_algos::ColorFindHSV, image_algos::ImageAlgo);
PLUGINLIB_DECLARE_CLASS(image_algos, PCDToImageProjector, image_algos::PCDToImageProjector, image_algos::ImageAlgo);

