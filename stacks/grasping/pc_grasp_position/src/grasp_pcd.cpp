
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <SimpleGraspPlanner.h>
#include <geometry_msgs/Pose.h>


using namespace sensor_msgs;

bool s_need_cloud_data = true;
ros::Publisher pub;
std::string s_input_cloud_topic = "graspable_segments";
std::string s_output_cloud_topic = "hand_config";

/**
  These parameters depends on the rotation of the robot to the object
  only the first quadrant is searched, with this value the quadrant can be shifted.
  Robots might have different properties, so the quadrant to search might change for
  higher tables and higher grasps, so there are two values wich can be set on every
  query to sgp.

  In this example we assume the object to be in the right orientation towards the
  robot.
*/
double param_start_top = 0.0;
double param_start_side = 0.0;
/**
*  This callback calculates mean and cov from the incoming cloud and feeds it to sgp
*/
void cloud_cb (const PointCloudConstPtr& cloud)
{
  /*
  Cite sgp:

  typedef struct
  {
     double sx; double sxy; double sxz;
     double syx; double sy; double syz;
      double szx; double szy; double sz;
  }
  CovariancePoint;
  */
  CovariancePoint cov;   /** > grasping target, in order to support obstacles this must be a list */
  Point3D mean, a, b, c1,c2,c3;
  mean.x = 0.0;
  mean.y = 0.0;
  mean.z = 0.0;
  for(size_t i = 0; i < (*cloud).points.size(); i++)
  {
    mean.x += (*cloud).points[i].x;
    mean.y += (*cloud).points[i].y;
    mean.z += (*cloud).points[i].z;
  }
  mean.x /=  (*cloud).points.size();
  mean.y /=  (*cloud).points.size();
  mean.z /=  (*cloud).points.size();
  cov.sx  = 0.0;
  cov.sxy = 0.0;
  cov.sxz = 0.0;
  cov.syx = 0.0;
  cov.sy  = 0.0;
  cov.syz = 0.0;
  cov.szx = 0.0;
  cov.szy = 0.0;
  cov.sz  = 0.0;
  for(size_t i = 0; i < (*cloud).points.size(); i++)
  {
     double tmpx = (*cloud).points[i].x - mean.x;
     double tmpy = (*cloud).points[i].y - mean.y;
     double tmpz = (*cloud).points[i].z - mean.z;

     cov.sx  += tmpx*tmpx;
     cov.sxy += tmpx*tmpy;
     cov.sxz += tmpx*tmpz;
     cov.syx += tmpy*tmpx;
     cov.sy  += tmpy*tmpy;
     cov.syz += tmpy*tmpz;
     cov.szx += tmpz*tmpx;
     cov.szy += tmpz*tmpy;
     cov.sz  += tmpz*tmpz;
  }
  cov.sx  /= (*cloud).points.size();
  cov.sxy /= (*cloud).points.size();
  cov.sxz /= (*cloud).points.size();
  cov.syx /= (*cloud).points.size();
  cov.sy  /= (*cloud).points.size();
  cov.syz /= (*cloud).points.size();
  cov.szx /= (*cloud).points.size();
  cov.szy /= (*cloud).points.size();
  cov.sz  /= (*cloud).points.size();
  HandConfig cfg = GetGraspLM(&mean, &cov,1, param_start_top, param_start_side);
  geometry_msgs::Point32 pout;
  pout.x= cfg.alpha;
  pout.y = cfg.beta;
  pout.z  = cfg.delta_max;
  /**
    The results can be Transformed r.g. with TransformPoint from the sgp library
    example would be:

  */
  b.x = 0.0;
  b.y = 0.0;
  b.z = 0.0;
  a.x = 1.0;
  a.y = 0.0;
  a.z = 0.0;
  c1 = TransformPoint(a, b, cfg.alpha, cfg.beta, cfg.delta_max) ;
  a.x = 0.0;
  a.y = 1.0;
  a.z = 0.0;
  c2 = TransformPoint(a, b, cfg.alpha, cfg.beta, cfg.delta_max) ;
  a.x = 0.0;
  a.y = 0.0;
  a.z = 1.0;
  c3 = TransformPoint(a, b, cfg.alpha, cfg.beta, cfg.delta_max) ;
  geometry_msgs::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;

  /** Transform to quaternion */
  /** Calculate Trace*/
  double S, T = 1 + c1.x + c2.y + c3.z;


  /* If the trace of the matrix is greater than zero, then
  perform an "instant" calculation.
  Important note wrt. rouning errors: */
  if ( T > 0.00000001 )
  {
      S = sqrt(T) * 2;
      pose.orientation.x = ( c2.z - c3.y ) / S;
      pose.orientation.y = ( c3.x - c1.z ) / S;
      pose.orientation.z = ( c1.y - c2.x ) / S;
      pose.orientation.w = 0.25 * S;
  }
  else
  {

    /*If the trace of the matrix is equal to zero then identify
    which major diagonal element has the greatest value.
    Deending on this, calculate the following:*/

    if ( c1.x > c2.y && c1.x > c3.z )  {	// Column 0:
        S  = sqrt( 1.0 + c1.x - c2.y - c3.z ) * 2;
        pose.orientation.x = 0.25 * S;
        pose.orientation.y = (c1.y + c2.x ) / S;
        pose.orientation.z = (c3.x + c1.z ) / S;
        pose.orientation.w = (c2.z - c3.y ) / S;
    } else if ( c2.y > c3.z ) {			// Column 1:
        S  = sqrt( 1.0 + c2.y - c1.x - c3.z ) * 2;
        pose.orientation.x = (c1.y + c2.x ) / S;
        pose.orientation.y = 0.25 * S;
        pose.orientation.z = (c2.z + c3.y ) / S;
        pose.orientation.w = (c3.x - c1.z ) / S;
    } else {						// Column 2:
        S  = sqrt( 1.0 + c3.z - c1.x - c2.y ) * 2;
        pose.orientation.x = (c3.x + c1.z ) / S;
        pose.orientation.y = (c2.z + c3.y ) / S;
        pose.orientation.z = 0.25 * S;
        pose.orientation.w = (c1.y - c2.x ) / S;
    }
  }

  pub.publish(pose);
}

#ifndef MAX_HAND_POINTS
#define MAX_HAND_POINTS 100
#endif
Point3D hand_points[MAX_HAND_POINTS];

void GetParams(ros::NodeHandle &n)
{
  using namespace XmlRpc;
  int counting_hand_points = 0;

  if( n.hasParam( "sgp_config" ) )
  {
      XmlRpcValue sgp_config_param_value;
      n.getParam( "sgp_config", sgp_config_param_value );
      if( sgp_config_param_value.getType() != XmlRpcValue::TypeArray )
      {
          printf("Type %d\n", sgp_config_param_value.getType());
          ROS_WARN("sgp_config has invalid type.");
      }
      else
      {
        for(int i=0; i<sgp_config_param_value.size(); i++)
        {
            if( sgp_config_param_value[i].getType() != XmlRpcValue::TypeArray && sgp_config_param_value[i].size() != 3)
            {
               ROS_WARN("Ignoring sgp_config entry: Not a point consisting of three real values");
               continue;
            }

            if( sgp_config_param_value[i][0].getType() !=  XmlRpcValue::TypeDouble ||
                sgp_config_param_value[i][1].getType() !=  XmlRpcValue::TypeDouble ||
                sgp_config_param_value[i][2].getType() !=  XmlRpcValue::TypeDouble)
            {
              ROS_WARN("Ignoring sgp_config entry: Not a point consisting of three real values");
              continue;
            }
            hand_points[counting_hand_points].x = sgp_config_param_value[i][0];
            hand_points[counting_hand_points].y = sgp_config_param_value[i][1];
            hand_points[counting_hand_points].z = sgp_config_param_value[i][2];
            printf("Added point %d with %f %f %f\n", counting_hand_points, hand_points[counting_hand_points].x,
                        hand_points[counting_hand_points].y, hand_points[counting_hand_points].z);
            counting_hand_points++;
        }
      }
  }
  else
    printf("sgp config not set!!\n\n");
  if(n.hasParam( "sgp_config_param_start_top"))
  {
    XmlRpcValue sgp_config_param_value;
    n.getParam( "sgp_config_param_start_top", sgp_config_param_value );

    if(sgp_config_param_value.getType() !=  XmlRpcValue::TypeDouble)
       ROS_WARN("Ignoring sgp_config_paramt_start_top: Not a real value");
    else
    {
      param_start_top = sgp_config_param_value;
    }
  }
  if(n.hasParam( "sgp_config_param_start_side"))
  {
    XmlRpcValue sgp_config_param_value;
    n.getParam( "sgp_config_param_start_side", sgp_config_param_value );
    if(sgp_config_param_value.getType() !=  XmlRpcValue::TypeDouble)
       ROS_WARN("Ignoring sgp_config_param_start_side: Not a real value");
    else
    {
      param_start_side = sgp_config_param_value;
    }
  }
  InitSGP(hand_points, counting_hand_points, 0.0);
}

int main(int argc, char* argv[])
{
  ros::init (argc, argv, "trans_pcd_jlo");
  ros::NodeHandle nh("~");

  ros::Subscriber cloud_sub = nh.subscribe (s_input_cloud_topic, 1, &cloud_cb);
  pub = nh.advertise<geometry_msgs::Pose>(s_output_cloud_topic, 1);
  GetParams(nh);
  while(nh.ok())
  {
    ros::spinOnce ();
  }
  return 0;
}
