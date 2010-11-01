#include <ros/ros.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
int main(int argc, char **argv){
  ros::init (argc, argv, "get_ik_solver_info");
  ros::NodeHandle rh;
  ros::service::waitForService("rosie_right_arm_kinematics/get_ik_solver_info");
  ros::ServiceClient query_client = rh.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("rosie_right_arm_kinematics/get_ik_solver_info");

  // define the service messages
  kinematics_msgs::GetKinematicSolverInfo::Request request;
  kinematics_msgs::GetKinematicSolverInfo::Response response;

  if(query_client.call(request,response))
  {
    for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
    {
      ROS_INFO("Joint: %d %s",i,response.kinematic_solver_info.joint_names[i].c_str());
    }
  }
  else
  {
    ROS_ERROR("Could not call query service");
  }
  ros::shutdown();
}


