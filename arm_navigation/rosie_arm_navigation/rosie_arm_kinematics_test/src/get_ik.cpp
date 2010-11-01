#include <ros/ros.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionIK.h>

int main(int argc, char **argv){
  ros::init (argc, argv, "get_ik");
  ros::NodeHandle rh;

  ros::service::waitForService("rosie_right_arm_kinematics/get_ik_solver_info");
  ros::service::waitForService("rosie_right_arm_kinematics/get_ik");

  ros::ServiceClient query_client = rh.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("rosie_right_arm_kinematics/get_ik_solver_info");
  ros::ServiceClient ik_client = rh.serviceClient<kinematics_msgs::GetPositionIK>("rosie_right_arm_kinematics/get_ik");

  // define the service messages
  kinematics_msgs::GetKinematicSolverInfo::Request request;
  kinematics_msgs::GetKinematicSolverInfo::Response response;

  if(query_client.call(request,response))
  {
    for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
    {
      ROS_DEBUG("Joint: %d %s",i,response.kinematic_solver_info.joint_names[i].c_str());
    }
  }
  else
  {
    ROS_ERROR("Could not call query service");
    ros::shutdown();
    exit(1);
  }
  // define the service messages
  kinematics_msgs::GetPositionIK::Request  gpik_req;
  kinematics_msgs::GetPositionIK::Response gpik_res;
  gpik_req.timeout = ros::Duration(5.0);
  gpik_req.ik_request.ik_link_name = "right_arm_hand_link";

  gpik_req.ik_request.pose_stamped.header.frame_id = "calib_right_arm_base_link";
  gpik_req.ik_request.pose_stamped.pose.position.x = 0.078357;
  gpik_req.ik_request.pose_stamped.pose.position.y = 0.098053;
  gpik_req.ik_request.pose_stamped.pose.position.z = 1.07627;

  gpik_req.ik_request.pose_stamped.pose.orientation.x = 0.167578;
  gpik_req.ik_request.pose_stamped.pose.orientation.y = 0.706558;
  gpik_req.ik_request.pose_stamped.pose.orientation.z = 0.679779;
  gpik_req.ik_request.pose_stamped.pose.orientation.w = 0.102928;


  
  gpik_req.ik_request.pose_stamped.header.frame_id = "base_link";
  gpik_req.ik_request.pose_stamped.pose.position.x = 0.268555;
  gpik_req.ik_request.pose_stamped.pose.position.y = -0.982867;
  gpik_req.ik_request.pose_stamped.pose.position.z = 1.70482;

  gpik_req.ik_request.pose_stamped.pose.orientation.x = 0.724440;
  gpik_req.ik_request.pose_stamped.pose.orientation.y = -0.213505;
  gpik_req.ik_request.pose_stamped.pose.orientation.z = -0.623758;
  gpik_req.ik_request.pose_stamped.pose.orientation.w = 0.201316;
  








  gpik_req.ik_request.ik_seed_state.joint_state.position.resize(response.kinematic_solver_info.joint_names.size());
  gpik_req.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names;
  for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
  {
    gpik_req.ik_request.ik_seed_state.joint_state.position[i] = (response.kinematic_solver_info.limits[i].min_position + response.kinematic_solver_info.limits[i].max_position)/2.0;
    //gpik_req.ik_request.ik_seed_state.joint_state.position[i] = 0.5;
    printf("Joint %d: %f\n", i, gpik_req.ik_request.ik_seed_state.joint_state.position[i]);
  }
  if(ik_client.call(gpik_req, gpik_res))
  {
    if(gpik_res.error_code.val == gpik_res.error_code.SUCCESS)
      for(unsigned int i=0; i < gpik_res.solution.joint_state.name.size(); i ++)
        ROS_INFO("Joint: %s %f",gpik_res.solution.joint_state.name[i].c_str(),gpik_res.solution.joint_state.position[i]);
    else
      ROS_ERROR("Inverse kinematics failed");
  }
  else
    ROS_ERROR("Inverse kinematics service call failed");
  ros::shutdown();
}

