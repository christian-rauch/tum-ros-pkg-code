#include <map>
#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <planning_environment/models/robot_models.h>

class JointStateAccumulator {

public:

  JointStateAccumulator() 
  {
    robot_model_ = new planning_environment::RobotModels("robot_description");
    
    planning_models::KinematicModel* km = robot_model_->getKinematicModel().get();
    unsigned int num = 0;
    std::vector<const planning_models::KinematicModel::Joint*> joints; 
    km->getJoints(joints);
    for(unsigned int i = 0; i < joints.size(); i++) {
      if(joints[i]->usedParams > 0) {
        joint_indices_[joints[i]->name] = num++;
        joint_values_[joints[i]->name] = 0.0;
        joint_velocities_[joints[i]->name] = 0.0;
      }
    }

    joint_state_subscriber_ = root_handle_.subscribe("joint_states", 1, &JointStateAccumulator::jointStateCallback, this);

    joint_state_publisher_ = root_handle_.advertise<sensor_msgs::JointState>("accumulated_joint_states", 1);
  }

  void jointStateCallback(const sensor_msgs::JointStateConstPtr &jointState) {
    if (jointState->name.size() != jointState->position.size() || jointState->name.size() !=jointState->velocity.size())
    {
      ROS_ERROR("Planning environment received invalid joint state");
      return;
    }
    for (unsigned int i = 0 ; i < jointState->name.size(); ++i)
    {
      joint_values_[jointState->name[i]] = jointState->position[i];
      joint_velocities_[jointState->name[i]] = jointState->position[i];
    }
  }

  void publishAccumlatedJointState() {
    
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(joint_indices_.size());
    joint_state.position.resize(joint_indices_.size());
    joint_state.velocity.resize(joint_indices_.size());
    for(std::map<std::string, unsigned int>::iterator it = joint_indices_.begin(); it != joint_indices_.end(); it++) {
      unsigned int ind = it->second;
      joint_state.name[ind] = it->first;
      joint_state.position[ind] = joint_values_[it->first];
      joint_state.velocity[ind] = joint_velocities_[it->first];
    }
    joint_state_publisher_.publish(joint_state);
  }
  

private:
  
  ros::NodeHandle root_handle_;
  planning_environment::RobotModels* robot_model_;

  ros::Publisher joint_state_publisher_;
  ros::Subscriber joint_state_subscriber_;

  std::map<std::string, unsigned int> joint_indices_;
  std::map<std::string, double> joint_values_;
  std::map<std::string, double> joint_velocities_;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_state_accumulator");
  
  ros::AsyncSpinner spinner(2); // Use 1 thread
  spinner.start();

  JointStateAccumulator jsa;

  ros::Rate pub_rate(10.0);
  
  while (ros::ok()) 
  {
    jsa.publishAccumlatedJointState();
    pub_rate.sleep();
  }
  ros::shutdown();
  return 0;
}
