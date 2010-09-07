// roslaunch arm_ik.launch
#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pr2_common_action_msgs/ArmMoveIKAction.h>
#include <tf/transform_listener.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>


void QuaternionToEuler(const btQuaternion &TQuat, btVector3 &TEuler)
{
   btScalar W = TQuat.getW();
   btScalar X = TQuat.getX();
   btScalar Y = TQuat.getY();
   btScalar Z = TQuat.getZ();
   float WSquared = W * W;
   float XSquared = X * X;
   float YSquared = Y * Y;
   float ZSquared = Z * Z;
   TEuler.setX(atan2f(2.0f * (Y * Z + X * W), -XSquared - YSquared + ZSquared + WSquared));
   TEuler.setY(asinf(-2.0f * (X * Z - Y * W)));
   TEuler.setZ(atan2f(2.0f * (X * Y + Z * W), XSquared - YSquared - ZSquared + WSquared));
 //   TEuler *= RADTODEG;
};


class RobotDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "cmd_vel" topic to issue commands
  ros::Publisher cmd_vel_pub_;
  //! We will be listening to TF transforms as well
  tf::TransformListener listener_;

public:
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/navigation/cmd_vel", 1);
  }


  //! Drive forward a specified distance based on odometry information
  void driveInMap(const float targetPose[])
  { 

   double distance = 0;
   tf::Stamped<tf::Pose> targetPoseMap;
   tf::Stamped<tf::Pose> targetPoseBase;

   bool done = false;

   while (ros::ok() && !done) {
  
      listener_.waitForTransform("base_footprint", "map", 
                               ros::Time(0), ros::Duration(10.0));

      targetPoseMap.frame_id_ = "map";
      targetPoseMap.stamp_ = ros::Time();
      targetPoseMap.setOrigin(btVector3( targetPose[0], targetPose[1], 0));
      targetPoseMap.setRotation(btQuaternion(0,0, targetPose[2],  targetPose[3]));

      targetPoseBase.stamp_ = ros::Time();

      listener_.transformPose("base_link", targetPoseMap, targetPoseBase);

      btVector3 euler;
      QuaternionToEuler(targetPoseBase.getRotation(),euler);
  
      ROS_INFO("target pose in base coords: %f %f %f %f ANGLE %f ", targetPoseBase.getOrigin().x(), targetPoseBase.getOrigin().y(), targetPoseBase.getRotation().z(), targetPoseBase.getRotation().w(),euler.z());

      //get the difference between the two poses
    //geometry_msgs::Twist diff = diff2D(target_pose, robot_pose);
    //tf_.transformPose(fixed_frame_, pose, fixed_pose);   

    //wait for the listener to get the first message
    listener_.waitForTransform("base_footprint", "odom_combined", 
                               ros::Time(0), ros::Duration(10.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the odometry to the base frame
    listener_.lookupTransform("base_footprint", "odom_combined", 
                              ros::Time(0), start_transform);
    
    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
   
    /*float delta = 0.01;
    float big_delta = 0.01;

    if (targetPoseBase.getOrigin().x() > delta)
      base_cmd.linear.x = 0.05;
    if (targetPoseBase.getOrigin().x() < -delta)
      base_cmd.linear.x = -0.05;

    if (targetPoseBase.getOrigin().y() > delta)
      base_cmd.linear.y = 0.05;
    if (targetPoseBase.getOrigin().y() < -delta)
      base_cmd.linear.y = -0.05;

    if (targetPoseBase.getOrigin().x() > big_delta)
      base_cmd.linear.x = 0.05;
    if (targetPoseBase.getOrigin().x() < -big_delta)
      base_cmd.linear.x = -0.05;

    if (targetPoseBase.getOrigin().y() > big_delta)
      base_cmd.linear.y = 0.05;
    if (targetPoseBase.getOrigin().y() < -big_delta)
      base_cmd.linear.y = -0.05;   

    float angle_delta = 0.01;
    if (euler.z() > angle_delta)
      base_cmd.angular.z = 0.05;
    if (euler.z() < -angle_delta)
      base_cmd.angular.z = -0.05;

    float big_angle_delta = 0.2;
    if (euler.z() > big_angle_delta)
      base_cmd.angular.z = 0.25;
    if (euler.z() < -big_angle_delta)
      base_cmd.angular.z = -0.25;  */ 

    //calulate speeds for reaching goal in 1 second and then scale them down to reasonable speeds
    float theoretic_x = targetPoseBase.getOrigin().x();
    float theoretic_y = targetPoseBase.getOrigin().y();
    float theoretic_w = euler.z();

    float scale_x = 1;    
    float scale_y = 1;    
    float scale_w = 1;    
    if (fabs(theoretic_x) > 0.05)
       scale_x = 0.05 / fabs(theoretic_x);
    if (fabs(theoretic_y) > 0.05)
       scale_y = 0.05 / fabs(theoretic_y);
    if (fabs(theoretic_w) > 0.25)
       scale_w = 0.25 / fabs(theoretic_w);

    if (fabs(theoretic_w) < 0.05)
       scale_w = 0.05 / fabs(theoretic_w);

    float scale = scale_x;
    if (scale_y < scale)
      scale = scale_y;
    if (scale_w < scale)
      scale = scale_w;

    ROS_INFO("scales %f %f %f", scale_x, scale_y, scale_w);

    scale = fabs(scale);
    if (scale > 1)
      scale = 1;

    base_cmd.linear.x = theoretic_x * scale;
    base_cmd.linear.y = theoretic_y * scale;
    base_cmd.angular.z = theoretic_w * scale;

    ROS_INFO("COMMAND %f %f %f", base_cmd.linear.x, base_cmd.linear.y, base_cmd.angular.z);
    
    ros::Rate rate(10.0);

    // if ((base_cmd.angular.z == 0) && (base_cmd.linear.y == 0) && (base_cmd.linear.x == 0))

    if ((fabs(targetPoseBase.getOrigin().x()) < 0.02) && (fabs(targetPoseBase.getOrigin().y()) < 0.02) &&  (fabs(euler.z()) < 0.01))
      done = true;
  //  while (!done && nh_.ok())
          
      //send the drive command after safety check
      if ((fabs(base_cmd.linear.x) <= 0.051) && (fabs(base_cmd.linear.y) <= 0.051) && (fabs(base_cmd.angular.z) <= 0.26))
          cmd_vel_pub_.publish(base_cmd);
      else 
          ROS_ERROR("COMMAND EXCEEDS MAXIMUM LIMITS (COMMAND %f %f %f)", base_cmd.linear.x, base_cmd.linear.y, base_cmd.angular.z);
      rate.sleep();
      //get the current transform
      try
      {
        listener_.lookupTransform("base_footprint", "odom_combined", 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
      //see how far we've traveled
      tf::Transform relative_transform = 
        start_transform.inverse() * current_transform;
      double dist_moved = relative_transform.getOrigin().length();

//      if(dist_moved > distance) done = true;
    

   }
    //if (done) return true;
    //return false;
  }

  void driveInOdom(const float targetPose[]){
  
      listener_.waitForTransform("base_link", "map", 
                               ros::Time(0), ros::Duration(10.0));

      tf::Stamped<tf::Pose> targetPoseMap;
      targetPoseMap.frame_id_ = "base_link";
      targetPoseMap.stamp_ = ros::Time();
      targetPoseMap.setOrigin(btVector3( targetPose[0], targetPose[1], 0));
      targetPoseMap.setRotation(btQuaternion(0,0, targetPose[2],  targetPose[3]));

      tf::Stamped<tf::Pose> targetPoseBase;

      targetPoseBase.stamp_ = ros::Time();

      listener_.transformPose("map", targetPoseMap, targetPoseBase);
      
      float pose[4];
      pose[0] = targetPoseBase.getOrigin().x();
      pose[1] = targetPoseBase.getOrigin().y();
      pose[2] = targetPoseBase.getRotation().z();
      pose[3] = targetPoseBase.getRotation().w();
     
      driveInMap(pose);
   }


};




// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class Gripper{
private:
  GripperClient* gripper_client_;  

public:
  //Action client initialization
  Gripper(){

    //Initialize the client for the Action interface to the gripper controller
    //and tell the action client that we want to spin a thread by default
    gripper_client_ = new GripperClient("r_gripper_controller/gripper_action", true);
    
    //wait for the gripper action server to come up 
    while(!gripper_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");
    }
  }

  ~Gripper(){
    delete gripper_client_;
  }

  //Open the gripper
  void open(){
    pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command.position = 0.08;
    open.command.max_effort = -1.0;  // Do not limit effort (negative)
    
    ROS_INFO("Sending open goal");
    gripper_client_->sendGoal(open);
    gripper_client_->waitForResult();
    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The gripper opened!");
    else
      ROS_INFO("The gripper failed to open.");
  }

  //Close the gripper
  void close(){
    pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
    squeeze.command.position = 0.0;
    squeeze.command.max_effort = 50.0;  // Close gently
    
    ROS_INFO("Sending squeeze goal");
    gripper_client_->sendGoal(squeeze);
    gripper_client_->waitForResult();
    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The gripper closed!");
    else
      ROS_INFO("The gripper failed to close.");
  }
};

// arm joint angles for seeing the marker with the hand camera
//high drawer
float highPoseA[7] = {-1.2658078481383668, 0.65194895948513354, -0.6651333173724735, -2.1146686310085161, -52.566807785671713, -1.1327560220881783, -29.66004589615245};
float highPoseB[7] = {-0.83999946139288795, 0.64805759302232857, -0.20651851395459886, -1.979887098710452, -52.732655383958971, -1.1379335835800284, -29.902173036506696};

//low drawer
float lowPoseA[7] = {-0.58879241080122102,1.2959701090793467,0.61289464879551958,-1.3123493485210054, -1.819574788110754,-1.1066071611083139,-0.425};
float lowPoseB[7] = {-0.56640762411483492,1.1399770708747323,0.49102498075440626,-0.9884972671560539,-1.8417302224128111,-0.88001270522963293,-0.425};

//middle drawer
float midPoseA[7] = {-1.136473525061469,0.87282630371086467,-0.20427354638542061,-2.1237891858256783,-2.2135217141215739,-1.2897797147276118,1.4621021956879214};
float midPoseB[7] = {-1.1291777427340541, 0.83112100662036859,-0.55432813235088196,-1.9421019430393511,-2.3539661773600271845,-0.76206336967963972,1.46};    

//tucking pose, expects left arm to be tucked already
float tuckPose[7] = {0.00059073198959080919, 1.1486903479544912, -1.4332329356842286, -1.4462622565507686, -51.274908792804851, -0.2154139507083308, -31.660673060172002};
//untucking poses, hand follows the base in a circular motion
float untuckPoseA[7] = {-1.3239253869056138, 1.2547723815274774, -2.1150616574089849, -1.6996110014719421, -51.308807185757871, -0.15602427477238412, -31.606852526008804};
float untuckPoseB[7] = {-2.2269442631579031, 0.96740342774164501, -2.3171087386350413, -1.7653369044400637, -51.229440982461696, -0.10677217721964216, -31.598672849030081};


float dishA[7] = {-2.1348350112742915, 0.64738083363749299, -1.595672374796923, -1.7539000182407651, -50.992267925964406, -0.56283267770359235, -37.575928292312355};
float dishB[7] = {-2.0394094946964008, -0.050442687051153672, -1.3657235537825063, -1.7921194860460163, -48.520404053480007, -0.07701207587152159, -37.641191672461751};
float dishC[7] =  {-0.80451542916409791, 0.098275187766477728, -1.4459009669674494, -0.39189716554453613, -48.893864846832827, -0.076881549111220027, -37.661771391668857};
float dishD[7] = {-1.4447203283947423, 1.1970786439702392, -0.76343082593721368, -0.96967707467619535, -50.016792498717102, -0.07775172751321624, -37.754706445001595};

bool tucked = false;

//l_arm_tucked = [0.06024, 1.248526, 1.789070, -1.683386, -1.7343417, -0.0962141, -0.0864407]
//r_arm_tucked = [-0.023593, 1.1072800, -1.5566882, -2.124408, -1.4175, -1.8417, 0.21436]

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

class RobotArm
{
private:
  // Action client for the joint trajectory action 
  // used to trigger the arm movement action
  TrajClient* traj_client_;


public:
  //! Initialize the action client and wait for action server to come up
  RobotArm() 
  {
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);

    // wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }

  //! Clean up the action client
  ~RobotArm()
  {
    delete traj_client_;
  }

  //! Sends the command to start a given trajectory
  void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal)
  {

    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    traj_client_->sendGoal(goal);
    traj_client_->waitForResult();

    if(traj_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
       ROS_INFO("traj action succ");
    else
       ROS_INFO("traj action failed");
  }

  //! Generates a simple trajectory with two waypoints, used as an example
  /*! Note that this trajectory contains two waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
  */
 pr2_controllers_msgs::JointTrajectoryGoal lookAtMarker(float *poseA, float *poseB)
  {

    tucked = (poseB == tuckPose);
    //our goal variable
    pr2_controllers_msgs::JointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
    goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
    goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
    goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
    goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

    // We will have two waypoints in this goal trajectory
    goal.trajectory.points.resize(2);

    // First trajectory point

    int ind = 0;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
      goal.trajectory.points[ind].positions[j] = poseA[j];
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 1 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(2.0);

    // Second trajectory point
   
    ind += 1;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
      goal.trajectory.points[ind].positions[j] = poseB[j];
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 2 seconds after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(4.0);

    //we are done; return the goal
    return goal;
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState()
  {
    return traj_client_->getState();
  }

  // rosrun tf_echo /base_link /r_wrist_roll_joint -> position
bool move_ik(float x, float y, float z, float ox, float oy, float oz, float ow) 
  {
  actionlib::SimpleActionClient<pr2_common_action_msgs::ArmMoveIKAction> ac("r_arm_ik", true);

  ac.waitForServer(); //will wait for infinite time  

  //ROS_INFO("server found.");

  pr2_common_action_msgs::ArmMoveIKGoal goal;
  goal.pose.header.frame_id = "base_link";
  goal.pose.header.stamp = ros::Time::now();

  goal.pose.pose.orientation.x = ox;
  goal.pose.pose.orientation.y = oy;
  goal.pose.pose.orientation.z = oz;
  goal.pose.pose.orientation.w = ow;
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  goal.pose.pose.position.z = z;

  goal.ik_timeout = ros::Duration(5.0);
  goal.ik_seed.name.push_back("r_shoulder_pan_joint");
  goal.ik_seed.name.push_back("r_shoulder_lift_joint");
  goal.ik_seed.name.push_back("r_upper_arm_roll_joint");
  goal.ik_seed.name.push_back("r_elbow_flex_joint");
  goal.ik_seed.name.push_back("r_forearm_roll_joint");
  goal.ik_seed.name.push_back("r_wrist_flex_joint");
  goal.ik_seed.name.push_back("r_wrist_roll_joint");

  // somewhat close to what we use for looking at the drawer
  goal.ik_seed.position.push_back(-1.1291777427340541);
  goal.ik_seed.position.push_back(0.83112100662036859);
  goal.ik_seed.position.push_back(-0.55432813235088196);
  goal.ik_seed.position.push_back(-1.9421019430393511);
  goal.ik_seed.position.push_back(-2.3539661773600271845);
  goal.ik_seed.position.push_back(-0.76206336967963972);
  goal.ik_seed.position.push_back( 1.46 );
				   
  goal.move_duration= ros::Duration(1.0);
  ac.sendGoal(goal);

  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
      return true;
    else 
      return false;
  }
  else
    ROS_INFO("Action did not finish before the time out.");
  return false;
  }
};

void getMarkerTransform(tf::StampedTransform &marker)
{
  tf::TransformListener listener;
  ros::NodeHandle node;
  ros::Rate rate(100.0);
  int count = 0;

  double lastTime = 0;
  tf::StampedTransform transSum; 
  int i = 0;
  double numSamples = 20;
  geometry_msgs::Quaternion rot[500];
  geometry_msgs::Vector3 trans[500];

  while (count < numSamples  && ros::ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/base_link", "/4x4_1",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      //ROS_ERROR("%s",ex.what());
    }

    double actTime = transform.stamp_.toSec();
                                  
    //ROS_INFO("I see a ARToolkit Marker. %f", transform.stamp_.toSec());
 
    if (actTime!=lastTime) {
      lastTime=actTime;
     // ROS_INFO("NEW MARKER POSITION %i" , count);
      marker = transform;
      trans[count].x = transform.getOrigin().x();
      trans[count].y = transform.getOrigin().y();
      trans[count].z = transform.getOrigin().z();
      rot[count].x = transform.getRotation().x();
      rot[count].y = transform.getRotation().y();
      rot[count].z = transform.getRotation().z();
      rot[count].w = transform.getRotation().w();
      count++;
      
      double meanZ = 0;
      for (i = 0; i < count; i++) 
      meanZ += trans[i].z;
      meanZ /= (double)count;
  
      double deviation=0;
      for (i = 0; i < count; i++)
         deviation+= (trans[i].z - meanZ) * (trans[i].z - meanZ);
      deviation/= (double)count;
		
      ROS_INFO("new marker %i STD DEVIATION IN Z %f", count, sqrtf(deviation));

      ROS_INFO("CURRENT MARKER x %f y %f z %f ",transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
      ROS_INFO("CURRENT MARKER OR x %f y %f z %f w %f", transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());


     btVector3 euler;
     //QuaternionToEuler(btQuaternion(rot[count].x,rot[count].y,rot[count].z,rot[count].w).normalize(), euler);
     QuaternionToEuler(transform.getRotation().normalize(), euler);
    
     ROS_INFO("EULER %f %f %f", euler.x(),euler.y(),euler.z());
    }

    rate.sleep();
  }

  //remove outliers
  double meanZ = 0;
  for (i = 0; i < numSamples; i++) 
	  meanZ += trans[i].z;
  meanZ /= numSamples;
  
  double deviation=0;
  for (i = 0; i < numSamples; i++)
	  deviation+= (trans[i].z - meanZ) * (trans[i].z - meanZ);
  deviation /= numSamples;
  
  ROS_INFO("STD DEVIATION IN Z %f", sqrtf(deviation));
  
  geometry_msgs::Quaternion rotMean;
  geometry_msgs::Vector3 transMean;

  transMean.x = 0;
  transMean.y = 0;
  transMean.z = 0;
  rotMean.x = 0;
  rotMean.y = 0;
  rotMean.z = 0;
  rotMean.w = 0; // would otherwise be 1
  
  double numGoodSamples = 0;
  for (i = 0; i < numSamples; i++) 
     if ((trans[i].z - meanZ) * (trans[i].z - meanZ) < deviation) {
		 transMean.x += trans[i].x;
		 transMean.y += trans[i].y;
		 transMean.z += trans[i].z;
		 rotMean.x += rot[i].x;
		 rotMean.y += rot[i].y;
		 rotMean.z += rot[i].z;
		 rotMean.w += rot[i].w;
		 numGoodSamples+=1;
	  }

  ROS_INFO("Number of accepted samples %f", numGoodSamples);
  
  transMean.x /= numGoodSamples; 
  transMean.y /= numGoodSamples;
  transMean.z /= numGoodSamples;
  rotMean.x /= numGoodSamples;
  rotMean.y /= numGoodSamples;   
  rotMean.z /= numGoodSamples;
  rotMean.w /= numGoodSamples;

  marker.setOrigin(btVector3(transMean.x,transMean.y,transMean.z));
  marker.setRotation(btQuaternion(rotMean.x,rotMean.y,rotMean.z,rotMean.w).normalize());

}



void getWristPose(tf::StampedTransform &marker)
{
  tf::TransformListener listener;
  ros::NodeHandle node;
  ros::Rate rate(10.0);

  tf::StampedTransform transform;
  bool transformOk = false;
  while (ros::ok() && (!transformOk)) {
  transformOk = true;
  try{
     listener.lookupTransform("/base_link", "/r_wrist_roll_link",ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
     //ROS_ERROR("%s",ex.what());
     transformOk = false;
    }   
   rate.sleep();
  }    
  marker = transform;
}


void openDrawer(int pos)
{

  Gripper gripper;  
  RobotArm arm;
  arm.startTrajectory(arm.lookAtMarker(untuckPoseA, untuckPoseB));
  gripper.open();
  // Start the trajectory    
  if (pos == 0) arm.startTrajectory(arm.lookAtMarker(lowPoseA,lowPoseB));
  if (pos == 1) arm.startTrajectory(arm.lookAtMarker(midPoseA,midPoseB));
  if (pos == 2) arm.startTrajectory(arm.lookAtMarker(highPoseA,highPoseB));
  // Wait for trajectory completion
  while(!arm.getState().isDone() && ros::ok())
  {
    usleep(500);
  }
  tf::StampedTransform aM; // averaged marker position 
  getMarkerTransform(aM);

  ROS_INFO("AVERAGED MARKER POS %f %f %f ROT %f %f %f %f", aM.getOrigin().x(),aM.getOrigin().y(),aM.getOrigin().z(),aM.getRotation().x(),aM.getRotation().y(),aM.getRotation().z(),aM.getRotation().w());

  double gripHorizontal = true;
  //double distToVert = (aM.getRotation().x() - 0.992) * (aM.getRotation().x() - 0.992) 
  //                  + (aM.getRotation().y() + 0.063) * (aM.getRotation().y() + 0.063)
  //                  + (aM.getRotation().z() - 0.073) * (aM.getRotation().z() - 0.073)
  //                  + (aM.getRotation().w() - 0.073) * (aM.getRotation().w() - 0.073);

  //double distToHoriz = (aM.getRotation().x() + 0.492809) * (aM.getRotation().x() + 0.492809) 
  //                  + (aM.getRotation().y() - 0.673529) * (aM.getRotation().y() - 0.673529)
  //                  + (aM.getRotation().z() - 0.456726) * (aM.getRotation().z() - 0.456726)
  //                  + (aM.getRotation().w() + 0.308057) * (aM.getRotation().w() + 0.308057);

  //   horiz 0.022361 -0.694367 0.018255 0.719042

  btVector3 euler;
  QuaternionToEuler(aM.getRotation().normalize(), euler);
   
  ROS_INFO("EULER %f %f %f", euler.x(),euler.y(),euler.z());

  //ROS_INFO("DISTANCE TO HORIZ %f DISTANCE TO VERT %f", distToHoriz, distToVert);

  //if (distToVert < distToHoriz)
  //  gripHorizontal = false;

  double gripOrientation[4];

  gripHorizontal = false;

  if (gripHorizontal) {
     gripOrientation[0] = 0;
     gripOrientation[1] = 0;
     gripOrientation[2] = 0;
     gripOrientation[3] = 1;
  } else {
     gripOrientation[0] = -.711;
     gripOrientation[1] = -.008;
     gripOrientation[2] = .005;
     gripOrientation[3] = .703;
  } 
  
  btQuaternion grip(aM.getRotation());
  grip *= btQuaternion(btVector3(0,-1,0), - M_PI / 2.0f);

 /*  gripOrientation[0] = grip.x();
  gripOrientation[1] = grip.y();
  gripOrientation[2] = grip.z();
  gripOrientation[3] = grip.w(); */

  arm.move_ik(aM.getOrigin().x() - .35,aM.getOrigin().y(),aM.getOrigin().z(), gripOrientation[0],gripOrientation[1],gripOrientation[2],gripOrientation[3]);
  arm.move_ik(aM.getOrigin().x() - .30,aM.getOrigin().y(),aM.getOrigin().z(), gripOrientation[0],gripOrientation[1],gripOrientation[2],gripOrientation[3]);
  arm.move_ik(aM.getOrigin().x() - .23,aM.getOrigin().y(),aM.getOrigin().z(), gripOrientation[0],gripOrientation[1],gripOrientation[2],gripOrientation[3]);
  arm.move_ik(aM.getOrigin().x() - .18,aM.getOrigin().y(),aM.getOrigin().z(), gripOrientation[0],gripOrientation[1],gripOrientation[2],gripOrientation[3]);

  gripper.close(); 

  tf::StampedTransform desiredPose = aM;  
  desiredPose.setOrigin(btVector3(aM.getOrigin().x() - .18 - .05,aM.getOrigin().y(),aM.getOrigin().z()));
  tf::StampedTransform startPose = aM;  
  startPose.setOrigin(btVector3(aM.getOrigin().x() - .18,aM.getOrigin().y(),aM.getOrigin().z()));

 //  arm.move_ik(aM.getOrigin().x() - .35,aM.getOrigin().y(),aM.getOrigin().z(), -0.711, -0.008, 0.005, 0.703);
  arm.move_ik(desiredPose.getOrigin().x(),desiredPose.getOrigin().y(),desiredPose.getOrigin().z(), gripOrientation[0],gripOrientation[1],gripOrientation[2],gripOrientation[3]);
  double dx, dy, dz;
  double lastK = 2;
  double maxK = 8;
  bool success = true;

  for (double k = 2; k <= maxK; k++) {    
    tf::StampedTransform actPose;
    getWristPose(actPose);
    ROS_INFO_STREAM("ACTUAL  POSE " << actPose.getOrigin().x() << " " << actPose.getOrigin().y() << " " << actPose.getOrigin().z()) ;
    ROS_INFO_STREAM("DESIRED POSE " << desiredPose.getOrigin().x() << " " << desiredPose.getOrigin().y() << " " << desiredPose.getOrigin().z()) ;
    ROS_INFO_STREAM("START   POSE " << startPose.getOrigin().x() << " " << startPose.getOrigin().y() << " " << startPose.getOrigin().z()) ;
    dx = actPose.getOrigin().x() - startPose.getOrigin().x();
    dy = actPose.getOrigin().y() - startPose.getOrigin().y();
    dz = actPose.getOrigin().z() - startPose.getOrigin().z();
    double length = sqrtf(dx * dx + dy * dy + dz * dz);
    dx *= 0.05 / length;
    dy *= 0.05 / length;
    dz *= 0.05 / length;
    ROS_INFO("D x %f y %f z %f", dx , dy ,dz);
    double x = startPose.getOrigin().x() + dx * k;
    double y = startPose.getOrigin().y() + dy * k;
    double z = startPose.getOrigin().z() + dz * k;
    ROS_INFO("D %f %f %f NEXT POSE %f %f %f" ,dx,dy,dz, x,y,z);
    if (success)
     success = arm.move_ik(x,y,z, gripOrientation[0],gripOrientation[1],gripOrientation[2],gripOrientation[3]);
    if (success)
     lastK = k; 
  }

  tf::StampedTransform lastPose = startPose;
  if (0)
  for (double k = 2; k <= maxK; k++) {    
    tf::StampedTransform actPose;
    getWristPose(actPose);
    ROS_INFO_STREAM("ACTUAL  POSE " << actPose.getOrigin().x() << " " << actPose.getOrigin().y() << " " << actPose.getOrigin().z()) ;
    ROS_INFO_STREAM("DESIRED POSE " << desiredPose.getOrigin().x() << " " << desiredPose.getOrigin().y() << " " << desiredPose.getOrigin().z()) ;
    ROS_INFO_STREAM("START   POSE " << startPose.getOrigin().x() << " " << startPose.getOrigin().y() << " " << startPose.getOrigin().z()) ;
    dx = actPose.getOrigin().x() - lastPose.getOrigin().x();
    dy = actPose.getOrigin().y() - lastPose.getOrigin().y();
    dz = actPose.getOrigin().z() - lastPose.getOrigin().z();
    lastPose = actPose;
    double length = sqrtf(dx * dx + dy * dy + dz * dz);
    dx *= 0.05 / length;
    dy *= 0.05 / length;
    dz *= 0.05 / length;
    ROS_INFO("D x %f y %f z %f", dx , dy ,dz);
    double x = actPose.getOrigin().x() + dx;
    double y = actPose.getOrigin().y() + dy;
    double z = actPose.getOrigin().z() + dz;
    ROS_INFO("D %f %f %f NEXT POSE %f %f %f" ,dx,dy,dz, x,y,z);
    if (success)
     success = arm.move_ik(x,y,z, gripOrientation[0],gripOrientation[1],gripOrientation[2],gripOrientation[3]);
    if (success)
     lastK = k; 
  }

  if (lastK < maxK) {
    double missing = maxK - lastK;
    double drx = dx * missing;
    double dry = dy * missing;
    float ps[4];
    ps[0] = drx;
    ps[1] = dry;
    ps[2] = 0;
    ps[3] = 1;

    ros::NodeHandle nh;
    RobotDriver driver(nh);
    driver.driveInOdom(ps);
  }

  //for (double retract = .05; retract < .45; retract += .05) {
  //   ROS_INFO("TRYING TO OPEN FOR %f cm", retract * 100);
  //   arm.move_ik(aM.getOrigin().x()  - retract - 0.18,aM.getOrigin().y(),aM.getOrigin().z(), -0.711, -0.008, 0.005, 0.703);
  // }
  gripper.open();
   // close it again   
  gripper.close();

  if (lastK < maxK) {
    double missing = maxK - lastK;
    double drx = dx * missing;
    double dry = dy * missing;
    float ps[4];
    ps[0] = -drx;
    ps[1] = -dry;
    ps[2] = 0;
    ps[3] = 1;

    ros::NodeHandle nh;
    RobotDriver driver(nh);
    driver.driveInOdom(ps);
  }

  for (double k = lastK; k > -1; --k) {    
    double x = startPose.getOrigin().x() + dx * k;
    double y = startPose.getOrigin().y() + dy * k;
    double z = startPose.getOrigin().z() + dz * k;
    ROS_INFO("D %f %f %f NEXT POSE %f %f %f" ,dx,dy,dz, x,y,z);
    arm.move_ik(x,y,z, gripOrientation[0],gripOrientation[1],gripOrientation[2],gripOrientation[3]);
  }

  gripper.open();

  arm.move_ik(aM.getOrigin().x()  - 0.2  - 0.18,aM.getOrigin().y(),aM.getOrigin().z(), gripOrientation[0],gripOrientation[1],gripOrientation[2],gripOrientation[3]);    
}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void moveBase(const float pose[])
{
  float x = pose[0];
  float y = pose[1];
  float oz = pose[2];
  float ow = pose[3];

  RobotArm arm;
  // Start the trajectory    

  if (!tucked)
    arm.startTrajectory(arm.lookAtMarker(untuckPoseA, tuckPose));

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.z = oz;
  goal.target_pose.pose.orientation.w = ow;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  bool finished_before_timeout = ac.waitForResult(ros::Duration(20.0));  

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved");
  else
    ROS_INFO("The base failed to movefor some reason");

  ros::NodeHandle nh;

  RobotDriver driver(nh);

  driver.driveInMap(pose);
}

// robot poses in map frame 
float poseA[4] = {-.942, .485, 1, 0}; // island left
float poseB[4] = {-1.013, 1.111, 1, 0}; //island middle
float poseC[4] = {-1.089, 2.039, 1, 0}; //island right
float poseD[4] = { .212, 2.89, 0, 1}; // below oven 
float poseE[4] = { .117, 1.957, 0, 1}; //sink left 
float poseF[4] = { .115, 1.17, 0, 1}; // sink dishwasher
float poseG[4] = { .232, .501, 0.05, 0.999}; // sink righ / trash
float poseH[4] = { .242, 2.263, 0, 1}; // right of heater

float poseD1[4] = { -0.996, 2.152, 0.921, -.390}; // %8
float poseD2[4] = { -.754, 0.239, 0.952, .306}; // %9
//float poseD3[4] = { .242, 2.263, 0, 1}; // right of heater
//float poseD4[4] = { .242, 2.263, 0, 1}; // right of heater

float pose0[4] = { -0.377, 1.382, 0, 1}; // somewhere in between island and sink

float *poses[20] = {poseA, poseB, poseC, poseD, poseE, poseF, poseG, poseH, poseD1, poseD2, pose0};

int drw_top = 2;
int drw_mid = 1;
int drw_low = 0;

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "robot_driver");

  printf("ias_drawer_executive \n");
  printf("usage: ias_drawer_executive <drawer_position> <drawer_height> [<drawer_position> <drawer_height> .. <drawer_position> <drawer_height>] \n");
  printf("drawer_position:-1 .. dont move base \n");
  printf("                 0 .. island left \n");
  printf("                 1 .. island middle \n");
  printf("                 2 .. island right \n");
  printf("                 3 .. under oven \n");
  printf("                 4 .. left of dishwasher \n");
  printf("                 5 .. dishwasher \n");
  printf("                 6 .. right of dishwasher \n");
  printf("drawer_height:-1 .. dont grasp/open drawer\n");
  printf("               0 .. low (~34cm) \n");
  printf("               1 .. middle (~64cm) \n");
  printf("               2 .. high (~78cm) \n\n");

  //RobotArm arm;
  // Start the trajectory    

  //arm.startTrajectory(arm.lookAtMarker(dishA, dishB));
  //arm.startTrajectory(arm.lookAtMarker(dishC, dishD));
   
  for (int i = 0; i < (argc - 1) / 2 ; ++i) {
    if (argc >= (i * 2 + 3)) {
       if (atoi(argv[i* 2 + 1])!= -1)
          moveBase(poses[atoi(argv[i* 2 + 1])]);
       if (atoi(argv[i* 2 + 2])!= -1)
          openDrawer(atoi(argv[i* 2 + 2]));
       }
   }

  
}

