#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

class DriveBaseAction
{
protected:

  ros::NodeHandle nh_;
  MoveBaseActionServer as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  move_base_msgs::MoveBaseFeedback feedback_;
  move_base_msgs::MoveBaseResult result_;

  //! We will be publishing to the "cmd_vel" topic to issue commands
  ros::Publisher cmd_vel_pub_;
  //! We will be listening to TF transforms as well
  tf::TransformListener listener_;

public:

  DriveBaseAction(std::string name) :
    as_(nh_, name, boost::bind(&DriveBaseAction::executeCB, this, _1)),
    action_name_(name)
  {
      cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  }

  ~DriveBaseAction(void)
  {
  }

  //! Drive to a relative position
  bool driveOdom(double x, double y)
  {
    //wait for the listener to get the first message
    listener_.waitForTransform("base_footprint", "odom_combined",
                               ros::Time(0), ros::Duration(1.0));

    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the odometry to the base frame
    listener_.lookupTransform("base_footprint", "odom_combined",
                              ros::Time(0), start_transform);

    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    //the command will be to go forward at 0.25 m/s
    base_cmd.angular.z = 0;

    double distance = sqrtf((x * x) + (y * y));
    double scale = 0.25 / distance;

    ROS_INFO("SCALE %f", scale);

    base_cmd.linear.x = x * scale;
    base_cmd.linear.y = y * scale;

    ROS_INFO("CMD %f %f", base_cmd.linear.x, base_cmd.linear.y);

    ros::Rate rate(10.0);
    bool done = false;
    while (!done && nh_.ok())
    {
      //send the drive command
      cmd_vel_pub_.publish(base_cmd);
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

      ROS_INFO("DIST MOVED %f, DISTANCE TO MOVE %f", dist_moved, distance);

      if(dist_moved > distance) done = true;
    }
    if (done) return true;
    return false;
  }


  void executeCB(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
  {
    geometry_msgs::PoseStamped goal = move_base_goal->target_pose;

    ROS_INFO("target pose %f %f %f   %f %f %f %f", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w);

    driveOdom(goal.pose.position.x, goal.pose.position.y);

    // always succeed, error handling tbd
    as_.setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drive_base_action");

  DriveBaseAction drivebase(ros::this_node::getName());
  ros::spin();

  return 0;
}
