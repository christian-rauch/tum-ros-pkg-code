#include <ros/ros.h>
#include <tf/tf.h>
//#include <actionlib/actionlib.h>
#include <actionlib/client/simple_action_client.h>

#include <find_base_pose/FindBasePoseAction.h>


void test_fridge(actionlib::SimpleActionClient<find_base_pose::FindBasePoseAction> &ac)
{

    float pts[][7] =
    {
        {0.478704, -1.0355, 1.18101, 0.767433, 0.639987, 0.022135, 0.0311955},
        {0.489086, -0.984206, 1.17956, 0.797904, 0.601535, 0.01726, 0.0347398},
        {0.494529, -0.937741, 1.1803, 0.830545, 0.555891, 0.0110758, 0.0325103},
        {0.504333, -0.909376, 1.18066, 0.849808, 0.526105, 0.00709967, 0.03147},
        {0.507886, -0.884252, 1.17954, 0.8814, 0.471274, 0.00699274, 0.0313926},
        {0.516993, -0.854729, 1.18006, 0.903026, 0.428457, 0.00859376, 0.0299171},
        {0.527833, -0.832331, 1.1803, 0.920176, 0.390256, 0.0125722, 0.0286066},
        {0.541463, -0.80644, 1.18, 0.931353, 0.362808, 0.0186723, 0.0245782},
        {0.571712, -0.760535, 1.17887, 0.936451, 0.349496, 0.024334, 0.017896},
        {0.608236, -0.715618, 1.17839, 0.944274, 0.327791, 0.0273483, 0.0123364},
        {0.647457, -0.676296, 1.17812, 0.954053, 0.298037, 0.0302956, 0.00623379},
        {0.690692, -0.638766, 1.17999, 0.964469, 0.262043, 0.0336022, 0.00195834},
        {0.734141, -0.609302, 1.18042, 0.974717, 0.220844, 0.0339708, -0.00102721},
        {0.781735, -0.583995, 1.17916, 0.983083, 0.180164, 0.0327274, -0.00426907},
        {0.828575, -0.564397, 1.17937, 0.990023, 0.137179, 0.0315954, -0.00617472},
        {0.870116, -0.550422, 1.17831, 0.995336, 0.0920069, 0.0283872, -0.00586025},
        {0.921693, -0.544899, 1.17853, 0.998734, 0.0415909, 0.0273629, -0.00714236},
        {0.971471, -0.549669, 1.17854, 0.998732, 0.0416648, 0.0273237, -0.00716123}
    };

    find_base_pose::FindBasePoseGoal goal;
    std_msgs::Int32 int32;
    int32.data = 0;

    geometry_msgs::PoseStamped ps;
    //ps.header.frame_id = "base_link";
    ps.header.frame_id = "map";
    ps.header.stamp = ros::Time::now();

    for (int k = 0; k < 17; ++k)
    {
        int adder = k * 8;
        int32.data = 0;
        ps.pose.position.x  = pts[k][0];
        ps.pose.position.y  = pts[k][1];
        ps.pose.position.z  = pts[k][2];
        ps.pose.orientation.x = pts[k][3];
        ps.pose.orientation.y = pts[k][4];
        ps.pose.orientation.z = pts[k][5];
        ps.pose.orientation.w = pts[k][6];
        goal.target_poses.push_back(ps);
        goal.arm.push_back(int32);
        ROS_INFO("pt");
    }

    ac.waitForServer(); //will wait for infinite time

    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(15.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
        find_base_pose::FindBasePoseResultConstPtr res = ac.getResult();
        for (int k = 0; k < res->base_poses.size(); ++k)
        {
            tf::Stamped<tf::Pose> respose;
            tf::poseStampedMsgToTF(res->base_poses[k], respose);
            ROS_INFO("Result: %f %f  %f", res->base_poses[k].pose.position.x, res->base_poses[k].pose.position.y,respose.getRotation().getAngle());
        }

    }
    else
        ROS_INFO("Action did not finish before the time out.");
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_find_base_pose");

    // create the action client
    actionlib::SimpleActionClient<find_base_pose::FindBasePoseAction> ac("find_base_pose_action",true);

    find_base_pose::FindBasePoseGoal goal;

    geometry_msgs::PoseStamped ps;
    //ps.header.frame_id = "base_link";
    ps.header.frame_id = "base_link";
    ps.header.stamp = ros::Time::now();

    float pos[] = {0.418, -0.219, 0.802};
    float rot[] = {0.695, 0.719, 0.004, 0.033};
    //pos[0] += 1.0;
    ps.pose.position.x = pos[0];
    ps.pose.position.y = pos[1];
    ps.pose.position.z = pos[2];
    ps.pose.orientation.x = rot[0];
    ps.pose.orientation.y = rot[1];
    ps.pose.orientation.z = rot[2];
    ps.pose.orientation.w = rot[3];
//   ps.pose.position.x = 0.221; ps.pose.position.y = -.411; ps.pose.position.z = 0.965;
//   ps.pose.orientation.x = -0.426; ps.pose.orientation.y = 0.091; ps.pose.orientation.x = 0.645; ps.pose.orientation.w = .628;

    std_msgs::Int32 int32;
    int32.data = 0;

    if (argc > 1)
    {
        for (int k = 0; k < atoi(argv[1]); ++k)
        {
            int adder = k * 8;
            int32.data = atoi(argv[2 + adder]);
            ps.pose.position.x  = atof(argv[3 + adder]);
            ps.pose.position.y  = atof(argv[4 + adder]);
            ps.pose.position.z  = atof(argv[5 + adder]);
            ps.pose.orientation.x = atof(argv[6 + adder]);
            ps.pose.orientation.y = atof(argv[7 + adder]);
            ps.pose.orientation.z = atof(argv[8 + adder]);
            ps.pose.orientation.w = atof(argv[9 + adder]);
            goal.target_poses.push_back(ps);
            goal.arm.push_back(int32);
            ROS_INFO("pt");
        }
    }


    test_fridge(ac);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(15.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
        find_base_pose::FindBasePoseResultConstPtr res = ac.getResult();
        for (int k = 0; k < res->base_poses.size(); ++k)
        {
            tf::Stamped<tf::Pose> respose;
            tf::poseStampedMsgToTF(res->base_poses[k], respose);
            ROS_INFO("Result: %f %f  %f", res->base_poses[k].pose.position.x, res->base_poses[k].pose.position.y,respose.getRotation().getAngle());
        }

    }
    else
        ROS_INFO("Action did not finish before the time out.");

    // shutdown the node
    ros::shutdown();

}

