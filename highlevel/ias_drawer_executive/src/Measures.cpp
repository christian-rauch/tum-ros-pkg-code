#include <ros/ros.h>

#include <iostream>
#include <fstream>
using namespace std;

#include <ias_drawer_executive/Approach.h>
#include <ias_drawer_executive/RobotDriver.h>
#include <ias_drawer_executive/Gripper.h>
#include <ias_drawer_executive/Pressure.h>
#include <ias_drawer_executive/RobotArm.h>
#include <ias_drawer_executive/Torso.h>
#include <ias_drawer_executive/Head.h>
#include <ias_drawer_executive/OperateHandleController.h>
#include <ias_drawer_executive/Keywords.h>
#include <ias_drawer_executive/ObjectLocalizer.h>


#include <ias_drawer_executive/Current.h>

#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>

#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <boost/thread.hpp>

#include <visualization_msgs/Marker.h>

#include <map>

#include <stdio.h>
#include <stdarg.h>



void test_func(Keywords &kp)
{
    ROS_INFO("string %s",kp.lookup_s("blub").c_str());
    ROS_INFO("double %f",kp.lookup_d("blub"));
    ROS_INFO("vec %f %f %f",kp.lookup_v("blic").x(),kp.lookup_v("blic").y(),kp.lookup_v("blic").z());

    ROS_INFO("double %f",(double)kp.lookup("blub"));
}

//std::map<std::string, double

int measures(int argc, char** argv)
{
    if (atoi(argv[1]) == -800)
    {
        ROS_INFO("Dip");
        PotLocalizer *pot = new PotLocalizer();

        tf::Stamped<tf::Pose> pose;

        ObjectLocalizer::localize("bla",&pose, 1);
        ObjectLocalizer::localize("Pot",&pose, 1);

        //Keywords kp = Keywords("bla", 1).contains()("blub", "blib")("tic", 5).contains();
        //kp.contains();
        //ROS_INFO("string lookup : %s", kp.lookup_s("blub").c_str());
        //ROS_INFO("double lookup : %f    ", kp.lookup_d("bla"));

        test_func(Keywords("bla",1) ("blub","blib") ("tic", 5) ("blub",7)("blic",tf::Vector3(0,0,1)) );

    }

    if (atoi(argv[1]) == -801)
    {
        tf::Stamped<tf::Pose> rTool = RobotArm::getInstance(0)->getToolPose("/base_link");

        for (double forward = 0; forward < 10; forward+=.25)
        {

            tf::Stamped<tf::Pose> act = rTool;
            act.getOrigin() += tf::Vector3(forward,0,0);

            {
                tf::Stamped<tf::Pose> result;
                std::vector<int> arm;
                std::vector<tf::Stamped<tf::Pose> > goal;
                arm.push_back(0);
                goal.push_back(act);
                //bool RobotArm::findBaseMovement(tf::Stamped<tf::Pose> &result, std::vector<int> arm, std::vector<tf::Stamped<tf::Pose> > goal, bool drive, bool reach)

                RobotArm::findBaseMovement(result, arm, goal, false, false);
            }

        }

    }


    if (atoi(argv[1]) == -802)
    {
        tf::Stamped<tf::Pose> rTool = RobotArm::getInstance(0)->getToolPose("/base_link");

        for (double forward = 0; forward > -10; forward-=.25)
        {

            tf::Stamped<tf::Pose> act = rTool;
            act.getOrigin() += tf::Vector3(forward,0,0);

            {
                tf::Stamped<tf::Pose> result;
                std::vector<int> arm;
                std::vector<tf::Stamped<tf::Pose> > goal;
                arm.push_back(0);
                goal.push_back(act);
                //bool RobotArm::findBaseMovement(tf::Stamped<tf::Pose> &result, std::vector<int> arm, std::vector<tf::Stamped<tf::Pose> > goal, bool drive, bool reach)

                RobotArm::findBaseMovement(result, arm, goal, false, false);
            }

        }

    }


    // -1.77 2.15 0.88

    return 0;

}

