/* 
 * Copyright (c) 2010, Thomas Ruehr <ruehr@cs.tum.edu>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ias_drawer_executive/OperateHandleController.h>
#include <ias_drawer_executive/Torso.h>
#include <ias_drawer_executive/RobotDriver.h>
#include <ias_drawer_executive/Poses.h>
#include <ias_drawer_executive/RobotArm.h>
#include <ias_drawer_executive/Pressure.h>
#include <ias_drawer_executive/Gripper.h>
#include <ias_drawer_executive/Head.h>
#include <ias_drawer_executive/DemoScripts.h>
#include <actionlib/server/simple_action_server.h>

#include <ias_drawer_executive/OperateHandleAction.h>
#include <ias_drawer_executive/CloseHandleAction.h>
#include <ias_drawer_executive/PickBottleAction.h>
#include <ias_drawer_executive/PickPlateAction.h>
#include <ias_drawer_executive/GenericAction.h>




class OperateHandleAction
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<ias_drawer_executive::OperateHandleAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    ias_drawer_executive::OperateHandleFeedback feedback_;
    ias_drawer_executive::OperateHandleResult result_;

public:

    OperateHandleAction(std::string name) :
            as_(nh_, name, boost::bind(&OperateHandleAction::executeCB, this, _1)),
            action_name_(name)
    {
        ROS_INFO("%s up and running..", action_name_.c_str());
    }

    ~OperateHandleAction(void)
    {
    }

    void executeCB(const ias_drawer_executive::OperateHandleGoalConstPtr &goal)
    {
        // helper variables
        ros::Rate r(20);
        bool success = true;

        // push_back the seeds for the fibonacci sequence
        // publish info to the console for the user
        ROS_INFO("%s: Executing, creating OperateHandleAction  arm %i posIdx %i heightIdx %i", action_name_.c_str(), goal->arm, goal->positionIdx, goal->heightIdx);

        // start executing the action
        /*for(int i=1; i<=goal->order; i++)
        {
          // check that preempt has not been requested by the client
          if (as_.isPreemptRequested() || !ros::ok())
          {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            // set the action state to preempted
            as_.setPreempted();
            success = false;
            break;
          }
          //feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
          // publish the feedback
          //as_.publishFeedback(feedback_);
          // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
          r.sleep();
        }*/

        int arm  = goal->arm;
        int posIdx  = goal->positionIdx;
        int hgtIdx  = goal->heightIdx;

        if (posIdx >= 0)
        {
            Torso *torso = Torso::getInstance();
            boost::thread *t;
            if (posIdx==8)
                t = &boost::thread(&Torso::up, torso);
            else
                t = &boost::thread(&Torso::down, torso);
            ROS_INFO("TARGET POSE IN MAP %f %f %f %f",Poses::poses[posIdx][0],Poses::poses[posIdx][1],Poses::poses[posIdx][2],Poses::poses[posIdx][3]);
            RobotDriver::getInstance()->moveBase(Poses::poses[posIdx]);
            t->join();
        }
        if (hgtIdx >= 0)
        {
            tf::Stamped<tf::Pose> aM  = OperateHandleController::getHandlePoseFromMarker(arm,hgtIdx);
            int handle = OperateHandleController::operateHandle(arm,aM);
            result_.trajectoryHandle = handle;
        }
        else   // close the thing again, expects base to be in a good spot where good means it wont crush the door when driving from current to next closing position
        {
            OperateHandleController::close(hgtIdx);
            result_.trajectoryHandle = hgtIdx;
        }

        if (success)
        {
            //result_.sequence = feedback_.sequence;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            as_.setSucceeded(result_);
        }
    }
};


class CloseHandleAction
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<ias_drawer_executive::CloseHandleAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    ias_drawer_executive::CloseHandleFeedback feedback_;
    ias_drawer_executive::CloseHandleResult result_;

public:

    CloseHandleAction(std::string name) :
            as_(nh_, name, boost::bind(&CloseHandleAction::executeCB, this, _1)),
            action_name_(name)
    {
        ROS_INFO("%s up and running..", action_name_.c_str());
    }

    ~CloseHandleAction(void)
    {
    }

    void executeCB(const ias_drawer_executive::CloseHandleGoalConstPtr &goal)
    {
        // helper variables
        ros::Rate r(20);
        bool success = true;

        // push_back the seeds for the fibonacci sequence
        // publish info to the console for the user
        ROS_INFO("%s: Executing, creating CloseHandleAction  arm %i handle %i ", action_name_.c_str(), goal->arm, goal->handle);

        int arm  = goal->arm;
        int handle  = goal->handle;
        OperateHandleController::close(handle);

        result_.trajectoryHandle = handle;

        if (success)
        {
            //result_.sequence = feedback_.sequence;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            as_.setSucceeded(result_);
        }
    }
};



class PickBottleAction
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<ias_drawer_executive::PickBottleAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    ias_drawer_executive::PickBottleFeedback feedback_;
    ias_drawer_executive::PickBottleResult result_;

public:

    PickBottleAction(std::string name) :
            as_(nh_, name, boost::bind(&PickBottleAction::executeCB, this, _1)),
            action_name_(name)
    {
        ROS_INFO("%s up and running..", action_name_.c_str());
    }

    ~PickBottleAction(void)
    {
    }

    void executeCB(const ias_drawer_executive::PickBottleGoalConstPtr &goal)
    {
        // helper variables
        ros::Rate r(20);
        bool success = true;

        // push_back the seeds for the fibonacci sequence
        // publish info to the console for the user
        ROS_INFO("%s: Executing, creating CloseHandleAction  arm %i ", action_name_.c_str(), goal->arm);

        int arm_  = goal->arm;

        {

            pr2_controllers_msgs::JointTrajectoryGoal goalB = RobotArm::getInstance(1)->lookAtMarker(Poses::prepDishL1,Poses::prepDishL1);
            boost::thread t3(&RobotArm::startTrajectory, RobotArm::getInstance(1), goalB);

            Torso *torso = Torso::getInstance();
            boost::thread t2(&Torso::up, torso);

            //RobotDriver::getInstance()->moveBase();

            RobotArm::getInstance(0)->tucked = true;
            float p[] = { 0.255, -0.571, -0.025, 1.000};
            //float p[] = { 0.255, -0.571, -0.108, 0.994 };
            RobotDriver::getInstance()->moveBase(p);

            //t2.join();t3.join();

            RobotHead::getInstance()->lookAt("/map", 1.243111, -0.728864, 0.9);
            tf::Stamped<tf::Pose> bottle = OperateHandleController::getBottlePose();


            {
                //float fridgeLink = atof(argv[2]);
                float fridgeLink = .1;


                std::vector<int> arm;
                std::vector<tf::Stamped<tf::Pose> > goal;
                btVector3 result;

                tf::Stamped<tf::Pose> p0;
                p0.frame_id_="map";
                p0.stamp_=ros::Time();
                p0.setOrigin(btVector3(0.8, -0.455, 1.251));
                p0.setRotation(btQuaternion(0.005, -0.053, -0.029, 0.998));
                goal.push_back(p0);
                arm.push_back(arm_);

                tf::Stamped<tf::Pose> p1;
                p1.frame_id_="map";
                p1.stamp_=ros::Time();
                p1.setOrigin(btVector3(0.8, -0.655, 1.251));
                p1.setRotation(btQuaternion(0.005, -0.053, -0.029, 0.998));
                goal.push_back(p1);
                arm.push_back(arm_);

                tf::Stamped<tf::Pose> p2;
                p2.frame_id_="map";
                p2.stamp_=ros::Time();
                //p2.setOrigin(btVector3(1.165 - fridgeLink, -0.655, 1.151));
                p2.setOrigin(bottle.getOrigin() + btVector3(-.1,0,.03));
                p2.setRotation(btQuaternion(0.005, -0.053, -0.029, 0.998));
                goal.push_back(p2);
                arm.push_back(arm_);

                tf::Stamped<tf::Pose> p3;
                p3.frame_id_="map";
                p3.stamp_=ros::Time();
                //p3.setOrigin(btVector3(1.265000 - fridgeLink, -0.655000, 0.951000));
                p3.setOrigin(bottle.getOrigin() + btVector3(.02,0,.03));
                p3.setRotation(btQuaternion(0.005001, -0.053009, -0.029005, 0.998160));
                goal.push_back(p3);
                arm.push_back(arm_);

                RobotArm::findBaseMovement(result, arm, goal,true, true);
                //RobotArm::findBaseMovement(result, arm, goal,false);
            }

            Gripper::getInstance(arm_)->open();
            RobotArm::getInstance(arm_)->universal_move_toolframe_ik(0.8, -0.455, 1.251, 0.005, -0.053, -0.029, 0.998, "map");
            Gripper::getInstance(arm_)->updatePressureZero();
            RobotArm::getInstance(arm_)->universal_move_toolframe_ik(0.8, -0.655, 1.251, 0.005, -0.053, -0.029, 0.998, "map");
            //RobotArm::getInstance(1)->universal_move_toolframe_ik(1.065, -0.655, 1.151, 0.005, -0.053, -0.029, 0.998, "map");
            RobotArm::getInstance(arm_)->universal_move_toolframe_ik(bottle.getOrigin().x() - .1, bottle.getOrigin().y(), bottle.getOrigin().z() + .03, 0.005, -0.053, -0.029005, 0.998160, "map");
            //RobotArm::getInstance(1)->universal_move_toolframe_ik(1.265000, -0.655000, 0.951000, 0.005001, -0.053009, -0.029005, 0.998160, "map");
            //RobotArm::getInstance(1)->universal_move_toolframe_ik(1.2, -0.655000, 0.951000, 0.005001, -0.053009, -0.029005, 0.998160, "map");
            RobotArm::getInstance(arm_)->universal_move_toolframe_ik(bottle.getOrigin().x(), bottle.getOrigin().y(), bottle.getOrigin().z() + .03, 0.005, -0.053, -0.029005, 0.998160, "map");

            Gripper::getInstance(arm_)->closeCompliant();
            Gripper::getInstance(arm_)->close(0.5);
            //RobotArm::getInstance(1)->universal_move_toolframe_ik(1.065, -0.655, 1.151, 0.005, -0.053, -0.029, 0.998, "map");
            RobotArm::getInstance(arm_)->universal_move_toolframe_ik(bottle.getOrigin().x() - .1, bottle.getOrigin().y(), bottle.getOrigin().z() + .03, 0.005, -0.053, -0.029005, 0.998160, "map");
            RobotArm::getInstance(arm_)->universal_move_toolframe_ik(0.8, -0.655, 1.251, 0.005, -0.053, -0.029, 0.998, "map");
            RobotArm::getInstance(arm_)->universal_move_toolframe_ik(0.8, -0.455, 1.251, 0.005, -0.053, -0.029, 0.998, "map");
        }


        result_.ok = 1;

        if (success)
        {
            //result_.sequence = feedback_.sequence;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            as_.setSucceeded(result_);
        }
    }
};



class PickPlateAction
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<ias_drawer_executive::PickPlateAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    ias_drawer_executive::PickPlateFeedback feedback_;
    ias_drawer_executive::PickPlateResult result_;

public:

    PickPlateAction(std::string name) :
            as_(nh_, name, boost::bind(&PickPlateAction::executeCB, this, _1)),
            action_name_(name)
    {
        ROS_INFO("%s up and running..", action_name_.c_str());
    }

    ~PickPlateAction(void)
    {
    }

    void executeCB(const ias_drawer_executive::PickPlateGoalConstPtr &goal)
    {
        // helper variables
        ros::Rate r(20);
        bool success = true;

        // push_back the seeds for the fibonacci sequence
        // publish info to the console for the user
        ROS_INFO("%s: Executing, creating PickPlateAction ", action_name_.c_str());

        int arm  = goal->arm;
        //int handle  = goal->handle;
        // in drawer
        RobotHead::getInstance()->lookAt("/map", .4 ,1.13, .7);
        // at sink
        //RobotHead::getInstance()->lookAt("/map",0.913379, 0.824588, 0.7);

        // at sink
        //float target[4];       target[0] =-0.048;       target[1] = 1.029;       target[2] = 0;       target[3] = 1;
        float target[] = {-0.129, 1.017, 0.018, 1.000};
        ROS_INFO("POSE IN BASE %f %f %f", target[0],target[1], target[2]);


        Torso *torso = Torso::getInstance();
        boost::thread *t = &boost::thread(&Torso::up, torso);

        //RobotArm::getInstance(0)->tucked = true;

        pr2_controllers_msgs::JointTrajectoryGoal goalA = RobotArm::getInstance(0)->lookAtMarker(Poses::prepDishR1,Poses::prepDishR1);
        pr2_controllers_msgs::JointTrajectoryGoal goalB = RobotArm::getInstance(1)->lookAtMarker(Poses::prepDishL1,Poses::prepDishL1);
        boost::thread t2(&RobotArm::startTrajectory, RobotArm::getInstance(0), goalA);
        boost::thread t3(&RobotArm::startTrajectory, RobotArm::getInstance(1), goalB);
        //RobotArm::getInstance(1)->startTrajectory(RobotArm::getInstance(1)->lookAtMarker(Poses::prepDishL0,Poses::prepDishL1));
        t2.join();
        t3.join();

        RobotArm::getInstance(0)->tucked = true;
        RobotDriver::getInstance()->moveBase(target,false);

        system("rosrun dynamic_reconfigure dynparam set  /camera_synchronizer_node projector_mode 1");

        t->join();

        //in drawer
        RobotHead::getInstance()->lookAt("/map", .4 ,1.13, .7);
        // at sink
        //RobotHead::getInstance()->lookAt("/map",0.913379, 0.824588, 0.7);

        // 20 up 7 down from drawer to table


        //bin/ias_drawer_executi2 0 0.65, -0.4, 1.15 0.651, 0.295, -0.621, -0.322 ; bin/ias_drawer_executive -2 1 0.65, 0.4, 1.15 -0.295, 0.621, -0.322, 0.651
        //RobotArm::getInstance(0)->startTrajectory(RobotArm::getInstance(0)->lookAtMarker(Poses::prepDishR0,Poses::prepDishR1));
        //boost::thread t2(&RobotArm::startTrajectory, RobotArm::getInstance(0), RobotArm::getInstance(0)->lookAtMarker(Poses::prepDishR0,Poses::prepDishR1));
        //boost::thread t3(&RobotArm::startTrajectory, RobotArm::getInstance(1), RobotArm::getInstance(1)->lookAtMarker(Poses::prepDishL0,Poses::prepDishL1));

        //RobotArm::getInstance(0)->universal_move_toolframe_ik(0.65, -0.4, 1.15, 0.651, 0.295, -0.621, -0.322);
        //RobotArm::getInstance(1)->universal_move_toolframe_ik(0.65, 0.4, 1.15, -0.295, 0.621, -0.322, 0.651);
        //OperateHandleController::getPlate(atoi(argv[2]));
        OperateHandleController::getPlate(0);

        if (success)
        {
            //result_.sequence = feedback_.sequence;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            as_.setSucceeded(result_);
        }
    }
};

class GenericAction
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<ias_drawer_executive::GenericAction> as_;
    std::string action_name_;
    ias_drawer_executive::GenericFeedback feedback_;
    ias_drawer_executive::GenericResult result_;

    int (*fn)(int);

public:

    GenericAction(std::string name, int (*pt2Func)(int)) :
            as_(nh_, name, boost::bind(&GenericAction::executeCB, this, _1)),
            action_name_(name)
    {
        ROS_INFO("%s up and running..", action_name_.c_str());
        fn = pt2Func;
    }

    ~GenericAction(void)
    {
    }

    void executeCB(const ias_drawer_executive::GenericGoalConstPtr &goal)
    {

        bool success=true;
        result_.handle = fn(goal->arm);
        if (success)
        {
            //result_.sequence = feedback_.sequence;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            as_.setSucceeded(result_);
        }
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "operate_handle_action");

    OperateHandleAction opHandle("operate_handle_action");
    CloseHandleAction clHandle("close_handle_action");
    PickBottleAction puBottle("pick_bottle_action");
    PickPlateAction pPlate("pick_plate_action");


    GenericAction openFridge("open_fridge", &DemoScripts::openFridge);

    GenericAction takeBottle("take_bottle", &DemoScripts::takeBottle);

    GenericAction closeFridge("close_fridge", &DemoScripts::closeFridge);

    GenericAction serveBottle("serve_bottle", &DemoScripts::serveBottle);

    GenericAction openDrawer("open_drawer", &DemoScripts::openDrawer);

    GenericAction takePlate("take_drawer", &DemoScripts::takePlate);

    GenericAction servePlateToIsland("serve_plate_to_island", &DemoScripts::servePlateToIsland);

    GenericAction takeSilverware("take_silverware", &DemoScripts::takeSilverware);

    GenericAction serveToTable("serve_to_table", &DemoScripts::serveToTable);

    GenericAction takePlateFromIsland("take_plate_from_island", &DemoScripts::takePlateFromIsland);

    ros::spin();

    return 0;
}
