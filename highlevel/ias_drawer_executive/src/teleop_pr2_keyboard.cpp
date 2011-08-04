/*
 * teleop_pr2_keyboard
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *     * Neither the name of the <ORGANIZATION> nor the names of its
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

// Author: Kevin Watts

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <ias_drawer_executive/RobotArm.h>
#include <ias_drawer_executive/Gripper.h>

#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_R 0x72

#define KEYCODE_I 0x69
#define KEYCODE_K 0x6B
#define KEYCODE_J 0x6A
#define KEYCODE_L 0x6C
#define KEYCODE_N 0x6E
#define KEYCODE_M 0x6D

#define KEYCODE_I_CAP 0x49
#define KEYCODE_K_CAP 0x4B
#define KEYCODE_J_CAP 0x4A
#define KEYCODE_L_CAP 0x4C
#define KEYCODE_N_CAP 0x4E
#define KEYCODE_M_CAP 0x4D

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57
#define KEYCODE_Q_CAP 0x51
#define KEYCODE_E_CAP 0x45

#define KEYCODE_BRACK_OP 0x5B
#define KEYCODE_BRACK_CL 0x5D

class TeleopPR2Keyboard
{
private:
    double walk_vel, run_vel, yaw_rate, yaw_rate_run;
    geometry_msgs::Twist cmd;

    ros::NodeHandle n_;
    ros::Publisher vel_pub_;

    RobotArm *arm_;
    Gripper *gripper_;

public:
    void init(int arm_no)
    {
        cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;

        arm_ = RobotArm::getInstance(arm_no);
        gripper_ = Gripper::getInstance(arm_no);

        //vel_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        ros::NodeHandle n_private("~");
        n_private.param("walk_vel", walk_vel, 0.5);
        n_private.param("run_vel", run_vel, 1.0);
        n_private.param("yaw_rate", yaw_rate, 1.0);
        n_private.param("yaw_run_rate", yaw_rate_run, 1.5);

    }

    ~TeleopPR2Keyboard()   { }
    void keyboardLoop();

};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    exit(0);
}

int main(int argc, char** argv)
{

    char name[50]  = "pr2_base_keyboard";

    int no_arm = 0;

    if (argc > 1)
    {
        no_arm = atoi(argv[1]);
        sprintf(name,"pr2_base_keyboard_%i",no_arm);
    }

    ros::init(argc, argv, name);

    ROS_INFO("INITIATLIZED NODE WITH NAME: %s", name);

    TeleopPR2Keyboard tpk;
    tpk.init(no_arm);

    signal(SIGINT,quit);

    tpk.keyboardLoop();

    return(0);
}


void TeleopPR2Keyboard::keyboardLoop()
{
    char c;
    bool dirty=false;
    bool grip_dirty = false;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use 'WASD' to translate");
    puts("Use 'QE' to yaw");
    puts("Press 'Shift' to run");

    //RobotArm *arm = RobotArm::getInstance(1);
    arm_->evil_switch = true;

    tf::Stamped<tf::Pose> toolPose = arm_->getToolPose();//const char frame[] = "base_link");
    float grip_open = gripper_->getAmountOpen();

    for (;;)
    {
        // get the next event from the keyboard
        if (read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        ROS_INFO("read: %X\n", c);

        cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;

        btVector3 dist(0,0,0);

        switch (c)
        {
            // Walking
        case KEYCODE_W:
            //cmd.linear.x = walk_vel;
            dirty = true;
            dist += btVector3(0.01,0,0);
            break;
        case KEYCODE_S:
            //cmd.linear.x = - walk_vel;
            dist += btVector3(-0.01,0,0);
            dirty = true;
            break;
        case KEYCODE_A:
            //cmd.linear.y = walk_vel;
            dist += btVector3(0,0.01,0);
            dirty = true;
            break;
        case KEYCODE_D:
            //cmd.linear.y = - walk_vel;
            dist += btVector3(0,-0.01,0);
            dirty = true;
            break;
        case KEYCODE_Q:
            //cmd.angular.z = yaw_rate;
            dist += btVector3(0,0,0.01);
            dirty = true;
            break;
        case KEYCODE_E:
            //cmd.angular.z = - yaw_rate;
            dist += btVector3(0,0,-0.01);
            dirty = true;
            break;


        case KEYCODE_I:
            toolPose = arm_->rotateAroundBaseAxis(toolPose,0.1,0,0);
            dirty = true;
            break;
        case KEYCODE_K:
            toolPose = arm_->rotateAroundBaseAxis(toolPose,-0.1,0,0);
            dirty = true;
            break;
        case KEYCODE_J:
            toolPose = arm_->rotateAroundBaseAxis(toolPose,0,0,0.1);
            dirty = true;
            break;
        case KEYCODE_L:
            toolPose = arm_->rotateAroundBaseAxis(toolPose,0,0,-0.1);
            dirty = true;
            break;
        case KEYCODE_N:
            toolPose = arm_->rotateAroundBaseAxis(toolPose,0,0.1,0);
            dirty = true;
            break;
        case KEYCODE_M:
            toolPose = arm_->rotateAroundBaseAxis(toolPose,0,-0.1,0);
            dirty = true;
            break;

        case KEYCODE_I_CAP:
            toolPose = arm_->rotateAroundToolframeAxis(toolPose,0.1,0,0);
            dirty = true;
            break;
        case KEYCODE_K_CAP:
            toolPose = arm_->rotateAroundToolframeAxis(toolPose,-0.1,0,0);
            dirty = true;
            break;
        case KEYCODE_J_CAP:
            toolPose = arm_->rotateAroundToolframeAxis(toolPose,0,0,0.1);
            dirty = true;
            break;
        case KEYCODE_L_CAP:
            toolPose = arm_->rotateAroundToolframeAxis(toolPose,0,0,-0.1);
            dirty = true;
            break;
        case KEYCODE_N_CAP:
            toolPose = arm_->rotateAroundToolframeAxis(toolPose,0,0.1,0);
            dirty = true;
            break;
        case KEYCODE_M_CAP:
            toolPose = arm_->rotateAroundToolframeAxis(toolPose,0,-0.1,0);
            dirty = true;
            break;


        case KEYCODE_BRACK_OP:
            grip_open += 0.0025;
            grip_dirty = true;
            break;
        case KEYCODE_BRACK_CL:
            grip_open -= 0.0025;
            grip_dirty = true;
            break;


        case KEYCODE_R:
            toolPose = arm_->getToolPose();//const char frame[] = "base_link");
            break;



            /*

            // Running
            case KEYCODE_W_CAP:
            cmd.linear.x = run_vel;
            dirty = true;
            break;
            case KEYCODE_S_CAP:
            cmd.linear.x = - run_vel;
            dirty = true;
            break;
            case KEYCODE_A_CAP:
            cmd.linear.y = run_vel;
            dirty = true;
            break;
            case KEYCODE_D_CAP:
            cmd.linear.y = - run_vel;
            dirty = true;
            break;
            case KEYCODE_Q_CAP:
            cmd.angular.z = yaw_rate_run;
            dirty = true;
            break;
            case KEYCODE_E_CAP:
            cmd.angular.z = - yaw_rate_run;
            dirty = true;
            break;*/
        }

        if (dirty)
        {
            //toolPose = arm_->getToolPose();//const char frame[] = "base_link");
            dist *= 5;
            toolPose.setOrigin(toolPose.getOrigin() + dist);
            //dist = btVector3(0,0,0);
            arm_->move_toolframe_ik_pose(toolPose);
            dirty = false;
        }

        if (grip_dirty)
        {
            //gripper->clos
            gripper_->closeHard(grip_open);
            for (int m = 0 ; m < 5; ++m)
            {
                float am_open = gripper_->getAmountOpen();
                ROS_INFO("GRIPPER OPENING :%f (getAmountOpen reports %f)", grip_open, am_open);
            }
            grip_dirty = false;
        }


        //if (dirty == true)
        //{
        //vel_pub_.publish(cmd);
        //}


    }
}
