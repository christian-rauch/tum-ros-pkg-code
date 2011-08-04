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

// roslaunch arm_ik.launch
#include <ros/ros.h>

#include <ias_drawer_executive/RobotDriver.h>

int main(int argc, char** argv)
{
    // Init the ROS node
    ros::init(argc, argv, "driver");

    printf("ias_drawer_executive %i\n", argc);

    ros::NodeHandle node_handle;

    //! drive in map x y oz ow
    if (atoi(argv[1]) == -4)
    {

      //RobotArm::getInstance(0)->tucked = true;
        for (int i = 0; i < (argc - 2 ) / 4; ++i)
        {
            float p[4];
            p[0] = atof(argv[2 + i * 4]);
            p[1] = atof(argv[3 + i * 4]);
            p[2] = atof(argv[4 + i * 4]);
            p[3] = atof(argv[5 + i * 4]);
            ROS_INFO("going to %f %f %f %f", p[0], p[1], p[2], p[3]);
            RobotDriver::getInstance()->moveBase(p);
	    sleep(atof(argv[argc - 1]));
        }
    }
}

