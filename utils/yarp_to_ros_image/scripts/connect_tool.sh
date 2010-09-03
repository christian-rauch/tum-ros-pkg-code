#!/bin/bash
rosrun yarp2 yarp wait $1
rosrun yarp2 yarp wait $2
rosrun yarp2 yarp connect $1 $2
