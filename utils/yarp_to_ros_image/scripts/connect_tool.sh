#!/bin/bash
YARP_BASE_DIR=`rospack find yarp2`
sleep 1
${YARP_BASE_DIR}/yarp2/bin/yarp connect $1 $2