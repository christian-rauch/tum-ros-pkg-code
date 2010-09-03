#!/bin/bash

YARP_BASE_DIR=`rospack find yarp2`

export PATH=$YARP_BASE_DIR/yarp2/bin:$PATH
export LD_LIBRARY_PATH=$YARP_BASE_DIR/yarp2/lib:$LD_LIBRARY_PATH

[ -n "$1" ] && $@

