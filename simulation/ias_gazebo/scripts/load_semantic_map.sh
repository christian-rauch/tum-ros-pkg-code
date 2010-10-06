#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: $0 <parameter name>"
    exit 1
fi

rosrun ias_gazebo_tools semantic_map2param /kitchen_description
rosrun gazebo_tools gazebo_model -p /kitchen_description -x -2.5 -y -2.7 -z -0.165 spawn semantic_map

