#!/bin/bash

function usage() {
    clear
    echo 
    echo Structured Light Demo - Pattern Control
    echo ---------------------
    echo
    echo Use keys:
    echo [1] - pattern on
    echo [0] - pattern off
    echo [q] - pattern off and exit
    echo
    echo ---------------------
    echo -n "Choose > "
}

#roscd projected_light
#cd ${ROS_ROOT}/../tumros-internal/demos/projected_light/
#roslaunch ../launch/pattern.launch &
#PATTERN_PID=$!

#sleep 5

# give the user the pattern control
while [ TRUE ]; do
    usage
    read user_choice
    if   [ $user_choice = '1' ]; then
	rosparam set /light_projector/pattern_on 1
    elif [ $user_choice = '0' ]; then
	rosparam set /light_projector/pattern_on 0
    elif [ $user_choice = 'q' ]; then
	kill $PATTERN_PID
	break
    else
	echo invalid choice
    fi
done

exit 0