#!/bin/bash

function usage() {
    #clear
    echo 
    echo Structured Light Demo - Camera Control
    echo ---------------------
    echo
    echo Use keys:
    echo [q] - stop camera and quit
    echo
    echo ---------------------
    echo -n "Choose > "
}

#roscd projected_light
cd ${ROS_ROOT}/../tumros-internal/demos/projected_light/
roslaunch ../launch/videre.launch &
CAMERA_PID=$!

sleep 5

# give the user the camera control
while [ TRUE ]; do
    usage
    read user_choice
    if [ $user_choice = 'q' ]; then
	kill $CAMERA_PID
	break
    else
	echo "invalid choice"
    fi
done

exit 0