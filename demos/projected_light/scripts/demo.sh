#!/bin/bash

# start xserver and remember its PID
#X &
#X_PID=$!

# wait for xserver :-)
#sleep 5

xterm -title "Pattern Control" -e bash control_pattern.sh &

# do the ros-stuff
roslaunch ../launch/demo.launch

# shutdown X
#kill ${X_PID}

echo


exit 0