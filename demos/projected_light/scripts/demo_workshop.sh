#################################################################
##Use if camera and projector are driven by 2 different computers
#################################################################
#!/bin/bash

echo "++++++++++++++++++++++++++++++++++++++++++++++++++++++"
echo "+ first do"
echo "+   export ROS_MASTER_URI=http://192.168.150.108:11311"
echo "++++++++++++++++++++++++++++++++++++++++++++++++++++++"
echo

xterm -title "camera" -e "bash start_camera.sh" &

sleep 10

# do the ros-stuff
xterm -title "Pattern Control" -e bash control_pattern.sh &

rosrun rviz rviz

echo


exit 0