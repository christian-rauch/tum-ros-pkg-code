#!/bin/bash
#gnuplots the values of channel CHANNEL
#- END_CHANNEL: is the name of the channel preceeding channel CHANNEL
#- TOPIC: topic the data is being published over  
CHANNEL=curvature
END_CHANNEL=f1
TOPIC='/radius_estimation_node/cloud_radius'
rostopic echo -p ${TOPIC}   | grep -v "^%" | head -n 1 | sed -e "s/^.*${CHANNEL},/plot \'-\' with lines\, \n/" -e"s/,${END_CHANNEL}.*$//" -e "s/\,/\n/g" | gnuplot -persist
