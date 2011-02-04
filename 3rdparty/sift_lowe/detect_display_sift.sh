#!/bin/bash
if [ -z "$1" ]
then
        echo "WARNING: Pass input image"
elif [ -z "$2" ]
then
        echo "WARNING: Pass output image (e.g. results.pgm)" 
else
 echo first argument: $1
 echo second argument: $2
 `rospack find sift_lowe`/build/siftDemoV4/sift -display <$1 >$2
fi