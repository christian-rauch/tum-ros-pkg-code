#!/bin/bash
if [ -z "$1" ]
then
        echo "WARNING: Pass input image"
elif [ -z "$2" ]
then
        echo "WARNING: Pass output keypoints file (e.g. book.key)" 
else
 echo first argument: $1
 echo second argument: $2
 `rospack find sift_lowe`/build/siftDemoV4/sift <$1 >$2
fi