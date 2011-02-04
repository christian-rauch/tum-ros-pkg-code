#!/bin/bash

# Check for proper number of command line args.
EXPECTED_ARGS=5
E_BADARGS=65

if [ -z "$1" ] || [ -z "$2" ] || [ -z "$3" ] || [ -z "$4" ] || [ -z "$5" ]
then
  echo "Usage: `basename $0`  book.pgm book.keys scene.pgm scene.keys result.pgm"
  exit $E_BADARGS
fi


echo Arguments: $1, $2, $3, $4, $5
`rospack find sift_lowe`/build/siftDemoV4/match -im1 $1 -k1 $2 -im2 $3 -k2 $4 > $5
