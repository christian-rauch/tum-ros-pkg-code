#!/bin/bash
for i in `find ./ -name "build"`
do
  echo $i
  rm -rf $i
done
