#!/bin/bash
CUR_DIR=`pwd`
./laser_sweep_client/sweep_laser_scan_client -i 1 -s 0 -e -90 -j 4 -f ${CUR_DIR}/demo.log
