#!/bin/bash
CUR_DIR=`pwd`
./laser_sweep_client/sweep_laser_scan_client -i 1 -s -92 -e -109 -j 4 -l 1 -f ${CUR_DIR}/demo.log
