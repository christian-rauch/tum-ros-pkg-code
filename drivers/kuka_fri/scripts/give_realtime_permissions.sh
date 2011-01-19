#!/bin/sh
# This script sets the POSIX capabilities for creating realtime threads

/sbin/setcap CAP_SYS_NICE=ep $1
