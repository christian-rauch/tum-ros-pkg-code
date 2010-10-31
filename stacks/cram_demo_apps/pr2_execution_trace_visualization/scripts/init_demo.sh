#!/bin/bash

execution_trace="$(rospack find pr2_execution_trace_visualization)/downloads/execution_traces/20101012004123-PANCAKE-DEMO.ek"

rosservice call --wait /execution_trace_server/simple_query init "(with-execution-trace \"$execution_trace\")"
rosservice call --wait /execution_trace_server/next_solution init
rosservice call --wait /execution_trace_server/finish init

tf_bag="$(rospack find pr2_execution_trace_visualization)/downloads/execution_traces/init_execution_trace_demo.bag"
rosbag play $tf_bag
