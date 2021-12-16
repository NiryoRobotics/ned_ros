#!/bin/bash

# echo "Start wait for $1 seconds"
sleep $1
# echo "End wait for $1 seconds"

shift # The sleep time is dropped
#     echo "now running 'roslaunch $@'"
roslaunch --no-summary "$@"
