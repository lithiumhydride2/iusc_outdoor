#!/bin/sh
rosnode kill --all
killall gzserver
killall gzclient
echo "killall gazebo"