#!/bin/bash

for i in `seq 1 7`;
do
  echo "Executing scenario: $i"
  # Launch the listener
  ./Debug/listener &
  # Launch gazebo
  gazebo world.sdf &
  # Wait for 15 seconds to complete
  echo "Terminating gzserver"
  sleep 15
  # Kill gazebo
  killall gzclient
  killall gzserver
  sleep 1
done    
        

