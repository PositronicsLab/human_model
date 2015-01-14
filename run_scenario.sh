#!/bin/bash

for i in `seq 1 100`;
do
  echo "Executing scenario: $i"
  # Launch the listener
  ./Debug/listener &
  # Launch gazebo
  gazebo world.sdf &
  # Wait for 15 seconds to complete
  sleep 10
  # Kill gazebo
  killall gzclient
  killall gzserver
done    
        

