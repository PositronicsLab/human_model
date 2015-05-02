#!/bin/bash

# Create a unique ID for the scenario
SCENARIO_FOLDER=`/bin/date +%s`
mkdir results/$SCENARIO_FOLDER

echo "Storing results in results/$SCENARIO_FOLDER"

# Execute human only
echo "Executing human only scenario"
RESULTS_FOLDER=results/$SCENARIO_FOLDER/human
echo "Creating folder for human results: $RESULTS_FOLDER"
mkdir $RESULTS_FOLDER
export RESULTS_FOLDER=$RESULTS_FOLDER

for i in `seq 1 100`;
do
  echo "Executing scenario: $i"
  export i=$i
  # Launch gazebo
  gzserver models/human/world.generated.sdf
done 

# Execute combined human pr2
echo "Executing normal-force scenario"
RESULTS_FOLDER=results/$SCENARIO_FOLDER/normal-force
echo "Creating folder for normal-force results: $RESULTS_FOLDER"
mkdir $RESULTS_FOLDER
export RESULTS_FOLDER=$RESULTS_FOLDER

for i in `seq 1 100`;
do
  echo "Executing scenario: $i"
  export i=$i
  # Launch gazebo
  gzserver models/combined-human-pr2/world.generated.sdf
done 

# Execute combined human pr2 zero force
echo "Executing zero-force scenario"
RESULTS_FOLDER=results/$SCENARIO_FOLDER/zero-force
echo "Creating folder for zero-force results: $RESULTS_FOLDER"
mkdir $RESULTS_FOLDER
export RESULTS_FOLDER=$RESULTS_FOLDER

for i in `seq 1 100`;
do
  echo "Executing scenario: $i"
  export export i=$i
  # Launch gazebo
 gzserver models/combined-human-pr2-zero-effort/world.generated.sdf
done
 
