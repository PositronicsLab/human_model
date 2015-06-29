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

export attempts="541,15,16,294,69,28,61,1,51,21,6,108,6,121,29,24,19,298,28,2,8,6,70,1,3,146,43,10,70,1,47,49,3,6,2,9,2,18,3,47,9,5,3,5,511,147,14,33,117,180,15,2,19,13,83,123,23,4,50,7,373,5,3,2,4,2,3,38,15,173,314,15,12,8,24,8,12,1,3,5,5,31,17,6,1,16,9,10,6,14,2,199,127,43,30,54,96,40,41,25"

for i in `seq 1 100`;
do
  echo "Executing scenario: $i"
  export i=$i
  # Launch gazebo
   gzserver models/human/world.generated.sdf
done 

export attempts=""
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

# Execute combined human locked arms
echo "Executing locked-arms scenario"
RESULTS_FOLDER=results/$SCENARIO_FOLDER/locked-arms
echo "Creating folder for locked-arms results: $RESULTS_FOLDER"
mkdir $RESULTS_FOLDER
export RESULTS_FOLDER=$RESULTS_FOLDER

for i in `seq 1 100`;
do
  echo "Executing scenario: $i"
  export export i=$i
  # Launch gazebo
  gzserver models/locked-arms/world.generated.sdf
done 
