#!/bin/bash

echo "Welcome to the JointPositionTrackingTest. I will first move the robot and then I will store the plot the data in a file called image.png \n"

echo "I'm going to move the robot. Watch out! \n"

JointPositionTracking --from ../config/jointPositionTrackingOptions.ini

echo "The trajectory is terminated. \n"

echo "Plot data \n"

python3 ./plot_dataset.py --dataset `ls -t Dataset* | head -1`

echo "Done. Thanks for using the script. \n"
