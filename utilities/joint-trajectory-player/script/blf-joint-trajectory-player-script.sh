#!/usr/bin/env bash

scriptDirectory="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
pythonScriptRootPath=`realpath $scriptDirectory/../share/BipedalLocomotionFramework/python`

echo $pythonScriptRootPath

echo "Welcome to the JointPositionTrackingTest. I will first move the robot and then I will store the plot the data in a file called image.png"

echo "I'm going to move the robot. Watch out!"

blf-joint-trajectory-player --from blf-joint-trajectory-player-options.ini

echo "The trajectory is terminated."

