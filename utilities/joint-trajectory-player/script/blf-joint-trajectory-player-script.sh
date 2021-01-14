#!/usr/bin/env bash

scriptDirectory="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

echo "Welcome to the JointPositionTrackingTest."

echo "I'm going to move the robot. Watch out!"

blf-joint-trajectory-player --from blf-joint-trajectory-player-options.ini

echo "The trajectory is terminated."

