#!/usr/bin/env bash

input_port_audio=$1

# Function to handle SIGINT signal
function handle_sigint {
    echo "Received SIGINT signal. Forwarding to background programs..."
    # Forward SIGINT to the background program
    kill -SIGINT $bg_pid1 $bg_pid2
}

function handle_exit {

    echo "Loking fro the audio file. It may take a while."
    until [ -f audio_out.wav ]
    do
        sleep 5
    done

    echo "Renaming the audio"

    # get the latest matfile name
    latest_file=$(find . -maxdepth 1 -type f -name "*.mat" -printf "%T@ %f\n" | sort -nr | head -n 1 | awk '{print $2}' | sed 's/\.mat$//')
    mv audio_out.wav $latest_file.wav

    echo "Closing"

    exit
}

# launch the audio device recorder
yarpdev --device AudioPlayerWrapper --name /YarpRobotLogger/audio --subdevice audioToFileDevice --start --file_name audio_out.wav --save_mode overwrite_file &
bg_pid1=$!
yarp wait /YarpRobotLogger/audio/audio:i
yarp connect $input_port_audio /YarpRobotLogger/audio/audio:i fast_tcp

# launch the logger device
yarprobotinterface --config launch-yarp-robot-logger.xml &
bg_pid2=$!

# register the signal handlers
trap handle_sigint SIGINT
trap handle_exit EXIT

# wait for closing the application
wait "$bg_pid1"
bg_exit_status1=$?

wait "$bg_pid2"
bg_exit_status2=$?
