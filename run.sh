#!/bin/bash

cleanup() {
    echo "Stopping npm server..."
    if [ ! -z "$NPM_PID" ]; then
        kill $NPM_PID
    fi

    echo "Stopping roslaunch..."
    if [ ! -z "$ROSLAUNCH_PID" ]; then
        kill $ROSLAUNCH_PID
    fi

    echo "Stopping Python scripts..."
    if [ ! -z "$RUN_PID" ]; then
        kill $RUN_PID
    fi
}
trap cleanup EXIT

ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ROSLAUNCH_PID=$!
sleep 2

CURRENT_DIR=$(pwd)
cd ./src/function

./python.sh &
RUN_PID=$!

cd $CURRENT_DIR

npm start &
NPM_PID=$!

wait
