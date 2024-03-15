#!/bin/bash

cleanup() {
    echo "Stopping record..."
    if [ ! -z "$RECORD_PID" ]; then
        kill $RECORD_PID
    fi
}
trap clenup EXIT

cd /home/dataset

######MODIFY#####
ros2 bag record -a &
RECORD_PID=$!

wait
