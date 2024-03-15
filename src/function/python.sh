#!/bin/bash

cleanup() {
    echo "Stopping ros2 bag..."
    if [ ! -z "$ROSBAG_PID" ]; then
        kill $ROSBAG_PID
    fi

    echo "Stopping data sync..."
    if [ ! -z "$DATASYNC_PID" ]; then
        kill $DATASYNC_PID
    fi

    echo "Stopping lidar to img..."
    if [ ! -z "$LIDARIMG_PID" ]; then
        kill $LIDARIMG_PID
    fi
}
trap cleanup EXIT

python3 rosbag.py &
ROSBAG_PID=$!

python3 data_sync.py &
DATASYNC_PID=$!

python3 lidar2img.py &
LIDARIMG_PID=$!

wait

