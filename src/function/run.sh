#!/bin/bash

# record.py를 백그라운드에서 실행
python3 rosbag.py &

# data_sync.py를 백그라운드에서 실행
python3 data_sync.py &

python3 lidar2img.py &
# 두 프로세스가 종료될 때까지 대기
wait

