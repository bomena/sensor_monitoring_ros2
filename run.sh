#!/bin/bash

# Start roscore in the background
roscore &
ROSCORE_PID=$!
sleep 2  # Give roscore time to start

# npm start 프로세스 종료 함수 정의
cleanup() {
    echo "Stopping npm server..."
    if [ ! -z "$NPM_PID" ]; then
        kill $NPM_PID
    fi

    echo "Stopping roslaunch..."
    if [ ! -z "$ROSLAUNCH_PID" ]; then
        kill $ROSLAUNCH_PID
    fi

    echo "Stopping roscore..."
    if [ ! -z "$ROSCORE_PID" ]; then
        kill $ROSCORE_PID
    fi

    echo "Stopping other background processes..."
    pkill -P $$  # This will kill all child processes started by this script
}

# trap 명령으로 스크립트 종료 시 cleanup 함수 실행
trap cleanup EXIT

# roslaunch를 백그라운드에서 실행하고 PID 저장
roslaunch rosbridge_server rosbridge_websocket.launch &
ROSLAUNCH_PID=$!
sleep 2  # Give roslaunch time to start

# run.sh 스크립트 실행
# 현재 디렉토리를 저장하고 run.sh가 있는 디렉토리로 이동
CURRENT_DIR=$(pwd)
cd ./src/function

# run.sh 실행
./run.sh &
RUN_PID=$!

# 원래 디렉토리로 돌아가기
cd $CURRENT_DIR

# npm start를 백그라운드에서 실행하고 PID 저장
npm start &
NPM_PID=$!

# 스크립트가 종료될 때까지 대기
wait

