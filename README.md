# FUNCTION

1. rosbag record 용량 및 기록 & 상태 확인(per 10s)
2. sensor sync 확인(per 5s) : 최대 차이 시간 출력
3. 각각의 sensor 연결 상태 확인
4. gps data를 통해 지도에서 실시간 현재 위치 확인 가능
5. cam과 LiDAR 확인 가능

# DOCKER
```
$ docker pull bmn3626/sensor_monitoring:1.0
```

# docker.sh
```
xhost +local:docker
sudo docker run --name monitoring -it \
	--privileged \
	--env ROS_MASTER_URI=http://localhost:11311 \
	--net=host \
	-e DISPLAY=$DISPLAY \
	-p 3000:3000 \
	-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
	-v /dev:/dev \
	-v <PATH>:/home/dataset \
	-w /home/Web/sensor_monitoring \
	bmn3626/sensor_monitoring:1.0
```
# Manual of Monitoring
1. docker.sh에서 <PATH>에 rosbag를 기록할 경로를 작성한다.
	( -v /home/media/user/SSD:/home/dataset )

2. sensorConfig.json파일에서 topic과 messageType을 알맞게 변경한다. 형식에 맞춰서 추가 및 삭제는 자유롭다.
	이때, 되도록 카메라와 라이다의            이름(id)은 바꾸지 않도록 한다. 만약 바꾼다면 그에 맞게 Sensor.jsx 파일에서 해당하는 이름을 전부 변경해야 한다.

3. sensor_monitoring/src/function 에 있는 data_sync.py, lidar2img.py 에서 Subcriber를 알맞게 수정해준다.


# How to build the dockerfile
```
$ ./docker.sh
$ exit
```

# How to start docker
```
$ sudo docker start monitoring
$ sudo docker exec -it monitoring /bin/bash
```
# How to run it.
```
$ ./run.sh
```

Runs the app in the development mode.\
# Open [http://localhost:3000](http://localhost:3000) to view it in your browser.
