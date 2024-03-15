xhost +local:docker
sudo docker run --name monitoring_ros2 -it \
  --privileged \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -p 3000:3000 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v /dev:/dev \
  -v /media/user/My\ Passport:/home/dataset \
  -w /home/Web/sensor_monitoring_ros2 \
  bmn3626/sensor_monitoring_ros2:latest
