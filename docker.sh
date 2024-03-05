xhost +local:docker
sudo docker run --name monitoring_ros2 -it \
  --privileged \
  --env ROS_MASTER_URI=http://localhost:11311 \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -p 3000:3000 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v /dev:/dev \
  -v /media/user/My\ Passport:/home/dataset \
  -w /home/Web/sensor_monitoring_ros2 \
  monitoring_ros2:latest