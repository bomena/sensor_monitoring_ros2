FROM ubuntu:jammy
FROM osrf/ros:humble

RUN \
  apt-get -qq update && \
  apt-get -qq upgrade --yes && \
  apt-get -qq install vim git curl --yes

RUN curl -sL https://deb.nodesource.com/setup_18.x | bash -
RUN apt-get -qq install nodejs --yes
RUN \
  apt install -y python3-pip && \
  apt install -y ros-humble-novatel-oem7-driver &&
  apt install -y ros-humble-novatel-oem7-msgs &&
  apt-get install ros-humble-sensor-msgs-py

WORKDIR /home/Web
RUN git clone https://github.com/bomena/sensor_monitoring_ros2.git
WORKDIR /home/Web/sensor_monitoring_ros2
RUN chmod +x run.sh
RUN chmod +x record.sh
RUN npm install
RUN apt-get install ros-humble-rosbridge-suite --yes
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN apt update
WORKDIR /home/Web/sensor_monitoring_ros2/src/function
RUN chmod +x run.sh
WORKDIR /home/Web/sensor_monitoring_ros2
