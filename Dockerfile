FROM ubuntu:jammy
FROM osrf/ros:humble-desktop-full-jammy

RUN \
  apt-get -qq update && \
  apt-get -qq upgrade --yes && \
  apt-get -qq install vim git curl --yes
  
RUN apt-get update && apt-get install -y \
    libx11-xcb1 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-xinerama0 \
    libxcb-xkb1 \
    libxkbcommon-x11-0 \
    libxcb-shape0 \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sL https://deb.nodesource.com/setup_18.x | bash -
RUN apt-get -qq install nodejs --yes

RUN \
  apt-get -qq update && \
  apt install -y python3-pip && \
  apt install -y ros-humble-novatel-oem7-driver && \
  apt install -y ros-humble-novatel-oem7-msgs && \
  apt-get install -y ros-humble-sensor-msgs-py && \
  apt install -y ros-humble-rqt* --fix-missing

WORKDIR /home/Web
RUN git clone https://github.com/bomena/sensor_monitoring_ros2.git
WORKDIR /home/Web/sensor_monitoring_ros2
RUN chmod +x run.sh
RUN npm install
RUN apt-get install ros-humble-rosbridge-suite --yes
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN apt update
WORKDIR /home/Web/sensor_monitoring_ros2/src/function
RUN chmod +x python.sh
RUN chmod +x record.sh
WORKDIR /home/Web/sensor_monitoring_ros2
