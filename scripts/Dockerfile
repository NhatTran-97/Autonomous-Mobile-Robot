# FROM ros:humble
FROM nhatbot:latest

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /home
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    nano \
    gedit \
    net-tools \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    dbus-x11 \
    usbutils \
    libmodbus-dev \
    libsm6 libxext6 libxrender-dev \
    libgl1-mesa-glx \
    v4l-utils \
    netcat \
    ros-humble-rviz2 \
    ros-humble-tf-transformations \
    ros-humble-joy \
    ros-humble-joy-teleop \
    ros-humble-cv-bridge \
    ros-humble-rosbridge-server \
    ros-humble-twist-mux* \
    ros-humble-cv-bridge \
    ros-humble-nav2* \
    ros-humble-nav2-simple-commander \
    ros-humble-robot-localization \
    ros-humble-rqt-tf-tree \
    ros-humble-ros2-control && \
    apt-get clean && rm -rf /var/lib/apt/lists/*
 
RUN python3 -m pip install --upgrade pip && \
    pip3 install --no-cache-dir --break-system-packages opencv-python pymodbus==2.3.0 asyncio pyserial transforms3d websocket-client


RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc


CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && export $(dbus-launch) && exec bash"]

