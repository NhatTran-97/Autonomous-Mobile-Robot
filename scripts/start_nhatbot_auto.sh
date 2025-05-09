#!/bin/bash
export DISPLAY=:0
xhost +local:root  

sudo docker run -it \
    --rm \
    --name nhatbot_container \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="ROS_DOMAIN_ID=7" \
    --network=host \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --device=/dev/video0 \
    --device=/dev/bus/usb \
    --privileged \
    --volume="/home/ninhnt/nhatbot_ws:/home/nhatbot_ws" \
    nhatbot:latest \
    /bin/bash -c "source /home/nhatbot_ws/install/setup.bash && ros2 launch nhatbot_stack nhatbot_bringup.launch.py"


