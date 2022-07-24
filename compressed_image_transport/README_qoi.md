# 

```bash
xhost local:root; docker run -it --privileged -v /tmp/.X11-unix:/tmp/.X11-unix --env="DISPLAY=${DISPLAY}" --net=host -h $HOSTNAME  -v $(pwd)/src:/home/batman/qoi_ws/src --name openpilot_ros_noetic auconav/openpilot_ros_noetic bash
sudo apt-get update
sudo apt-get install ros-noetic-image-transport ros-noetic-image-transport-plugins ros-noetic-camera-info-manager
```