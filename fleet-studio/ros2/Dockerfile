# Raspberry Pi (arm64) üzerinde ROS 2 Humble + rosbridge
FROM ros:humble-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-rosbridge-suite ros-humble-rplidar-ros \
    && rm -rf /var/lib/apt/lists/*
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090"]
