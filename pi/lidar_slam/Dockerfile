FROM ros:humble-ros-base

# 1) Eski imajla AYNI katman -> Docker onbellekten kullanir, yeniden indirmez
RUN apt-get update && apt-get install -y \
    ros-humble-rplidar-ros ros-humble-cartographer-ros \
    ros-humble-foxglove-bridge ros-humble-nav2-map-server \
 && rm -rf /var/lib/apt/lists/*
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# 2) YENI: agv_gateway (masaustu uygulamasi koprusu, ws://agv.local:8765) calisma zamani bagimliliklari
#    (slam_toolbox gerekmez: gateway onu istege bagli import eder; harita map_saver_cli ile kaydedilir)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-nav2-msgs ros-humble-tf2-ros-py \
    python3-numpy python3-yaml python3-pip python3-colcon-common-extensions \
 && pip3 install --no-cache-dir "websockets>=11,<14" \
 && rm -rf /var/lib/apt/lists/*

# 3) agv_gateway paketini derle -> /opt/agv_ws/install (saf Python, Pi 4'te birkac saniye)
WORKDIR /opt/agv_ws
COPY src/agv_gateway src/agv_gateway
RUN . /opt/ros/humble/setup.sh \
 && colcon build --packages-select agv_gateway \
 && rm -rf build log
RUN echo "source /opt/agv_ws/install/setup.bash" >> /root/.bashrc
WORKDIR /
