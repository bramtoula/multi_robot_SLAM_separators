FROM ros:melodic-perception
RUN apt-get update && apt-get install -y \
    wget git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev sudo unzip \
    && rm -rf /var/lib/apt/lists/*


RUN git clone https://github.com/lajoiepy/buildLibrealsense2TX2.git && \
    cd buildLibrealsense2TX2 && \
    chmod +x installLibrealsense.sh && \
    sed -i -e 's/sudo//g' ./installLibrealsense.sh && \
    ./installLibrealsense.sh

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

RUN apt-get update && \
    apt-get install -y nano vim

ARG rebuild_realsense_ros=unknown

RUN apt-get update && \
    apt-get install -y --no-install-recommends ros-melodic-ddynamic-reconfigure ros-melodic-diagnostic-updater && \
    rm -rf /var/lib/apt/lists/* && \
    mkdir -p ~/catkin_ws/src && \
    cd ~/catkin_ws/src/ && \
    git clone https://github.com/IntelRealSense/realsense-ros.git && \
    cd realsense-ros/ && \
    git checkout `git tag | sort -V | grep -P "^\d+\.\d+\.\d+" | tail -1` && \
    cd .. && \
    source /opt/ros/melodic/setup.bash && \
    catkin_init_workspace && \
    cd .. && \
    catkin_make clean && \
    catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release && \
    catkin_make install && \
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc && \
    source ~/.bashrc

RUN cd ~/catkin_ws/src/realsense-ros/realsense2_camera/launch/ && \
    git clone https://github.com/lajoiepy/multi_robot_slam_camera_launch.git

COPY entrypoint.sh entrypoint.sh
RUN chmod +x entrypoint.sh
WORKDIR /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["100", "0", "camera"]
