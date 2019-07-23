FROM ros:melodic-perception
RUN apt-get update && apt-get install -y \
    wget git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev sudo unzip \
    && rm -rf /var/lib/apt/lists/*


RUN git clone https://github.com/lajoiepy/buildLibrealsense2TX2.git && \
    cd buildLibrealsense2TX2 && \
    chmod +x installLibrealsense.sh && \
    sed -i -e 's/sudo//g' ./installLibrealsense.sh && \
    ./installLibrealsense.sh


# RUN useradd -m docker && echo "docker:docker" | chpasswd && adduser docker sudo


# RUN git clone https://github.com/IntelRealSense/librealsense.git
# USER docker

# RUN cd librealsense && \
#     ./scripts/setup_udev_rules.sh && \
#     ./scripts/patch-realsense-ubuntu-lts.sh
# USER root
# RUN mkdir build && cd build && \
#     cmake ../ -DCMAKE_BUILD_TYPE=Release && \
#     make uninstall && make clean && make && make install


# RUN apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE && \
#     add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u && \
#     apt-get install -y librealsense2-dkms && \
#     apt-get install -y librealsense2-utils && \
#     apt-get install -y librealsense2-dev && \
#     apt-get install -y librealsense2-dbg

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

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
