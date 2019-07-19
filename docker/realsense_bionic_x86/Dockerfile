FROM ros:melodic-perception
RUN apt-get update && apt-get install -y \
    unzip wget git software-properties-common udev \
    && rm -rf /var/lib/apt/lists/*

RUN apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE && \
    add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u && \
    apt-get install -y librealsense2-dkms && \
    apt-get install -y librealsense2-utils && \
    apt-get install -y librealsense2-dev && \
    apt-get install -y librealsense2-dbg

RUN rm /bin/sh && ln -s /bin/bash /bin/sh
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