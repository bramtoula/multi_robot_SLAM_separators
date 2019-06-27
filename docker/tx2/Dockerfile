FROM ros:melodic-perception

# install rtabmap packages
RUN apt-get update && apt-get install -y \
    ros-melodic-rtabmap-ros \
    && apt-get remove -y \
    ros-melodic-rtabmap \
    && rm -rf /var/lib/apt/lists/

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

WORKDIR /root/

ARG CACHE_DATE=2016-01-01

RUN git clone https://github.com/introlab/rtabmap.git

# Build RTAB-Map project
RUN source /ros_entrypoint.sh && \
    cd rtabmap/build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    cd ../.. && \
    rm -rf rtabmap && \
    ldconfig

RUN source /ros_entrypoint.sh && \
    mkdir -p catkin_ws/src && \
    cd catkin_ws/src && \
    catkin_init_workspace && \
    git clone https://github.com/introlab/rtabmap_ros.git && \
    cd .. && \
    catkin_make -j2 -DCMAKE_INSTALL_PREFIX=/opt/ros/melodic install && \
    cd && \
    rm -rf catkin_ws

#  Install tensorflow 
RUN apt-get update && \
    apt-get install -y python-dev python-pip python-h5py

COPY tensorflow-1.11.0-cp27-cp27mu-linux_aarch64.whl /tmp/
RUN pip install /tmp/tensorflow-1.11.0-cp27-cp27mu-linux_aarch64.whl && \
    pip install --upgrade numpy && \
    apt-get install -y python-scipy


WORKDIR /root/

# Some tools
RUN apt-get update && apt-get install -y libopenni2-dev wget vim unzip \
    # Python catkin tools
    && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list' \
    && wget http://packages.ros.org/ros.key -O - | apt-key add - \
    && apt-get update \
    && apt-get install -y python-catkin-tools \
    && rm -rf /var/lib/apt/lists/

# GTSAM 
RUN git clone https://bitbucket.org/gtborg/gtsam  \
    && cd gtsam \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j4 \
    && make install

# Clone my repo
RUN git clone https://github.com/bramtoula/multi_robot_SLAM_separators.git 

RUN cd multi_robot_SLAM_separators/ros_ws/src/multi_robot_separators/scripts && \
    git clone https://github.com/uzh-rpg/netvlad_tf_open.git && \
    cd netvlad_tf_open && \
    apt-get install -y wget unzip && \
    wget http://rpg.ifi.uzh.ch/datasets/netvlad/vd16_pitts30k_conv5_3_vlad_preL2_intra_white.zip && \
    unzip vd16_pitts30k_conv5_3_vlad_preL2_intra_white.zip && \
    mv vd16_pitts30k_conv5_3_vlad_preL2_intra_white/* checkpoints && \
    cd matlab && \
    wget http://rpg.ifi.uzh.ch/datasets/netvlad/example_stats.mat 

RUN chmod +x multi_robot_SLAM_separators/ros_ws/src/multi_robot_separators/scripts/find_separators.py

ENV PYTHONPATH=":PYTHONPATH:/root/multi_robot_SLAM_separators/ros_ws/src/multi_robot_separators/scripts/netvlad_tf_open/python"