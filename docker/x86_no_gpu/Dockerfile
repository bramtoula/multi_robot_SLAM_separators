FROM ros:melodic-perception

# Install common dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    dpkg \ 
    git \
    pkg-config \
    python \
    python-dev \
    python-numpy \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Fix from http://answers.ros.org/question/325039/apt-update-fails-cannot-install-pkgs-key-not-working/, install python stuff and catkin tools
RUN apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116 \
    &&  apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt-get update && apt-get install -y libopenni2-dev wget vim unzip python-pip libmetis-dev\
    && pip install tensorflow opencv-python scipy==0.16 \
    && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list' \
    && wget http://packages.ros.org/ros.key -O - | apt-key add - \
    && apt-get update \
    && apt-get install -y python-catkin-tools \
    && rm -rf /var/lib/apt/lists/

WORKDIR /root/

# GTSAM 
RUN apt-get update && apt-get install -y \
    libboost-all-dev \
    cmake \
    libtbb-dev \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://bitbucket.org/gtborg/gtsam  \
    && cd gtsam \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j2 \
    && make install

# Install Eigen3 (packaged in GTSAM)
RUN cd gtsam/gtsam/3rdparty/Eigen && \
    mkdir build && \
    cd build && \
    cmake .. &&\
    make install

# Install Buzz
RUN git clone https://github.com/MISTLab/Buzz.git /home/docker/buzz
RUN cd /home/docker/buzz &&\
    mkdir build &&\
    cd build &&\
    cmake -DCMAKE_BUILD_TYPE=Debug ../src &&\
    make &&\
    make install &&\
    ldconfig

# Download resources needed later for netvlad
RUN git clone https://github.com/uzh-rpg/netvlad_tf_open.git && \
    wget http://rpg.ifi.uzh.ch/datasets/netvlad/vd16_pitts30k_conv5_3_vlad_preL2_intra_white.zip && \
    unzip vd16_pitts30k_conv5_3_vlad_preL2_intra_white.zip && \ 
    rm vd16_pitts30k_conv5_3_vlad_preL2_intra_white.zip && \
    wget http://rpg.ifi.uzh.ch/datasets/netvlad/example_stats.mat 

# Rtabmap-ros modified package
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
RUN apt-get update && \
    # Install dependencies of original rtabmap_ros
    apt-get install -y $(apt-cache depends ros-melodic-rtabmap-ros | grep Depends | sed "s/.*ends:\ //" | tr '\n' ' ') && \
    mkdir -p rtabmap_ros_ws/src && \
    cd rtabmap_ros_ws/src && \
    git clone https://github.com/bramtoula/rtabmap_ros.git && \
    cd .. && \
    source /opt/ros/melodic/setup.bash && \
    catkin build -j2

# Install and setup some editing and debugging tools
RUN apt-get update && apt-get install -y gedit nano strace valgrind

RUN wget -O vscode.deb https://go.microsoft.com/fwlink/?LinkID=760868 &&\
    apt install -y ./vscode.deb &&\
    apt install -y libasound2

RUN mkdir ~/code &&\
    code --install-extension ms-vscode.cpptools --user-data-dir ~/code &&\
    code --install-extension ms-python.python --user-data-dir ~/code

# Settings for buzz editing
RUN touch ~/code/User/settings.json &&\
    echo "{ \"files.associations\": { \"*.bzz\":\"python\" } }" > ~/code/User/settings.json

# Argument to clone the repo from scratch
ARG reset_clone0

# Multi robot SLAM separators
RUN git clone https://github.com/bramtoula/multi_robot_SLAM_separators.git  && \
    cd multi_robot_SLAM_separators/ros_ws/src/multi_robot_separators/scripts && \
    mv ~/netvlad_tf_open . && \
    cd netvlad_tf_open && \
    mv ~/vd16_pitts30k_conv5_3_vlad_preL2_intra_white/* checkpoints && \
    cd matlab && \
    mv ~/example_stats.mat . && \
    cd ~/multi_robot_SLAM_separators/ros_ws && \
    source ~/rtabmap_ros_ws/devel/setup.bash && \
    catkin build && \
    source devel/setup.bash && \
    mkdir ~/multi_robot_SLAM_separators/logs && \
    mkdir ~/multi_robot_SLAM_separators/bags

# Setup debug config for visual studio code
RUN mkdir ~/multi_robot_SLAM_separators/.vscode && touch ~/multi_robot_SLAM_separators/.vscode/launch.json &&\
    echo "{\"version\": \"0.2.0\",\"configurations\": [{\"name\": \"(gdb) Attach\", \"type\": \"cppdbg\", \"request\": \"attach\", \"program\" : \"/home/docker/multi_robot_SLAM_separators/build/\", \"processId\": \"\${command:pickProcess}\", \"MIMode\": \"gdb\", \"setupCommands\": [ {\"description\": \"Enable pretty-printing for gdb\", \"text\": \"-enable-pretty-printing\",\"ignoreFailures\": true}]}]}" \
    > ~/multi_robot_SLAM_separators/.vscode/launch.json
    
RUN chmod +x multi_robot_SLAM_separators/ros_ws/src/multi_robot_separators/scripts/find_separators.py

ENV PYTHONPATH=":PYTHONPATH:/root/multi_robot_SLAM_separators/ros_ws/src/multi_robot_separators/scripts/netvlad_tf_open/python"

# Clone RDPGO
RUN mkdir ~/rdpgo_ws && mkdir ~/rdpgo_ws/src && cd ~/rdpgo_ws/src &&\
    git clone https://github.com/lajoiepy/robust_distributed_slam_module.git robust_distributed_slam_module &&\
    cd robust_distributed_slam_module &&\
    git checkout ros_integration &&\
    git submodule update --init --recursive

# Install Robust Distributed Mapper
RUN cd  ~/rdpgo_ws/src/robust_distributed_slam_module/buzz_slam/robust_distributed_mapper/cpp &&\
    mkdir build &&\
    cd build &&\
    cmake -DCMAKE_BUILD_TYPE=Debug ../ &&\
    make &&\
    make install

# Install Buzz SLAM
RUN cd ~/rdpgo_ws/ &&\
    source ~/multi_robot_SLAM_separators/ros_ws/devel/setup.bash &&\
    catkin build

# Compile buzz script
RUN cd ~/rdpgo_ws/src/robust_distributed_slam_module/scripts &&\
    bzzc ros_slam.bzz

ARG update_repos=unknown