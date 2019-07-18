FROM ros:melodic-perception

WORKDIR /root/

##################################################### Python catkin tools #####################################################
# Fix from http://answers.ros.org/question/325039/apt-update-fails-cannot-install-pkgs-key-not-working/, install python stuff and catkin tools
RUN apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116 && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && apt-get install --no-install-recommends -y libopenni2-dev libmetis-dev wget && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list' && \
    wget http://packages.ros.org/ros.key -O - | apt-key add - && \
    apt-get remove -y wget && \
    apt-get update && \
    apt-get install --no-install-recommends -y python-catkin-tools && \
    apt-get --purge autoremove python-pip && \
    rm -rf /var/lib/apt/lists/*

##################################################### Tensorflow #####################################################
RUN apt-get update && \
    apt-get install -y --no-install-recommends python-pip && \
    pip install tensorflow opencv-python scipy==0.16 && \
    apt-get --purge autoremove -y python-pip && \
    rm -rf /var/lib/apt/lists/*

##################################################### GTSAM #####################################################
RUN git clone https://bitbucket.org/gtborg/gtsam  && \
    cd gtsam && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j2 && \
    make install && \
    cd ../.. && \
    rm -rf gtsam && \
    ldconfig

##################################################### RTAB-Map ROS #####################################################
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

RUN git clone https://github.com/introlab/rtabmap.git && \
    source /ros_entrypoint.sh && \
    cd rtabmap/build && \
    cmake .. && \
    make -j2 && \
    make install && \
    cd ../.. && \
    rm -rf rtabmap && \
    ldconfig

RUN source /ros_entrypoint.sh && \
    apt-get update && \
    # Install dependencies of original rtabmap_ros
    apt-get install -y --no-install-recommends $(apt-cache depends ros-melodic-rtabmap-ros | grep Depends | sed "s/.*ends:\ //" | tr '\n' ' ') && \
    rm -rf /var/lib/apt/lists/* && \
    mkdir -p catkin_ws/src && \
    cd catkin_ws/src && \
    catkin_init_workspace && \
    git clone https://github.com/bramtoula/rtabmap_ros.git && \
    cd .. && \
    catkin_make -j2 -DCMAKE_INSTALL_PREFIX=/opt/ros/melodic install && \
    cd && \
    rm -rf catkin_ws

##################################################### Buzz #####################################################
RUN git clone https://github.com/MISTLab/Buzz.git /home/docker/buzz && \
    cd /home/docker/buzz && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Debug ../src && \
    make && \
    make install && \
    ldconfig && \
    rm -r /home/docker/buzz

##################################################### Netvlad stuff  #####################################################
# RUN apt-get update && \
#     apt-get install -y --no-install-recommends unzip wget && \
#     git clone https://github.com/uzh-rpg/netvlad_tf_open.git && \
#     wget http://rpg.ifi.uzh.ch/datasets/netvlad/vd16_pitts30k_conv5_3_vlad_preL2_intra_white.zip && \
#     unzip vd16_pitts30k_conv5_3_vlad_preL2_intra_white.zip && \ 
#     rm vd16_pitts30k_conv5_3_vlad_preL2_intra_white.zip && \
#     apt-get remove -y unzip wget && \
#     rm -rf /var/lib/apt/lists/*

##################################################### Multi-robot SLAM separators package #####################################################
# --------------------------------------------------- Reset argument ------------------------------------------#
ARG rebuild_multi_robot_slam_separators=0
# -------------------------------------------------------------------------------------------------------------#

RUN git clone https://github.com/bramtoula/multi_robot_SLAM_separators.git  && \
    cd multi_robot_SLAM_separators/ros_ws/src/multi_robot_separators/scripts && \
    git clone https://github.com/uzh-rpg/netvlad_tf_open.git && \
    cd netvlad_tf_open/checkpoints && \
    apt-get update && \
    apt-get install -y --no-install-recommends unzip wget && \
    wget http://rpg.ifi.uzh.ch/datasets/netvlad/vd16_pitts30k_conv5_3_vlad_preL2_intra_white.zip && \
    unzip vd16_pitts30k_conv5_3_vlad_preL2_intra_white.zip && \ 
    mv vd16_pitts30k_conv5_3_vlad_preL2_intra_white/* . && \
    rm -r vd16_pitts30k_conv5_3_vlad_preL2_intra_white && \
    rm vd16_pitts30k_conv5_3_vlad_preL2_intra_white.zip && \
    apt-get remove -y unzip wget && \
    rm -rf /var/lib/apt/lists/* && \
    cd ~/multi_robot_SLAM_separators/ros_ws && \
    source /opt/ros/melodic/setup.bash && \
    catkin build && \
    source devel/setup.bash && \
    mkdir ~/multi_robot_SLAM_separators/logs && \
    mkdir ~/multi_robot_SLAM_separators/bags && \
    chmod +x  ~/multi_robot_SLAM_separators/ros_ws/src/multi_robot_separators/scripts/find_separators.py 

ENV PYTHONPATH=":PYTHONPATH:/root/multi_robot_SLAM_separators/ros_ws/src/multi_robot_separators/scripts/netvlad_tf_open/python"

##################################################### Robust Distributed Mapper - RDPGO - Buzz SLAM #####################################################
# --------------------------------------------------- Reset argument ------------------------------------------#
ARG rebuild_rdpgo=0
# -------------------------------------------------------------------------------------------------------------#
# Clone RDPGO
RUN mkdir ~/rdpgo_ws && mkdir ~/rdpgo_ws/src && cd ~/rdpgo_ws/src &&\
    git clone https://github.com/lajoiepy/robust_distributed_slam_module.git robust_distributed_slam_module &&\
    cd robust_distributed_slam_module &&\
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

##################################################### Debugging tools #####################################################
# Install and setup some editing and debugging tools
RUN apt-get update && apt-get install -y --no-install-recommends vim htop net-tools gedit nano wget strace valgrind && \
    wget -O vscode.deb https://go.microsoft.com/fwlink/?LinkID=760868 &&\ 
    apt install -y --no-install-recommends ./vscode.deb && \
    apt install -y --no-install-recommends libasound2 && \
    rm -rf /var/lib/apt/lists/* 

RUN mkdir ~/code &&\
    code --install-extension ms-vscode.cpptools --user-data-dir ~/code &&\
    code --install-extension ms-python.python --user-data-dir ~/code

# Settings for buzz editing
RUN touch ~/code/User/settings.json &&\
    echo "{ \"files.associations\": { \"*.bzz\":\"python\" } }" > ~/code/User/settings.json

# Setup debug config for visual studio code
RUN mkdir ~/multi_robot_SLAM_separators/.vscode && touch ~/multi_robot_SLAM_separators/.vscode/launch.json &&\
    echo "{\"version\": \"0.2.0\",\"configurations\": [{\"name\": \"(gdb) Attach\", \"type\": \"cppdbg\", \"request\": \"attach\", \"program\" : \"/home/docker/multi_robot_SLAM_separators/build/\", \"processId\": \"\${command:pickProcess}\", \"MIMode\": \"gdb\", \"setupCommands\": [ {\"description\": \"Enable pretty-printing for gdb\", \"text\": \"-enable-pretty-printing\",\"ignoreFailures\": true}]}]}" \
    > ~/multi_robot_SLAM_separators/.vscode/launch.json




##################################################### Repull and build #####################################################
# --------------------------------------------------- Reset argument ------------------------------------------#
ARG repull_and_build=1
# -------------------------------------------------------------------------------------------------------------#

RUN cd multi_robot_SLAM_separators/ros_ws/ && \
    git pull && \
    source /opt/ros/melodic/setup.bash && \
    catkin build && \
    cd ~/rdpgo_ws/src/robust_distributed_slam_module && \
    git pull --recurse-submodules && \
    cd  ~/rdpgo_ws/src/robust_distributed_slam_module/buzz_slam/robust_distributed_mapper/cpp/build && \
    cmake -DCMAKE_BUILD_TYPE=Debug ../ && \
    make && \
    make install && \
    cd ~/rdpgo_ws/ &&\
    source ~/multi_robot_SLAM_separators/ros_ws/devel/setup.bash &&\
    catkin build && \
    cd ~/rdpgo_ws/src/robust_distributed_slam_module/scripts &&\
    bzzc ros_slam.bzz

