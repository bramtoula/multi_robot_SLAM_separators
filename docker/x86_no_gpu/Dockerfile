FROM introlab3it/rtabmap_ros:melodic-latest

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
RUN git clone https://bitbucket.org/gtborg/gtsam  \
    && cd gtsam \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j4 \
    && make install

# Download resources needed later for netvlad
RUN git clone https://github.com/uzh-rpg/netvlad_tf_open.git && \
    wget http://rpg.ifi.uzh.ch/datasets/netvlad/vd16_pitts30k_conv5_3_vlad_preL2_intra_white.zip && \
    unzip vd16_pitts30k_conv5_3_vlad_preL2_intra_white.zip && \ 
    wget http://rpg.ifi.uzh.ch/datasets/netvlad/example_stats.mat 

ARG reset_clone0

RUN git clone https://github.com/bramtoula/multi_robot_SLAM_separators.git  && \
    cd multi_robot_SLAM_separators/ros_ws/src/multi_robot_separators/scripts && \
    mv ~/netvlad_tf_open . && \
    cd netvlad_tf_open && \
    mv ~/vd16_pitts30k_conv5_3_vlad_preL2_intra_white/* checkpoints && \
    cd matlab && \
    mv ~/example_stats.mat . && \
    cd ~/multi_robot_SLAM_separators/ros_ws && \
    source /opt/ros/melodic/setup.bash && \
    catkin build && \
    source devel/setup.bash && \
    mkdir ~/multi_robot_SLAM_separators/logs

RUN chmod +x multi_robot_SLAM_separators/ros_ws/src/multi_robot_separators/scripts/find_separators.py

ENV PYTHONPATH=":PYTHONPATH:/root/multi_robot_SLAM_separators/ros_ws/src/multi_robot_separators/scripts/netvlad_tf_open/python"
