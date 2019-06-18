FROM introlab3it/rtabmap_ros:kinetic-latest

# Python 2
RUN apt-get update && apt-get install -y libopenni2-dev wget vim unzip python-pip \
    && pip install tensorflow opencv-python scipy==0.16\
    && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list' \
    && wget http://packages.ros.org/ros.key -O - | apt-key add - \
    && apt-get update \
    && apt-get install -y python-catkin-tools \
    && rm -rf /var/lib/apt/lists/

# # Python 3
# RUN apt-get update && apt-get install -y libopenni2-dev wget vim unzip python3-pip python3-yaml \
#     && pip3 install tensorflow opencv-python scipy rospkg catkin_pkg\
#     && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list' \
#     && wget http://packages.ros.org/ros.key -O - | apt-key add - \
#     && apt-get update \
#     && apt-get install -y python-catkin-tools \
#     && rm -rf /var/lib/apt/lists/

WORKDIR /root/

ARG counter1
# Clone my repo
RUN git clone https://github.com/bramtoula/multi_robot_SLAM_separators.git 

RUN cd multi_robot_SLAM_separators/ros_ws/src/multi_robot_separators/scripts && \
    git clone https://github.com/uzh-rpg/netvlad_tf_open.git && \
    cd netvlad_tf_open && \
    apt-get install -y wget unzip && \
    wget http://rpg.ifi.uzh.ch/datasets/netvlad/vd16_pitts30k_conv5_3_vlad_preL2_intra_white.zip && \
    mkdir checkpoints && \
    unzip vd16_pitts30k_conv5_3_vlad_preL2_intra_white.zip && \
    mv vd16_pitts30k_conv5_3_vlad_preL2_intra_white/* checkpoints && \
    cd matlab && \
    wget http://rpg.ifi.uzh.ch/datasets/netvlad/example_stats.mat 

RUN chmod +x multi_robot_SLAM_separators/ros_ws/src/multi_robot_separators/scripts/find_separators.py

# # Fix python path to use python3 with ros kinetic
# RUN echo 'export PYTHONPATH=/usr/local/lib/python3.5/dist-packages/:$PYTHONPATH' >> .bashrc 
ENV PYTHONPATH=":PYTHONPATH:/root/multi_robot_SLAM_separators/ros_ws/src/multi_robot_separators/scripts/netvlad_tf_open/python"