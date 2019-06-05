FROM introlab3it/rtabmap_ros:kinetic-latest
# Install package
RUN apt-get update && apt-get install -y libopenni2-dev wget vim\
    && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list' \
    && wget http://packages.ros.org/ros.key -O - | apt-key add - \
    && apt-get update \
    && apt-get install -y python-catkin-tools \
    && rm -rf /var/lib/apt/lists/

WORKDIR /root/

ARG counter1
# Clone my repo
RUN git clone https://github.com/bramtoula/multi_robot_SLAM_separators.git 
