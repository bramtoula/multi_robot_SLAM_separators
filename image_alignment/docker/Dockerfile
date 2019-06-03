FROM introlab3it/rtabmap_ros:kinetic-latest
# Install package
RUN apt-get update && apt-get install -y libopenni2-dev \
    && rm -rf /var/lib/apt/lists/

WORKDIR /root/

# Clone my repo
RUN git clone https://github.com/bramtoula/multi_robot_SLAM_separators.git