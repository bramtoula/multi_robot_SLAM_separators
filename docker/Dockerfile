FROM ros:melodic


WORKDIR /root

# Python things
RUN apt-get update && \
    apt-get install -y python3-pip && \
    pip3 install tensorflow-gpu opencv-python scipy

# Netvlad tensorflow implementation from UZH-RPG
RUN git clone https://github.com/uzh-rpg/netvlad_tf_open.git && \
    cd netvlad_tf_open && \
    apt-get install -y wget unzip && \
    wget http://rpg.ifi.uzh.ch/datasets/netvlad/vd16_pitts30k_conv5_3_vlad_preL2_intra_white.zip && \
    mkdir checkpoints && \
    unzip vd16_pitts30k_conv5_3_vlad_preL2_intra_white.zip && \
    mv vd16_pitts30k_conv5_3_vlad_preL2_intra_white/* checkpoints

RUN cd netvlad_tf_open/matlab && \
    wget http://rpg.ifi.uzh.ch/datasets/netvlad/example_stats.mat


ENV PYTHONPATH="$PYTHONPATH:/root/netvlad_tf_open/python"




