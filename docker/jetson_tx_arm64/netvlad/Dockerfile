FROM ros:melodic-perception

# Python things
RUN apt-get update && \
    apt-get install -y python3-pip && \
    apt-get install -y python3-scipy && \
    apt-get install -y wget unzip && \
    rm -rf /var/lib/apt/lists/*pt


# Netvlad tensorflow implementation from UZH-RPG
RUN git clone https://github.com/uzh-rpg/netvlad_tf_open.git && \
    cd netvlad_tf_open && \
    wget http://rpg.ifi.uzh.ch/datasets/netvlad/vd16_pitts30k_conv5_3_vlad_preL2_intra_white.zip && \
    mkdir checkpoints && \  
    unzip vd16_pitts30k_conv5_3_vlad_preL2_intra_white.zip && \
    rm vd16_pitts30k_conv5_3_vlad_preL2_intra_white.zip && \
    mv vd16_pitts30k_conv5_3_vlad_preL2_intra_white/* checkpoints && \
    cd matlab && \
    wget http://rpg.ifi.uzh.ch/datasets/netvlad/example_stats.mat && \
    apt-get remove -y wget unzip

# Install TF
RUN apt-get update && \
    apt-get -y install libhdf5-serial-dev hdf5-tools && \
    pip3 install --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v42 tensorflow-gpu==1.13.1+nv19.4 --user && \
    pip3 install --upgrade numpy  && \
    rm -rf /var/lib/apt/lists/*

# Install OpenCV
# # Use sudo command for the script
# RUN apt-get update && \
#     apt-get -y install sudo && \
#     useradd -m docker && echo "docker:docker" | chpasswd && adduser docker sudo && \
#     sudo apt-get -y install qttools5-dev qt5-default qttools5-dev-tools && \
#     rm -rf /var/lib/apt/lists/*

# USER docker

# # Get script to build and install opencv
# RUN cd ~/ && \
#     mkdir tmp && \
#     cd tmp && \
#     git clone https://github.com/jetsonhacks/buildOpenCVTX2.git
    
# RUN cd ~/tmp/buildOpenCVTX2 && \
#     wget https://nextcloud.benjaminramtoula.me/nextcloud/index.php/s/7zS5gLHCoGK4bda/download && \
#     mv download tmp.sh && \
#     chmod +x tmp.sh && \
#     ./tmp.sh
# RUN cd ~/opencv/build && \
#     sudo make install
#     #./buildOpenCV.sh
# # RUN sudo apt-get install -y libcurl3 && \
# #     cd ~/opencv/build && \
# #     sudo make install
 
# USER root

ENV PYTHONPATH="$PYTHONPATH:/root/netvlad_tf_open/python"




