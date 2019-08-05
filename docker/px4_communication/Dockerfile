FROM lajoiepy/realsense-tx2

RUN apt-get update && \
    apt-get -y install screen python-wxgtk3.0 python-matplotlib python-opencv python-pip python-numpy python-dev libxml2-dev libxslt1-dev ros-melodic-mavros* ros-melodic-mavlink && \
    pip install future pymavlink mavproxy && \
    wget https://raw.githubusercontent.com/mavlink/mavros/de9f39a719b091b8448214a17d27b3b1c415d0dc/mavros/scripts/install_geographiclib_datasets.sh && \
    chmod +x install_geographiclib_datasets.sh && \
    ./install_geographiclib_datasets.sh && \
    rm install_geographiclib_datasets.sh && \
    rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh entrypoint.sh
RUN chmod +x entrypoint.sh
ENTRYPOINT ["/root/entrypoint.sh"]
