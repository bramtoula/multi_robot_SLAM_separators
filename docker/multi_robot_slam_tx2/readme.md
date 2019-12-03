**Detailed documentation coming soon. These steps are no longer required with the latest Jetpack versions, which provide an nvidia runtime with Docker.**

The following steps were necessary on the TX2 to run through Docker:

Make sure you can use [Tegra Docker](https://github.com/Technica-Corporation/Tegra-Docker/).

On the TX2 itself, you need to install some libraries (so that they are available through the shared folders when running the Tegra Docker wrapper):
```
sudo apt-get update
sudo apt-get install rhash curl libuv1 libsqlite3-dev libsuitesparse-dev libfreenect-dev libdc1394-22-dev libglvnd-dev libopencv-contrib3.2 libmetis-dev libboost-all-dev libpcl-dev liblz4-dev libogre-1.9-dev liburdfdom-dev liblog4cxx-dev libtinyxml2-dev libassimp4 libyaml-cpp0.5v5
```


Build the Docker image from this directory, and run it (with tx2-docker if you are using the wrapper from Tegra Docker):
```
docker build -t multi_robot_separators .
tx2-docker run -it multi_robot_separators
```

