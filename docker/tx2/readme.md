**STILL WIP (Not everything is online, working, or detailed yet)**

The following steps were necessary on the TX2 to run through Docker:

Make sure you can use [Tegra Docker](https://github.com/Technica-Corporation/Tegra-Docker/).

On the TX2 itself, you need to install some libraries (so that they are available through the shared folders when running the Tegra Docker wrapper):
```
sudo apt-get update
sudo apt-get install rhash curl libuv1 libsqlite3-dev libsuitesparse-dev libfreenect-dev libdc1394-22-dev libglvnd-dev libopencv-contrib3.2
```

There is also the need for some other libopencv packages, but I'm not sure if the best way is to copy them directly in the container, or figure out how to install them through the package manager yet...

Build the Docker image from this directory, and run it (with tx2-docker if you are using the wrapper from Tegra Docker):
```
docker build -t multi_robot_separators .
tx2-docker run -it multi_robot_separators
```

