# Steps we used to setup our TX2s using Jetpack 4.2. (more recent Jetpack versions make the use of Docker easier)

Steps in order to use a TX2 GPU in Docker, with B.A.T.M.A.N., a TP-Link Archer T2UH dongle, a RealSense D435, based on Jetpack 4.2.

## Flash the OS using Jetpack 4.2
For this step, we use NVIDIA's SDK manager.

Download the SDK manager here: https://developer.nvidia.com/embedded/jetpack.

Install it, and then you can open it typing `sdkmanager` in a terminal.

If you don't plan on using an older Connectech Elroy carrier board (revision E and earlier), or don't care about using USB 3 on it, you can follow the steps to install Jetpack 4.2 (not 4.2.1 in our case, since we had issues with librealsense with this version), making sure to flash the OS and install the packages you'll need.

### If you want to use an older Connectech Elroy carrier board (revision E and earlier) with USB 3.
In this case, you have to install the Connecttech Board Support Package.

First, flash the TX using the instructions found here: http://connecttech.com/resource-center/cti-l4t-nvidia-jetson-board-support-package-release-notes/.

Once the TX is flashed, it still doesn't have everything you might want installed. You'll have to reboot it and connect a keyboard and screen to it to finalize the setup (choose login, password, etc.).
To finish the process, make sure the TX2 is booted on the Ubuntu homepage and connected through USB to the desktop you use with the NVIDIA SDK manager. 

Open the SDK manager, and install the additional modules you want. Make sure you don't flash it again, but only install additional modules.


## Rebuild the kernel to have the B.A.T.M.A.N. module, and librealsense.
We have scripts in order to rebuild the kernel with the necessary modules.
This will take a while to run though.
```
git clone https://github.com/bramtoula/buildLibrealsense2AndBatmanTX2.git
cd buildLibrealsense2AndBatmanTX2
./buildPatchedKernel.sh
sudo cp ./image/Image /boot
```

Reboot after this.

Now activate the batman-adv module and install some necessary packages:
```
sudo modprobe batman-adv
sudo apt-get install batctl bridge-utils
```


## Install drivers for the TP-Link Archer T2UH dongle.
```
git clone https://github.com/xtknight/mt7610u-linksys-ae6000-wifi-fixes.git
cd mt7610u-linksys-ae6000-wifi-fixes
make clean
make
sudo make install
```

Now if you connect the dongle and reboot, you should see it, probably on ra0, when running `iwconfig`.

## Clean up
We can clean the repos used to rebuild the kernel and the dongle driver, and remove the kernel sources:
```
cd ~/buildLibrealsense2AndBatmanTX2
./removeAllKernelSources.sh
cd ..
rm -rf mt7610u-linksys-ae6000-wifi-fixes buildLibrealsense2AndBatmanTX2
```

## Prepare the TX so that the GPU is usable with Docker.
First, let's make sure we don't need sudo to run Docker.
Run:
```
sudo groupadd docker
sudo usermod -aG docker $USER
```
Once you'll have rebooted, you should be able to run Docker without sudo.

For the GPU to be used within Docker containers, we use [Tegra-Docker](https://github.com/Technica-Corporation/Tegra-Docker).

On the TX, run the following:
```
git clone https://github.com/Technica-Corporation/Tegra-Docker.git
chmod +x Tegra-Docker/bin/tx2-docker
sudo cp Tegra-Docker/bin/tx2-docker /usr/bin
rm -rf Tegra-Docker
```

Now you can use the `tx2-docker` command instead of `docker` when running a container.

Doing this might overwrite some libraries installed in your images (since it replaces some directories). To fix this, you should install the required libraries locally on the TX2, and they will then be available in the shared libraries within the Docker container.

## Set up network
We use two wireless interfaces. We use wlan0 (original interface) to connect normally to a WiFi network, through a router, with a static IP address.
We use ra0 (dongle) to connect to a mesh network with other TX2s.

### wlan0 interface on our router
To connect to a network through the wifi interface, we use wpa supplicant, and set the IP to be static. In our example, we assume that the IP addresses your network gives are `192.168.12.XX`.

Open the interfaces file to setup the IP to be static :
`sudo nano /etc/network/interfaces`
You should have the following inside, with the IP you defined for your Spiri :
```
# interfaces(5) file used by ifup(8) and ifdown(8)
# Include files from /etc/network/interfaces.d:
source-directory /etc/network/interfaces.d

# The loopback network interface
auto lo
iface lo inet loopback

auto wlan0
iface wlan0 inet static
address 192.168.12.<the number you chose for your drone’s IP>
netmask 255.255.255.0
gateway 192.168.12.1
wpa-conf /etc/wpa_supplicant/wpa_supplicant.conf
```

Second, open the wpa supplicant conf file to define which networks to connect to :
`sudo nano /etc/wpa_supplicant/wpa_supplicant.conf`

In this file, you can set up one or more networks you want to connect to. If the network is open and has no password, you should use the configuration like the first network below. If it does require a password, use the second network below. You can add as many networks as you want, defining different priorities. If a robot sees different available networks it will connect to the one with highest priority.
If you are using some kind of filtering on your router (MAC filtering for example), don't forget to allow your Spiri!
```
network={
        ssid="<open-network-name>"
        scan_ssid=1
        key_mgmt=NONE
        priority=6
        }

network={
        ssid="<password-protected-network-name>"
        scan_ssid=1
        psk="network-password"
        key_mgmt=WPA-PSK
        priority=5
        }
```

Once you have these files changed, reboot the drone and make sure it connects to the network.
You can check your drone gets the expected IP using `ifconfig`.
You can verify that it is connected to the network by trying to ping the router from the TX :
`ping 192.168.12.1`
If you get no response then there is an issue somewhere.
The next test you should make is to try to SSH to the TX. From another computer on the same network use :
`ssh nvidia@192.168.12.XX`
replacing XX by the IP of your TX. If you didn’t change the password yet, it should still be `nvidia`.

One helpful thing you can do is add the drone to your ssh config file to easily connect to it later. On your computer, run :
`sudo nano ~/.ssh/config`
And add the following at the end, replacing <spiri-name> and <spiri-IP> by the actual name and static IP of your Spiri :
```
Host <spiri-name>
        User nvidia
        Hostname 192.168.12.<spiri-IP>
```

Now you'll be able to run the following command on your computer to access the drone :
`ssh <spiri-name>`

It is important that this works before going forward since it will be our only way to access the TX once it is inserted in the drone.

### Setting ra0 and bat0 interfaces manually (better for debugging)

#### ra0 in a mesh network for B.A.T.M.A.N

First, make sure ra0 is visible when you run `iwconfig`.
Then run the following commands to join (or create if you are the first to join) a mesh network:
```
sudo iw dev ra0 set type ibss
sudo iw dev ra0 ibss join NVIDIA 5220 
```

To check whether it has been set correctly, you can run
```
sudo iw dev ra0 info
```


#### Batman

Now to use batman:
```
sudo batctl if add ra0
sudo ip link set up dev bat0
```

To obtain an IPv4 address associated to the batman interface, run:
```
sudo avahi-autoipd bat0 &
```

You can now check if you see other devices using batman either by using ping with their bat0 IPv4 addresses, or checking the neighbors:
```
sudo batctl n
```


### Setting ra0 and bat0 interfaces automatically on boot, with a static IP.
We have a script ready which can run on boot and sets up the interface with a static and manually set IP.
You can find the script here: [run_batman.sh](run_batman.sh).

You should copy this script in `/etc/init.d/`, and modify the `192.168.10.150` by the static IP address you want on the bat0 interface.

Then run the following commands for the script to run on boot:
```
cd /etc/init.d
sudo chmod +x run_batman.sh
sudo update-rc.d run_batman.sh defaults
```

Once you'll reboot you should be able to check that the ra0 interface is of type IBSS connected to the NVIDIA SSID with:
```
sudo iw dev ra0 info
```

You should see the bat0 interface when running `ifconfig`, with the static IP you chose.

## Install libraries needed for our code
On the TX2 itself, we need libraries used in our Docker containers.
```
sudo apt-get install rhash curl libuv1 libsqlite3-dev libsuitesparse-dev libfreenect-dev libdc1394-22-dev libglvnd-dev libopencv-contrib3.2 libmetis-dev libboost-all-dev libpcl-dev liblz4-dev libogre-1.9-dev liburdfdom-dev liblog4cxx-dev libtinyxml2-dev libassimp4 libyaml-cpp0.5v5 geographiclib-tools
```



