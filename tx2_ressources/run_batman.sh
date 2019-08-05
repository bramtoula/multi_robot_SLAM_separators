#!/bin/bash
### BEGIN INIT INFO
# Provides:          ra0 setup for batman
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: Sets up the ra0 for a batman network defined in /etc/network/interfaces
# Description:       This file should be used to construct scripts to be
#                    placed in /etc/init.d.  This example start a
#                    single forking daemon capable of writing a pid
#                    file.  To get other behavoirs, implemend
#                    do_start(), do_stop() or other functions to
#                    override the defaults in /lib/init/init-d-script.
### END INIT INFO



run_batman()
{
    sleep 10
    # sudo ip addr flush dev bat0
    # sudo ip addr flush dev wlan0
    iw dev ra0 set type ibss
    iw dev ra0 ibss join NVIDIA 5220
    batctl if add ra0
    ifconfig bat0 192.168.10.150 netmask 255.255.255.0 up
    # route add default gw 192.168.10.1
    # sudo ip link set up dev bat0
    # sudo ip addr flush dev bat0
    # sudo ifdown bat0 && sudo ifup --ignore-errors bat0
    # sudo echo -e "# interfaces(5) file used by ifup(8) and ifdown(8)\n# Include files from /etc/network/interfaces.d:\nsource-directory /etc/network/interfaces.d\n\n# The loopback network interface\nauto lo\niface lo inet loopback\n\nauto wlan0\niface wlan0 inet static\naddress 192.168.12.150\nnetmask 255.255.255.0\ngateway 192.168.12.1\nwpa-conf /etc/wpa_supplicant/wpa_supplicant.conf\n\nauto bat0 \niface bat0 inet static \naddress 192.168.123.3 \nnetmask 255.255.255.0 \ngateway 192.168.123.1 \npre-up /usr/sbin/batctl if add ra0" > /etc/network/interfaces
    # sudo echo -e "\nallow-hotplug bat0 \niface bat0 inet static \naddress 192.168.123.4 \nnetmask 255.255.255.0 \ngateway 192.168.123.1 \npre-up /usr/sbin/batctl if add ra0" >> /etc/network/interfaces

    # # sudo echo -e "\n\nauto bat0 \niface bat0 inet static \naddress 192.168.123.3 \nnetmask 255.255.255.0 \ngateway 192.168.123.1 \npre-up /usr/sbin/batctl if add ra0" >> /etc/network/interfaces
    # sudo systemctl restart networking.service

    # sudo head -n -7 /etc/network/interfaces > /home/nvidia/temp.txt ; sudo mv /home/nvidia/temp.txt /etc/network/interfaces
}

run_batman &