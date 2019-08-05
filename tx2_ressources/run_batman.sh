#!/bin/bash
### BEGIN INIT INFO
# Provides:          ra0 setup for batman
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: Sets up the ra0 and bat0 interfaces.
# Description:       This file should be used to construct scripts to be
#                    placed in /etc/init.d.  This example start a
#                    single forking daemon capable of writing a pid
#                    file.  To get other behavoirs, implemend
#                    do_start(), do_stop() or other functions to
#                    override the defaults in /lib/init/init-d-script.
### END INIT INFO

run_batman()
{
    sleep 30
    iw dev ra0 set type ibss
    iw dev ra0 ibss join NVIDIA 5220
    batctl if add ra0
    ifconfig bat0 192.168.10.150 netmask 255.255.255.0 up
}

run_batman &