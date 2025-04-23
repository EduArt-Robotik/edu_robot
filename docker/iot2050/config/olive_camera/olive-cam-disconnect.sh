#! /bin/sh
ip link set eno2 nomaster
ip link delete br0 type bridge
dhclient eno2

