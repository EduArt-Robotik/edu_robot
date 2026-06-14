#! /bin/sh
OLIVE_CAM_INTERFACE=enxa8f2fb1fa7d4
ROBOT_IP_ADDRESS=192.168.0.100/24

ip addr flush eno2
ip addr flush $OLIVE_CAM_INTERFACE
ip link add br0 type bridge
ip link set dev br0 up
sysctl net.bridge.bridge-nf-call-iptables=0
ip link set eno2 master br0
ip link set $OLIVE_CAM_INTERFACE master br0
ip addr add $ROBOT_IP_ADDRESS dev br0

