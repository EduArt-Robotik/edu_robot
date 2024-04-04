#! /bin/sh
ip addr flush eno2
ip addr flush enxa8f2fb1fa7d4
ip link add br0 type bridge
ip link set dev br0 up
sysctl net.bridge.bridge-nf-call-iptables=0
ip link set eno2 master br0
ip link set enxa8f2fb1fa7d4 master br0
ip addr add 192.168.0.100/24 dev br0

