sudo brctl addbr br0
sudo brctl addif br0 eth0
sudo brctl addif br0 eth1
sudo ifconfig br0 192.168.1.2 netmask 255.255.255.0 up
nmap -sP 192.168.1.0/24
