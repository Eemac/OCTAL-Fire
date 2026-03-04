# OlinBAD
A BADass Firefighting Drone


## commands for power up CANable:
- sudo slcand -f -o -s6 -S 115200 -t hw /dev/ttyACMX can0
- sudo ip link set can0 up
- candump can0
- cansniffer can0
- cansend can0 
