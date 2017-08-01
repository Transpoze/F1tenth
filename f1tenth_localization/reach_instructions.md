## Setup reach RTK kit

### get gps data as a ros ropic
The ros node `gps_node.py` reads serial data from the reach module and publishes the data to two topics:
- `gps/navsat` - a standard [NavSatFix](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html) message
- `gps/reach` - a custom message containing all the information provided by the reach (including solution status) 


- Make sure the Reach module is connected to the PC that runs the node via USB
- Ron `rosrun f1tenth_localization gps_node.py` in a terminal. If the antenna has an unobstructede sky view you should start seeing gps measurements being printed. 
- The default port of the reach is `/dev/ttyACM0` but some times this port willl be occupied (for example by the Teensy board). In that case simply use `rosrun f1tenth_localization gps_node.py _port:=/dev/ttyACM1` to set the corresponding ros-parameter.


### Configuring reach settings
All configuration of the reach module is done through the [reachView app](https://docs.emlid.com/reach/common/reachview/). To access the app:

- Make sure the wireless router is turned on 
- Power the reach and wait for the LED to blink blue and green
- connect your phone/laptop to `TP-LINK_EBE0` with password `20906577`
- go to `192.168.0.144` 

### Troubleshooting
- If the LED blinks white and green the reach module was unable to connect to the Wi-Fi network and has gone into hotspot mode. Make sure that the network is active and then either try resetting the reach or:
    - connect to the WiFi network `reach:14:4c` with password `emlidreach` 
    - go to `Wi-Fi/Bluetooth` 
    - click `TP-LINK_EBE0` and then `connect`


### Base correction
The reach module can subscribe to NTRIP data provided by the Swedish land-surveying company *Lantmäteriet* in order to get more accurate position measurements
