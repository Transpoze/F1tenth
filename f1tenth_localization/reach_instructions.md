## Setup reach RTK kit

### get gps data as a ros ropic
The ros node `gps_node.py` reads serial data from the reach module and publishes a standard [NavSatFix](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html) mseeage to `gps/navsat` and a custom [gps_reach](https://github.com/KTH-SML/SML_summerproject/blob/master/f1tenth_localization/msg/gps_reach.msg) message following the LLH standard to `gps/reach`. To run it:


- Make sure the Reach module is connected to the PC that runs the node via USB
- Run `rosrun f1tenth_localization gps_node.py` in a terminal. If the antenna has an unobsructed sky view you should start to see gps measurements being printed. 
- The default port of the reach is `/dev/ttyACM0` but some times this port willl be occupied (for example by the Teensy board). In that case simply use `rosrun f1tenth_localization gps_node.py _port:=/dev/ttyACM1` to set the corresponding ros-parameter.


### Configuring reach settings
All configuration of the reach module is done through the [reachView app](https://docs.emlid.com/reach/common/reachview/). To access the app:

- Make sure the wireless router is turned on 
- Power the reach and wait for the LED to blink blue and green
- connect your phone/laptop to `TP-LINK_EBE0` with password `20906577`
- go to `192.168.0.144` 


- If the LED blinks white and green the reach module was unable to connect to the Wi-Fi network and has gone into hotspot mode. Make sure that the network is active and then either try resetting the reach or:
    - connect to the WiFi network `reach:82:4c` with password `emlidreach` 
    - go to `192.168.42.1`
    - go to `Wi-Fi/Bluetooth` 
    - click`TP-LINK_EBE0` in the list of Wi-Fi networks and then `connect`


### Base correction
For better positioning we subscribe to NTRIP data from Lantmäteriet's SWEPOS service. Use the following settings to connect 

![correction input](https://github.com/KTH-SML/SML_summerproject/blob/master/f1tenth_localization/ntripsettings.jpg)

ask Jonas Mårtensson for username/password if necessary.

### Localization quality
The quality of the gps measurements are heavily dependent on the satellite quality. For centimeter accuracy at least 3 satellites with >40% SNR seems to be required. With lower signal quality it is still usually possible go get fairly accurate relative position measurements, but the estimate might drift significantly (~1 meter drift in one minute is fairly typical in these cases). In these cases it is often necessary to reset the `ekf_localization_node` before running any control algorithms that rely on accurate positioning. 

If the position estimate seems to get stuck in the wrong location it some times help to move the robot around or to cover the antenna for a couple of seconds and then uncover it to reset the estimation. 

We have found the following RTK-settings to work well, but feel free to play around.

![RTK settings](https://github.com/KTH-SML/SML_summerproject/blob/master/f1tenth_localization/rtksettings.jpg)

