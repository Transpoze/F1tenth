## How to use ZED
ZED is claimed to be the fastest depth camera in terms of framerate (1080p 30fps, WVGA 100fps) and it's the first sensor
to introduce indoor and outdoor long range depth perception (0.5-20m, 110deg max).
Here is the official [documentation](https://www.stereolabs.com/documentation/overview/getting-started/introduction.html).

### Installation on Jetson TK1
Please follow the instruction [here](https://www.stereolabs.com/blog/index.php/2015/09/24/getting-started-with-jetson-tk1-and-zed/).
Some important notes:
- You need a Linux host computer to install JetPack and setup TK1 for ZED
- Always try to use USB 3.0, even if the official documentation claims that USB 2.0 works
- If you have to flash TK1 (usually this is only for installing the OS on TK1), you can check [this video](https://devtalk.nvidia.com/default/topic/1001763/jetson-tx2/error-jetpack-must-be-run-on-x86_64-host-platform-detected-aarch64-platform-/) for reference.
- The [lastest version of ZED SDK for TK1](https://www.stereolabs.com/developers/release/1.2/) is 1.2 and it stops updating.
- TK1 only works with CUDA 6.5
- Default OpenCV installed by JetPack is 2.7
- You have to use c++ to develop applications with ZED

### Retrieving Data from ZED
#### For local application
Use [ZED API](https://www.stereolabs.com/developers/documentation/API/index.html).
#### For ROS
Use ROS package zed-ros-wrapper. Check on [ros.org](http://wiki.ros.org/zed-ros-wrapper) or [Github](https://github.com/stereolabs/zed-ros-wrapper) 

## Troubleshooting
### ZED
- Problem:

Test ZED with ZED Explorer after installing everything, but it shows nothing, even though the camera can be recognized (you can tell by checking if it recognizes the serial number on the up-right corner)
- Solution:

With reference to [this github issue](https://github.com/stereolabs/zed-ros-wrapper/issues/28)
1. check if there is conf file in /usr/local/zed/settings, the filename should be SN****.conf. If not, do:
/usr/local/zed/tools/ZED\ Explorer --dc # or --download_calibration
2. check if the computer recognize the USB port:
lsusb | grep 2b03:f580 | wc -l
should return 1 if ZED is detected
3. make sure ZED is connected by USB3.0, and change the usb_port_owner = 2 via:
sudo vi /boot/etlinux/extlinux.conf
4. make sure Jetson TK1 is properly setup (JetPack, CUDA and ZED SDK), check out [this instruction](
https://www.stereolabs.com/blog/index.php/2015/09/24/getting-started-with-jetson-tk1-and-zed/)
