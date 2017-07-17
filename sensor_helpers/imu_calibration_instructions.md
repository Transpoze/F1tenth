## Imu calibration

The Imu must be calibrated in order to work properly (otherwise it will report useless measurements at certain angles). Before you start, make sure you have Arduino installed and set `Tools>Serial Port` to `/dev/ttyUSB0` and set `Tools>board` to `Arduino Pro or Pro mini (3.3V, 8MHz)w/ ATmega 328`.

#### Check the current calibration parameters:

1. open Arduino
2. open `serial monitor` and set it to 57600 baud
3. enter #p and click `send`

The current calibration parameters should be printed in the serial output. 

#### set new calibration parametres:

1. Perform the calibration steps outlined on the [ros wiki](http://wiki.ros.org/razor_imu_9dof) (section 7.1) and write down the results.
    * NOTE: instead of using the extended calibration mode for the magnetometer (with Processing etc) that is described on the website it seems to be enough to use the standard calibration procedure. 
    * This is done in the same way as the accelerometer calibration (except you send `#on` once while in accelerometer calibration mode to see the max/min values for the magnetometer). 
2. Open `SML_summerproject/sensor_helpers/Razor_AHRS/Razor_AHRS.ino` using Arduino
3. Scroll down to USER SETUP AREA and change the values to match the new calibration
4. Click `Verify` and then `upload`


Once the parameters are set you can use `rosrun sensor_helpers razor_imu.py` to publish the imu data as a ros topic and check that the correct calibration parameters were saved correctly. 

NOTE: The standard `razor_imu_9dof/imu_node.py` node can also be used to set the calibration parameters using a `.yaml` file. But we have had problems with some values being corrupted when doing this, so using the above is preferable.
