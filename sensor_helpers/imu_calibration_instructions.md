## Imu calibration

The Imu must be calibrated in order to work properly (otherwise it will report useless measurements at certain angles). Before you start, make sure you have Arduino installed and set `Tools>Serial Port` to `/dev/ttyUSB0` and set `Tools>board` to `Arduino Pro or Pro mini (3.3V, 8MHz)w/ ATmega 328`.

#### Check the current calibration parameters:

1. open Arduino
2. open `serial monitor` and set it to 57600 baud
3. enter #p and click `send`

The current calibration parameters should be printed in the serial output. 

#### set new calibration parametres:

1. Perform the calibration steps outlined on the [ros wiki](http://wiki.ros.org/razor_imu_9dof) (section 7.1) and write down the results.
    * NOTE: We have found that the extended version of the magnetometer calibration is necessary for good orientation measurements (see below for more info on how to perform this on the Jetson) 
2. Open `SML_summerproject/sensor_helpers/Razor_AHRS/Razor_AHRS.ino` using Arduino
3. Scroll down to USER SETUP AREA and change the values to match the new calibration
4. Click `Verify` and then `upload`


Once the parameters are set you can use `rosrun sensor_helpers razor_imu.py` to publish the imu data as a ros topic and check that the correct calibration parameters were saved correctly. 

NOTE: The standard `razor_imu_9dof/imu_node.py` node can also be used to set the calibration parameters using a `.yaml` file. But we have had problems with some values being corrupted when doing this, so using the above is preferable.

#### Extended magnetometer calibration:
Perform the following steps to calibrate the IMU magnetometer. We reccomend performing these steps on a laptop connected to the IMU with a long cable (as Processing can be very slow on the Jetson). See the [official guide](http://wiki.ros.org/razor_imu_9dof) section 7.1.3 and the comments in `Magnetometer_calibration.pde` for more information and pictures.
1. Before calibrating, make sure the magnetometer is mounted in the magnetic environment it will later be used in. This is very important since any electronics can drastically change the magnetic field. 
   * To see this in behaviour try running `rosrun sensor_helpers print_imu_yaw.py` while the `razor_imu.py` node is active, then try moving some equipment (like a laptop) closer to the imu and see how the readings change.
2. Download and extract Processing from https://processing.org/download/ (only confirmed to work with  verison 3.3.5, but later versions should be fine)
3. Copy `magnetometer_calibration/EJML.jar` from this repository to `sketchbook/libraries/EJML/library/` (the `sketchbook/libraries` folder should already be in your home directory as long as the arduino IDE is installed)
4. Open `magnetometer_calibration/Magnetometer_calibration.pde` with Processing and press `run` (a black window with blue dots should appear)
5. Turn the IMU (and whatever its mounted on!) arround until all directions are covered 
6. Press space to quit and copy the values printed in the terminal to `Razor_AHRS.ino` as described in the last section.
