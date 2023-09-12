# u-blox F9P driver for ROS2

Driver focused on supporting single u-blox ZED-F9P (no sensor fusion) chip.
Currently, this driver supports `UBX-NAV-PVT` (navigation, position, velocity, time solution) protocol message.
It requires u-blox configuration to be done beforehand. The configuration can be done using the [OpenMower instructions](https://openmower.de/docs/robot-assembly/prepare-the-parts/prepare-the-gps/).
Tested only against FW version `1.32`.

The work is based on the [xbot_driver_gps](https://github.com/ClemensElflein/xbot_driver_gps/) originally used in the [OpenMower](https://openmower.de/) project.

## Features:

- **Low latency:** I want this to support the F9R with its internal sensor fusion. No point in having the data after
  some seconds.
    - :heavy_check_mark:The driver reads the header first and then the exact amount of bytes needed to process the next
      packet. These bytes are read and the packet is immediately processed and sent to ROS. This way, the latency is
      kept to a minimum.
- **:heavy_check_mark: RTCM support:** Sends RTCM from ROS to the u-blox
- **:wrench: Use the latest configuration protocol:** since we're only supporting the newer generations of u-blox
  chips (9+), we can use the new configuration protocol instead of the deprecated one.
- **:heavy_check_mark: Simple code base:** With less code, there are hopefully fewer errors
- **:heavy_check_mark: Robust:** The driver recovers quickly from lost bytes or invalid data

## TODO

- **:wrench: node diagnostic:** Update node diagnostic with more information about node state
- **:wrench: publish raw data:** Publish raw data from the GPS
- **:wrench: port IMU fusion:** The driver is able to send IMU feedback to the F9R for the internal sensor fusion to
  work.
- **:wrench: port odometry fusion:** The driver is able to send odometry feedback to the F9R for the internal sensor fusion to
  work.

## Parameters

- **/port (string):** The serial port to use, defaults to /dev/ttyACM0
- **/baudrate (int):** The baudrate to use, defaults to 921600
- **/config (bool):** Enable F9P configuration via `UBX-CFG-VALSET` message, defaults to false. Required to be set to true to make configuration options below work.
  -  **/config.measurement_rate (uint16):** Nominal frequency between GNSS measurements, defaults to 5, maximum 40
  -  **/config.uart_output_rate (uint8):** Rate of UBX_NAV_PVT measurements to arrive at UART1, defaults to 5

## Subscribed Topics:

- **/rtcm (rtcm_msgs/Message)** RTCM which will be sent to the GPS
- TODO: **/odometry (nav_msgs/Odometry) ** odometry messages which will be sent to the GPS for sensor fusion (e.g. F9R)

## Published Topics:

- **/gps/fix (sensor_msgs/NavSatFix):** The GPS fix
