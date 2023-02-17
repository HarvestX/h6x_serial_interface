# h6x_serial_interface
HarvestX Serial Device Interface Package.

## Repository Status

| ROS2 Distro  | Build status                                                                                                                                                                                       |
| ------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **galactic** | [![ci_galactic](https://github.com/HarvestX/h6x_serial_interface/actions/workflows/ci_galactic.yml/badge.svg?branch=galactic)](https://github.com/HarvestX/h6x_serial_interface/actions/workflows/ci_galactic.yml) |
| **humble**   | [![ci_humble](https://github.com/HarvestX/h6x_serial_interface/actions/workflows/ci_humble.yml/badge.svg?branch=humble)](https://github.com/HarvestX/h6x_serial_interface/actions/workflows/ci_humble.yml)       |



## Requirements

- ROS 2
  - Ubuntu20 : [Galactic Geochelone](https://docs.ros.org/en/galactic/Installation.html)
  - Ubuntu22 : [Humble Hawksbill](https://docs.ros.org/en/humble/Installation.html)


## Install
### Locate package in workspace

```bash
mkdir -p ~/ws_ros2/src
cd ~/ws_ros2/src
git clone git@github.com:HarvestX/h6x_serial_interface.git
rosdep install -r -y -i --from-paths . --rosdistro $ROS_DISTRO
```

## Build Source

Open new terminal and type followings.

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/ws_ros2
colcon build
```



## Run simple example
#### Read packet from the device
- `dev` : Path of the device
- `baudrate` : Baudrate

```bash
ros2 run h6x_serial_interface_example simple_read_node_exec \
  --ros-args \
    -p dev:=/dev/ttyUSB0 -p baudrate:=115200
```

#### Write packet
- `dev` : Path of the device
- `baudrate` : Baudrate

The sample send `Hello World` as ASCII string.
```bash
ros2 run h6x_serial_interface_example simple_write_node_exec \
  --ros-args \
    -p dev:=/dev/ttyUSB0 -p baudrate:=115200
```


## Create own interface

You can implement your own serial interface by using `port_handler`.
```cpp
// Header
#include <h6x_serial_interface/h6x_serial_interface.hpp>

// - Source code --------------------------------------------------------
  std::string dev = "/dev/ttyUSB0";
  int baudrate = 115200;
  const auto port_handler = std::make_unique<PortHandler>(dev, baudrate);

  // Open port
  port_handler->openPort();

  // Get available bytes
  ssize_t available_bytes = port_handler->getBytesAvailable();

  // Read serial packet into read_buf
  char read_buf[128];
  port_handler->readPort(read_buf, sizeof(read_buf));

  // Write serial packet
  char write_buf[] = "Hello World";
  port_handler->writePort(write_buf, strlen(write_buf))

  // Close port
  port_handler->closePort();
// ----------------------------------------------------------------------
```
