# RM520N-ROS2

RM520N(Plus)'s Examples for ROS2.

/ Version 1.0


## Package Information

- Package:	rm520n_multi_driver
- Node:		rm520n_multi_driver_node
 
Published Topics:
```

```

## Quick Start

Launch the node (defualt serial port is `/dev/ttyUSB0` for UART on the UP2 pin header):
```
ros2 launch rm520n_multi_driver rm520n_multi_driver.launch.py
```
```
ros2 launch rm520n_multi_driver rm520n_multi_driver.launch.py \
  params_file:=/path/to/your_params.yaml
```


## Set up serial Devices rules

# RM520N devices(ttyUSB6 →  symlink name: rm520n) 
```
KERNEL=="ttyUSB*", KERNELS=="2-3.1:1.3", SYMLINK+="rm520n", GROUP="dialout", MODE="0660"
```
# RM520N-gps devices(ttyUSB4 →  symlink name: rm520n-gps) 
```
KERNEL=="ttyUSB*", KERNELS=="2-3.1:1.1", SYMLINK+="gps", GROUP="dialout", MODE="0660"
```
