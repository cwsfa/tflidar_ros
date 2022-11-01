
# Benewake mini, 03 infrared range sensor **TFlidar Plus** ROS2 package

## Package Information
- **Package Name**: `tflidar_ros`
- **Node Name**: `tflidar_ros_node`
- **TF Frame ID**: `TFlidar`
- **Published Topics**: `/tflidar_ros_node/range_feedback` (sensor_msgs/Range)
- **Note**: This node won't publish topic if no object exists within TFlidar's measurement range, and the behavior can be changed in file  `/src/TFlidar_ros_node.cpp`

## Quick Start
```bash
sudo apt-get install libboost-all-dev
cd ~/robot_ws/src
git clone https://github.com/cwsfa/tflidar_ros.git -b ros2
cd ~/robot_ws
colcon build --packages-select tflidar_ros
ros2 launch tflidar_ros tflidar.launch.py
```
## Set TFlidar model
1. open `tflidar.launch.py`
2. change parameters below
```python
    {'serial_port': "/dev/ttyUSB0"},
    {'baud_rate': 115200},
    {'model': "TF03"}
```

## Check TFlidar Data
```bash
ros2 topic echo /range_feedback
```
