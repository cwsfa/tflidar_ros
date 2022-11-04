
# Benewake mini, 03 infrared range sensor **TFlidar Plus** ROS2 package

## Package Information
- **Package Name**: `tflidar_ros`
- **Node Name**: `tflidar_ros_node`
- **TF Frame ID**: `TFlidar`
- **Published Topics**: `/range_feedback` (sensor_msgs/Range)
- **Note**: This node won't publish topic if no object exists within TFlidar's measurement range, and the behavior can be changed in file  `/src/TFlidar_ros_node.cpp`

## Quick Start
```bash
sudo apt-get install libboost-all-dev
cd ~/robot_ws/src
git clone https://github.com/cwsfa/tflidar_ros.git -b ros2
cd ~/robot_ws
colcon build --packages-select tflidar_ros
ros2 launch tflidar_ros tflidar.launch.py
# enable permission with the following command
sudo chmod 777 /dev/ttyUSB*
```
## Set TFlidar model
1. open `tflidar.launch.py`
2. change parameters below
    ```python
    model = LaunchConfiguration('model', default="TF03")
    serial_port = LaunchConfiguration('serial_port', default="/dev/ttyUSB0")
    baud_rate = LaunchConfiguration('baud_rate', default=115200)
    topic_name = LaunchConfiguration('topic_name', default="range_feedback")
    frame_link = LaunchConfiguration('frame_link', default="TFlidar")
    ```
    or you can achieve the same from running from CLI
    ```bash
    ros2 launch tflidar_ros tflidar.launch.py model:=TF03 serial_port:=/dev/ttyUSB0 frame_link:=TFlidar
    ```


## Check TFlidar Data
```bash
ros2 topic echo /range_feedback
```
