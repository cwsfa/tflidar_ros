
# Benewake mini, 03 infrared range sensor **TFlidar Plus** ROS package

## Package Information
- **Package Name**: tflidar_ros
- **Node Name**: tflidar_ros_node
- **TF Frame ID**: TFlidar
- **Published Topics**: /tflidar_ros_node/range (sensor_msgs/Range)
- **Note**: This node won't publish topic if no object exists within TFlidar's measurement range, and the behavior can be changed in file  /src/TFlidar_ros_node.cpp


## Quick Start
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/cwsfa/tflidar_ros.git
$ cd ..
$ catkin_make
$ roslaunch tflidar_ros tflidar.launch
```
## Set TFlidar model
```
1. open tflidar.launch
2. <param name="model" type="string" value="TF03" />
3. change value(TFmini, TF03)
```

## Check TFlidar Data
```
$ rostopic echo /tflidar_ros_node/range
```
