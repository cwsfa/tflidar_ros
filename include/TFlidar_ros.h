#ifndef TFLIDAR_ROS__TFLIDAR_ROS_HPP_
#define TFLIDAR_ROS__TFLIDAR_ROS_HPP_

#include <functional>
#include <memory>
#include <valarray>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

class TFlidar: public rclcpp::Node {
public:
  TFlidar();
  ~TFlidar() override;

private:
  std::string portName_;
  std::string model_;
  int baudRate_;

  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_publisher_;
};
#endif