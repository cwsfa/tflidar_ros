#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <TFlidar.h>
#include <TFlidar_ros.h>
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;


TfMiniTest::TfMiniTest()
: Node("tflidar_ros_node")
{
  RCLCPP_INFO(this->get_logger(), "TF03 Node Started!!!");

  // param
  std::string id{"TFlidar"};
  this->declare_parameter("portName", std::string("/dev/ttyUSB0"));
  this->declare_parameter("baudRate", 115200);
  this->declare_parameter("model", std::string("TF03"));
  this->declare_parameter("topic_name", std::string("range_feedback"));
  std::string portName_ = this->get_parameter("portName").get_value<std::string>();
  int baudRate_ = this->get_parameter("baudRate").get_value<int>();
  std::string model_ = this->get_parameter("model").get_value<std::string>();
  std::string topic_name_ = this->get_parameter("topic_name").get_value<std::string>();

  // DRIVER OBJECT
  benewake::TFlidar tflidar_obj(portName_, baudRate_);

  // PUBLISHER
  const auto qos_profile =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  range_publisher_ = this->create_publisher<sensor_msgs::msg::Range>(
    topic_name_,
    qos_profile);

  // range msg
  sensor_msgs::msg::Range TFlidar_range;
  TFlidar_range.radiation_type = sensor_msgs::msg::Range::INFRARED;
  if(model_ == std::string("TFmini")) {
    RCLCPP_INFO(this->get_logger(), "Set For  TFmini...");
    TFlidar_range.field_of_view = 0.04;
    TFlidar_range.min_range = 0.3;
    TFlidar_range.max_range = 12;
  } else {
    RCLCPP_INFO(this->get_logger(), "Set For  TF03...");
    TFlidar_range.field_of_view = 0.00872665;
    TFlidar_range.min_range = 0.1;
    TFlidar_range.max_range = 100;
  }

  TFlidar_range.header.frame_id = id;
  float dist{0.0};

  RCLCPP_INFO(this->get_logger(), "Start processing TFlidar...");

  while(rclcpp::ok()) {
    dist = tflidar_obj.getDist();
    if (dist > 0.0 && dist < TFlidar_range.max_range) {
      TFlidar_range.range = dist;
    }
    else if (dist == 0.0) {
      continue;
    }

    TFlidar_range.header.stamp = rclcpp::Clock().now();
    range_publisher_->publish(TFlidar_range); // publish data

    if(dist == -1.0) {
      RCLCPP_INFO(this->get_logger(), "Failed to read data. TFlidar node stopped!");
      break;
    }
  }
  // close port
  tflidar_obj.closePort();
};

TfMiniTest::~TfMiniTest()
{
  RCLCPP_INFO(this->get_logger(), "Destroying");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  TfMiniTest tftest;
  auto node = std::make_shared<TfMiniTest>();
  rclcpp:spin(node);
  rclcpp::shutdown();
  return 0;
}
