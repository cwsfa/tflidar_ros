#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <TFlidar.h>
#include <TFlidar_ros.h>
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;


TFlidar::TFlidar()
: Node("tflidar_ros_node")
{
  RCLCPP_INFO(this->get_logger(), "TFlidar ROS Node Started!!!");

  // declare parameters
  this->declare_parameter("model", "TF03");
  this->declare_parameter("serial_port", "/dev/ttyUSB0");
  this->declare_parameter("baud_rate", 115200);
  this->declare_parameter("topic_name", "range_feedback");
  this->declare_parameter("frame_link", "TFlidar");

  std::string modelName = this->get_parameter("model").get_value<std::string>();
  std::string portName = this->get_parameter("serial_port").get_value<std::string>();
  int baudRate = this->get_parameter("baud_rate").get_value<int>();
  std::string topicName = this->get_parameter("topic_name").get_value<std::string>();
  std::string frameId = this->get_parameter("frame_link").get_value<std::string>();

  RCLCPP_INFO(this->get_logger(),"Device model: %s", modelName.c_str());
  RCLCPP_INFO(this->get_logger(),"Serial port: %s", portName.c_str());
  RCLCPP_INFO(this->get_logger(),"Baud rate: %d", baudRate);
  RCLCPP_INFO(this->get_logger(),"Published topic: %s", topicName.c_str());
  RCLCPP_INFO(this->get_logger(),"Published frame id: %s", frameId.c_str());

  // DRIVER OBJECT
  tflidar_obj = new benewake::TFlidar(portName, baudRate); // create TFlidar object

  // PUBLISHER
  const auto qos_profile =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  range_publisher_ = this->create_publisher<sensor_msgs::msg::Range>(
    topicName,
    qos_profile);

  // range msg
  TFlidar_range.header.frame_id = frameId;
  TFlidar_range.radiation_type = sensor_msgs::msg::Range::INFRARED;
  if(modelName == std::string("TFmini")) {
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

  // main process
  RCLCPP_INFO(this->get_logger(), "Start processing TFlidar...");
  while(rclcpp::ok()) {
    distance = tflidar_obj->getDist();
    if(distance == -1.0) {
      RCLCPP_WARN(this->get_logger(), "Failed to read data. TFlidar node stopped!");
      break;
    }
    if (distance > 0.0 && distance < TFlidar_range.max_range) {
      TFlidar_range.range = distance;
    }
    else {
      continue;
    }
    TFlidar_range.header.stamp = rclcpp::Clock().now();
    range_publisher_->publish(TFlidar_range); // publish data
  }
  tflidar_obj->closePort();  // close port
};

TFlidar::~TFlidar()
{
  RCLCPP_WARN(this->get_logger(), "Destroying");
  delete[] tflidar_obj;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TFlidar>();
  rclcpp:spin(node);
  rclcpp::shutdown();
  return 0;
}
