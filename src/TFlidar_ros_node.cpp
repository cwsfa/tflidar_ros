#include <TFlidar.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "tflidar_ros_node");
  ros::NodeHandle nh("~");
  std::string id = "TFlidar"; // TF Frame id
  std::string topic_name = "range"; // ROS Topic name
  std::string portName;
  std::string model;
  int baud_rate;
  benewake::TFlidar *tflidar_obj;

  nh.param("serial_port", portName, std::string("/dev/ttyUSB0"));
  nh.param("baud_rate", baud_rate, 115200);
  nh.param("model", model, std::string("TF03"));

  // init TFlidar variable & ros publisher
  tflidar_obj = new benewake::TFlidar(portName, baud_rate); // create TFlidar object
  ros::Publisher pub_range = nh.advertise<sensor_msgs::Range>(topic_name, 1000, true);

  // sensor msgs init
  sensor_msgs::Range TFlidar_range;
  TFlidar_range.radiation_type = sensor_msgs::Range::INFRARED;
  if(model == std::string("TFmini")) {
    ROS_INFO_STREAM("Set For  TFmini...");
    TFlidar_range.field_of_view = 0.04;
    TFlidar_range.min_range = 0.3;
    TFlidar_range.max_range = 12;
  } else {
    ROS_INFO_STREAM("Set For TF03...");
    TFlidar_range.field_of_view = 0.00872665;
    TFlidar_range.min_range = 0.1;
    TFlidar_range.max_range = 30;
  }

  TFlidar_range.header.frame_id = id;
  float dist = 0;

  ROS_INFO_STREAM("Start processing TFlidar...");

  while(ros::master::check() && ros::ok()){
    ros::spinOnce();
    dist = tflidar_obj->getDist();
    if(dist > 0 && dist < TFlidar_range.max_range) {
      TFlidar_range.range = dist;
    }
    else if(dist == 0.0) {
      TFlidar_range.range = TFlidar_range.max_range;
    }
    TFlidar_range.header.stamp = ros::Time::now();
    pub_range.publish(TFlidar_range); // publish data

    if(dist == -1.0) {
      ROS_ERROR_STREAM("Failed to read data. TFlidar node stopped!");
      break;
    }
  }

  tflidar_obj->closePort();
}
