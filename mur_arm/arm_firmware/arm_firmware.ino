#include <ros.h> // ROS header
#include <std_msgs/Bool.h> // Empty msg
#include <GRMI_actuator.h> // Import the library

// ROS infraestructure
ros::NodeHandle  nh;

// Create actuators
GRMI_actuator base(0);
GRMI_actuator gripper(1);

// Callbacks
void gripper_sub(const std_msgs::Bool& msg ) {
  if (msg.data) {
    nh.loginfo("Opening Gripper . . .");
    gripper.open(gripper.time_step);
  }
  else{
    nh.loginfo("Closing Gripper . . .");
    gripper.close(gripper.time_step);
  }
}


void base_sub(const std_msgs::Bool& msg ) {
  if (msg.data) {
    nh.loginfo("Opening Base . . .");
    base.open(base.time_step);
  }
  else{
    nh.loginfo("Closing Gripper . . .");
    base.close(base.time_step);
  }
}

// Servers definition
ros::Subscriber<std_msgs::Bool> gripper_msg("/arm/gripper", &gripper_sub);
ros::Subscriber<std_msgs::Bool> base_msg("/arm/base", &base_sub);

void setup() {
  // node Init
  nh.initNode();
  // Servers init
  nh.subscribe(gripper_msg);
  nh.subscribe(base_msg);
  // Actuators Init
  nh.loginfo("Arm Initialize . . .");
  gripper.time_step = 750;

}

void loop() {
  nh.spinOnce();
  delay(10);
}
