#include <ros.h> // ROS header
#include <std_msgs/Bool.h> // Empty msg
#include <std_msgs/String.h> // Status msg
#include <GRMI_actuator.h> // Import the library

// ROS infraestructure
ros::NodeHandle  nh;
std_msgs::String status_msg;
ros::Publisher pub_status("/arm/status", &status_msg);

// Create actuators
GRMI_actuator base(0);
GRMI_actuator gripper(1);

// Callbacks
void gripper_sub(const std_msgs::Bool& msg ) {
  if (msg.data) {
    gripper.time_step = 2000;
    status_msg.data = "G:Open";
    pub_status.publish(&status_msg);
    gripper.open(gripper.time_step);
    status_msg.data = "Done";
    pub_status.publish(&status_msg);
  }
  else {
    gripper.time_step = 2000;
    status_msg.data = "G:Close";
    pub_status.publish(&status_msg);
    gripper.close(gripper.time_step);
    status_msg.data = "Done";
    pub_status.publish(&status_msg);
  }
}


void base_sub(const std_msgs::Bool& msg ) {
  if (msg.data) {
    base.time_step = 1200;
    status_msg.data = "B:Open";
    pub_status.publish(&status_msg);
    base.open(base.time_step);
    status_msg.data = "Done";
    pub_status.publish(&status_msg);
  }
  else {
    base.time_step = 1500;
    status_msg.data = "B:Close";
    pub_status.publish(&status_msg);
    base.close(base.time_step);
    status_msg.data = "Done.";
    pub_status.publish(&status_msg);
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
  nh.advertise(pub_status);
  // Actuators Init
  status_msg.data = "Arm Init";
  pub_status.publish(&status_msg);
}

void loop() {
  nh.spinOnce();
  delay(10);
}
