#include "ps5_joy/ps5Joy.h"

using namespace joy;

PS5Joy::PS5Joy(ros::NodeHandle &nh) {
  joy_sub_ = nh.subscribe("/joy", 10, &PS5Joy::joyCallback, this);
  buttons_.fill(false);
  axes_.fill(0.0);
}

bool PS5Joy::getButtonState(PS5ButtonMap::Button button) const {
  return buttons_[button];
}

double PS5Joy::getAxisValue(PS5ButtonMap::Axis axis) const {
  return axes_[axis];
}

void PS5Joy::joyCallback(const sensor_msgs::JoyConstPtr &msg) {
  for (size_t i = 0; i < buttons_.size(); ++i) {
    buttons_[i] = msg->buttons[i] == 1; // Update button states
  }

  for (size_t i = 0; i < axes_.size(); ++i) {
    axes_[i] = msg->axes[i]; // Update axis values
  }
}
