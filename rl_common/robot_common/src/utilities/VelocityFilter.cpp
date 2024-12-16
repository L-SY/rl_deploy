//
// Created by lsy on 24-9-29.
//

#include "robot_common/utilities/VelocityFilter.h"
#include <cmath>    // 用于 std::abs()
#include <stdexcept> // 用于抛出异常

VelocityFilter::VelocityFilter(double max_velocity)
  : max_velocity_(max_velocity), prev_value_(0.0), current_output_(0.0), is_initialized_(false) {}

void VelocityFilter::input(double value, double dt) {
  if (dt <= 0) {
    throw std::invalid_argument("时间间隔 dt 必须大于 0");
  }

  if (!is_initialized_) {
    prev_value_ = value;
    current_output_ = value;
    is_initialized_ = true;
    return;
  }

  double current_velocity = (value - prev_value_) / dt;

  if (std::abs(current_velocity) > max_velocity_) {
    current_output_ = prev_value_;
  } else {
    current_output_ = value;
    prev_value_ = value;
  }
}

double VelocityFilter::output() const {
  return current_output_;
}

void VelocityFilter::reset() {
  prev_value_ = 0.0;
  current_output_ = 0.0;
  is_initialized_ = false;
}

