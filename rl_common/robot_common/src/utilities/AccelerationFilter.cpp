//
// Created by lsy on 24-9-29.
//

#include "robot_common/utilities/AccelerationFilter.h"
#include <cmath> // 用于 std::abs()
#include <stdexcept>

AccelerationFilter::AccelerationFilter(double max_acceleration)
  : max_acceleration_(max_acceleration), prev_value_(0.0), prev_velocity_(0.0), current_output_(0.0), is_initialized_(false) {}

void AccelerationFilter::input(double value, double dt) {
  if (dt <= 0) {
    throw std::invalid_argument("时间间隔 dt 必须大于 0");
  }

  if (!is_initialized_) {
    prev_value_ = value;
    prev_velocity_ = 0.0;
    current_output_ = value;
    is_initialized_ = true;
    return;
  }

  double current_velocity = (value - prev_value_) / dt;
  double acceleration = (current_velocity - prev_velocity_) / dt;

  if (std::abs(acceleration) > max_acceleration_) {
    current_output_ = prev_value_;
  } else {
    current_output_ = value;
    prev_velocity_ = current_velocity;
    prev_value_ = value;
  }
}

double AccelerationFilter::output() const {
  return current_output_;
}

void AccelerationFilter::reset() {
  prev_value_ = 0.0;
  prev_velocity_ = 0.0;
  current_output_ = 0.0;
  is_initialized_ = false;
}

