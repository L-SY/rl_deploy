//
// Created by lsy on 24-9-26.
//

#include "robot_common/utilities/InertialFilter.h"

InertiaFilter::InertiaFilter(double alpha) : alpha_(alpha), last_in_(0), output_(0) {}

void InertiaFilter::input(double in) {
  output_ = alpha_ * in + (1 - alpha_) * last_in_;
  last_in_ = in;
}

double InertiaFilter::output() const {
  return output_;
}

void InertiaFilter::reset() {
  output_ = 0;
}
