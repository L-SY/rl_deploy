//
// Created by lsy on 24-9-26.
//

#include "robot_common/utilities/MeanFilter.h"
#include <stdexcept>

MeanFilter::MeanFilter(int window_size) : window_size_(window_size), sum_(0) {
  if (window_size_ <= 0) {
    throw std::invalid_argument("窗口大小必须大于0");
  }
}

void MeanFilter::input(double value) {
  data_.push_back(value);
  sum_ += value;

  if (static_cast<int>(data_.size()) > window_size_) {
    sum_ -= data_.front();
    data_.erase(data_.begin());
  }
}

double MeanFilter::output() const {
  if (data_.empty()) {
    throw std::runtime_error("没有足够的数据进行滤波");
  }
  return sum_ / data_.size();
}

void MeanFilter::reset() {
  data_.clear();
  sum_ = 0;
}
