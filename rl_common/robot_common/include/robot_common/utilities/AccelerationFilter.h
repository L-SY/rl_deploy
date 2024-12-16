//
// Created by lsy on 24-9-29.
//

#pragma once

class AccelerationFilter {
public:
  explicit AccelerationFilter(double max_acceleration);

  void input(double value, double dt);

  double output() const;

  void reset();

private:
  double max_acceleration_;    // 最大允许的加速度阈值
  double prev_value_;          // 上一个值
  double prev_velocity_;       // 上一个速度（即两个值之间的差）
  double current_output_;
  bool is_initialized_;        // 检查是否有初始值
};


