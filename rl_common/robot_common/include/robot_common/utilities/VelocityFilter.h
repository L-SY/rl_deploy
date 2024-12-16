//
// Created by lsy on 24-9-29.
//

#pragma once

class VelocityFilter {
public:
  explicit VelocityFilter(double max_velocity);

  void input(double value, double dt);

  double output() const;

  void reset();

private:
  double max_velocity_;
  double prev_value_;
  double current_output_;
  bool is_initialized_;
};


