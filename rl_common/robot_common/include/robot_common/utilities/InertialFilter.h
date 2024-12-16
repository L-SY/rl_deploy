//
// Created by lsy on 24-9-26.
//

#pragma once

class InertiaFilter {
public:
  explicit InertiaFilter(double alpha);
  void input(double in);
  double output() const;
  void reset();

private:
  double alpha_;
  double last_in_;
  double output_;
};


