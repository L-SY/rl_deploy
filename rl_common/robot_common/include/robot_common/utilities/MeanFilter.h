//
// Created by lsy on 24-9-26.
//

#pragma once

#include <vector>

class MeanFilter {
public:
  explicit MeanFilter(int window_size);  // 构造函数，指定窗口大小
  void input(double value);              // 输入新的数据值
  double output() const;                 // 获取当前的均值输出
  void reset();                          // 重置滤波器

private:
  int window_size_;                      // 滑动窗口的大小
  std::vector<double> data_;             // 存储当前窗口的数据
  double sum_;                           // 存储当前窗口数据的总和
};


