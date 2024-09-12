//
// Created by lsy on 24-9-12.
//

#include <torch/script.h>
#include <iostream>
#include <memory>

int main() {
  std::string model_path = "/home/lsy/rl_ws/src/assets/diablo/models/policy_27.pt";

  torch::jit::script::Module model;
  try {
    model = torch::jit::load(model_path);
  } catch (const c10::Error& e) {
    std::cerr << "Error loading the model.\n";
    return -1;
  }

  std::cout << "Model loaded successfully.\n";

  torch::Tensor input_tensor = torch::rand({1, 27});
  std::cout << "input_tensor" << input_tensor.sizes() << std::endl;

  torch::Tensor output_tensor;
  try {
    output_tensor = model.forward({input_tensor}).toTensor();
  } catch (const c10::Error& e) {
    std::cerr << "Error during forward pass.\n";
    return -1;
  }

  std::cout << "Output tensor: " << output_tensor << std::endl;

  return 0;
}
