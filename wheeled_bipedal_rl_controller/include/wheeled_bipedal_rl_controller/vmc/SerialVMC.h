//
// Created by lsy on 24-9-14.
//

#pragma once
#include <vector>
#include <string>
#include <stdexcept>
#include <iostream>
#include <complex>

namespace vmc{
class SerialVMC {
public:
  SerialVMC(double l1, double l2);
  ~SerialVMC() = default;

  void update(double phi1, double dphi1, double tau1, double phi2, double dphi2,
              double tau2);

  std::vector<double> getDesJointEff(double phi1, double phi2,double fR, double fT);

  std::vector<double> getDesJointPos(double desR, double desTheta);

  std::vector<double> getDesJointVel(double phi1, double phi2, double dVelR, double dVelTheta);

  void calculateVLEPos(double phi1, double phi2);

  void calculateVLEVel(double phi1, double dphi1, double phi2, double dphi2);

  void calculateVLEEff(double phi1, double tau1, double phi2, double tau2);

  std::vector<std::vector<double>> calculateJacobian(double phi1, double phi2);

  std::vector<std::vector<double>> calculateJacobianTranspose(const std::vector<std::vector<double>>& jacobian);

  std::vector<std::vector<double>>calculateInverseJacobian(const std::vector<std::vector<double>> &jacobian);

  void verifyInverse(const std::vector<std::vector<double>>& jacobian, const std::vector<std::vector<double>>& inverse);

public:
  double centre_offset_;
  double l1_, l2_;
  double phi1_, phi2_;

  double r_, theta_;
  double dr_, dtheta_;
  double Fr_, Ftheta_;
};
}// namespace vmc
