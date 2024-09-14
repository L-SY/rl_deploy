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

class BaseVMC {
public:
  BaseVMC(double l1, double l2) : l1_(l1), l2_(l2) {}
  virtual ~BaseVMC() = default;

  virtual void update(double phi1, double dphi1, double tau1, double phi2, double dphi2,
                      double tau2) = 0;

  virtual std::vector<double> getDesJointEff(double phi1, double phi2, double fR, double fT) = 0;
  virtual std::vector<double> getDesJointPos(double desR, double desTheta) = 0;
  virtual std::vector<double> getDesJointVel(double phi1, double phi2, double dVelR, double dVelTheta) = 0;

  virtual void calculateVLEPos(double phi1, double phi2) = 0;
  virtual void calculateVLEVel(double phi1, double dphi1, double phi2, double dphi2) = 0;
  virtual void calculateVLEEff(double phi1, double tau1, double phi2, double tau2) = 0;

  virtual std::vector<std::vector<double>> calculateJacobian(double phi1, double phi2) = 0;
  virtual std::vector<std::vector<double>> calculateJacobianTranspose(const std::vector<std::vector<double>>& jacobian) = 0;
  virtual std::vector<std::vector<double>> calculateInverseJacobian(const std::vector<std::vector<double>>& jacobian) = 0;
  virtual void verifyInverse(const std::vector<std::vector<double>>& jacobian, const std::vector<std::vector<double>>& inverse) = 0;

protected:
  double l1_, l2_;
};

}// namespace vmc

