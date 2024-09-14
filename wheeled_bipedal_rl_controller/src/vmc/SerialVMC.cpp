//
// Created by lsy on 24-9-14.
//

#include "wheeled_bipedal_rl_controller/vmc/SerialVMC.h"

namespace vmc
{
SerialVMC::SerialVMC(double l1, double l2)
  : l1_(l1), l2_(l2), r_(0), theta_(0), dr_(0), dtheta_(0), Fr_(0), Ftheta_(0)
{
  if (l1 <= 0 || l2 <= 0)
  {
    throw std::invalid_argument("Lengths of the links must be positive.");
  }
  std::cout << "l1 =  " << l1 << ", l2 =  " << l2 << std::endl;
}

void SerialVMC::calculateVLEPos(double phi1, double phi2)
{
  double x = -l1_ * sin(phi1) - l2_ * sin(phi1 + phi2);
  double y = -l1_ * cos(phi1) - l2_ * cos(phi1 + phi2);
  r_ = std::sqrt(x * x + y * y);
  theta_ = -(std::atan2(y, x));
}

std::vector<std::vector<double>> SerialVMC::calculateJacobian(double phi1, double phi2) {
  double x = -l1_ * sin(phi1) - l2_ * sin(phi1 + phi2);
  double y = -l1_ * cos(phi1) - l2_ * cos(phi1 + phi2);
  double r = std::sqrt(x * x + y * y);

  double dx_dphi1 = -l1_ * cos(phi1) - l2_ * cos(phi1 + phi2);
  double dx_dphi2 = -l2_ * cos(phi1 + phi2);
  double dy_dphi1 = l1_ * sin(phi1) + l2_ * sin(phi1 + phi2);
  double dy_dphi2 = l2_ * sin(phi1 + phi2);

  double dr_dphi1 = (dx_dphi1 * x + dy_dphi1 * y) / r;
  double dr_dphi2 = (dx_dphi2 * x + dy_dphi2 * y) / r;
  double dtheta_dphi1 = -(dy_dphi1 * x - dx_dphi1 * y) / (r * r);
  double dtheta_dphi2 = -(dy_dphi2 * x - dx_dphi2 * y) / (r * r);

  std::vector<std::vector<double>> jacobian = {
    {dr_dphi1, dr_dphi2},
    {dtheta_dphi1, dtheta_dphi2}
  };

  return jacobian;
}

std::vector<std::vector<double>> SerialVMC::calculateJacobianTranspose(const std::vector<std::vector<double>>& jacobian) {
  std::vector<std::vector<double>> jacobianTranspose(2, std::vector<double>(2));
  jacobianTranspose[0][0] = jacobian[0][0];
  jacobianTranspose[0][1] = jacobian[1][0];
  jacobianTranspose[1][0] = jacobian[0][1];
  jacobianTranspose[1][1] = jacobian[1][1];
  return jacobianTranspose;
}

std::vector<std::vector<double>> SerialVMC::calculateInverseJacobian(const std::vector<std::vector<double>>& jacobian) {
  if (jacobian.size() != 2 || jacobian[0].size() != 2 || jacobian[1].size() != 2) {
    throw std::invalid_argument("Jacobian must be a 2x2 matrix");
  }

  double a = jacobian[0][0];
  double b = jacobian[0][1];
  double c = jacobian[1][0];
  double d = jacobian[1][1];
  double det = a * d - b * c;

  //  if (std::abs(det) < 1e-6) {
  //    throw std::runtime_error("Jacobian matrix is singular or nearly singular");
  //  }

  std::vector<std::vector<double>> inverseJacobian(2, std::vector<double>(2));
  inverseJacobian[0][0] = d / det;
  inverseJacobian[0][1] = -b / det;
  inverseJacobian[1][0] = -c / det;
  inverseJacobian[1][1] = a / det;

  return inverseJacobian;
}

void SerialVMC::calculateVLEVel(double phi1, double dphi1, double phi2, double dphi2)
{
  auto jacobian = calculateJacobian(phi1, phi2);

  dr_ = jacobian[0][0] * dphi1 + jacobian[0][1] * dphi2;
  dtheta_ = jacobian[1][0] * dphi1 + jacobian[1][1] * dphi2;
}

void SerialVMC::calculateVLEEff(double phi1, double tau1, double phi2, double tau2) {
  auto jacobian = calculateJacobian(phi1, phi2);

  auto jacobianTranspose = calculateJacobianTranspose(jacobian);
  std::vector<std::vector<double>> inverseJacobianTranspose = calculateInverseJacobian(jacobianTranspose);

  double Fr = inverseJacobianTranspose[0][0] * tau1 + inverseJacobianTranspose[0][1] * tau2;
  double Ftheta = inverseJacobianTranspose[1][0] * tau1 + inverseJacobianTranspose[1][1] * tau2;

  Fr_ = Fr;
  Ftheta_ = Ftheta;
}

std::vector<double> SerialVMC::getDesJointPos(double desR, double desTheta)
{
  double x = desR * cos(desTheta);
  double y =  - desR * sin(desTheta);
  double d = (l1_ * l1_ + l2_ * l2_ - desR * desR) / (2 * l1_ * l2_);
  if (d < -1.0 || d > 1.0) {
    throw std::runtime_error("No valid solution for the given coordinates");
  }
  double phi2 = M_PI - std::acos(d);
  double k1 = l1_ + l2_ * cos(phi2);
  double k2 = l2_ * sin(phi2);
  double phi1 = atan(x/y) - atan(k2/k1);

  return {phi1, phi2};
}

std::vector<double> SerialVMC::getDesJointVel(double phi1, double phi2, double dVelR, double dVelTheta) {
  std::vector<std::vector<double>> jacobian = calculateJacobian(phi1, phi2);

  std::vector<std::vector<double>> inverseJacobian = calculateInverseJacobian(jacobian);

  double dphi1 = inverseJacobian[0][0] * dVelR + inverseJacobian[0][1] * dVelTheta;
  double dphi2 = inverseJacobian[1][0] * dVelR + inverseJacobian[1][1] * dVelTheta;

  return {dphi1, dphi2};
}

std::vector<double> SerialVMC::getDesJointEff(double phi1, double phi2, double fR, double fT) {
  std::vector<std::vector<double>> jacobian = calculateJacobian(phi1, phi2);

  std::vector<std::vector<double>> jacobianTranspose(2, std::vector<double>(2));
  jacobianTranspose[0][0] = jacobian[0][0];
  jacobianTranspose[0][1] = jacobian[1][0];
  jacobianTranspose[1][0] = jacobian[0][1];
  jacobianTranspose[1][1] = jacobian[1][1];

  double tau1 = jacobianTranspose[0][0] * fR + jacobianTranspose[0][1] * fT;
  double tau2 = jacobianTranspose[1][0] * fR + jacobianTranspose[1][1] * fT;

  return {tau1, tau2};
}

void SerialVMC::update(double phi1, double dphi1, double tau1, double phi2, double dphi2, double tau2)
{
  phi1_ = phi1;
  phi2_ = phi2;
  calculateVLEPos(phi1,phi2);
  calculateVLEVel(phi1,dphi1,phi2,dphi2);
  calculateVLEEff(phi1,tau1,phi2,tau2);
}

void SerialVMC::verifyInverse(const std::vector<std::vector<double>>& jacobian, const std::vector<std::vector<double>>& inverse) {
  std::vector<std::vector<double>> identity = {
    {0, 0},
    {0, 0}
  };
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      for (int k = 0; k < 2; ++k) {
        identity[i][j] += jacobian[i][k] * inverse[k][j];
      }
    }
  }

  std::cout << "Identity Matrix:" << std::endl;
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      std::cout << identity[i][j] << " ";
    }
    std::cout << std::endl;
  }
}

} // namespace vmc