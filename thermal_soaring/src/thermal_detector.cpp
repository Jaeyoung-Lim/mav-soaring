#include "thermal_soaring/thermal_detector.h"

ThermalDetector::ThermalDetector() {
  // Vehicle Specific Parameters
  mass_ = 1.5;
  A_wing_ = 1.2;
  C_D0_ = 0.0067;  // SOAR_POLAR_C_D0 0 - 0.5
  C_L_MAX_ = 5.0;
  B_ = 0.037;  // SOAR_POLAR_B 0 - 0.5

  K_ = 2 * mass_ * g_ / (rho_ * A_wing_);  // SOAR_POLAR_K
}

ThermalDetector::~ThermalDetector() {}

void ThermalDetector::UpdateState(const Eigen::Vector3d &velocity, const Eigen::Vector4d &attitude) {
  // TODO: Infer time differences
  double dt = 1.0;

  Eigen::Quaterniond q(attitude[0], attitude[1], attitude[2], attitude[3]);
  Eigen::Vector3d rpy = Eigen::Matrix3d(q).eulerAngles(0, 1, 2);  // RPY
  double bank_angle = rpy(0);

  // TODO: Get airspeed properly
  double vz = getDragPolarCurve(velocity.norm(), bank_angle);
  double e_dot = getSpecificEnergyRate(velocity, prev_velocity_, dt);
  netto_variometer_ = e_dot + vz;

  prev_velocity_ = velocity;
}

double ThermalDetector::getDragPolarCurve(double airspeed, double bank_angle) {
  if (airspeed <= 0.0) {
    return 0.0;
  }
  double C_L = std::min(std::max(K_ / std::pow(airspeed, 2), 0.0), C_L_MAX_);
  double v_z = airspeed * (C_D0_ / C_L + B_ * C_L / std::pow(cos(bank_angle), 2));

  return v_z;
}

double ThermalDetector::getSpecificEnergyRate(Eigen::Vector3d velocity, Eigen::Vector3d prev_velocity, double dt) {
  if (dt <= 0) {
    return 0.0;
  }
  double v_dot = (velocity.norm() - prev_velocity.norm()) / dt;
  double h_dot = velocity(2);
  double e_dot = h_dot + velocity.norm() * v_dot / g_;

  return e_dot;
}
