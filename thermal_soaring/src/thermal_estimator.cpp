//  October/2019, Auterion AG, Jaeyoung Lim, jaeyoung@auterion.com

#include "thermal_soaring/thermal_estimator.h"

using namespace Eigen;
using namespace std;
//Constructor
ThermalEstimator::ThermalEstimator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private),
  thermal_detected_(false) {
  
  mass_ = 1.0;
  A_wing_ = 1.0;

  K_ = 2 * mass_ * g_ / (rho_ * A_wing_);
  B_ = 1.0;
  C_D0_ = 0.3;

}

ThermalEstimator::~ThermalEstimator() {
  //Destructor
}

void ThermalEstimator::UpdateState(Eigen::Vector3d position, Eigen::Vector3d velocity){
  position_ = position;
  velocity_ = velocity;

}

bool ThermalEstimator::IsInThermal(){
  double soaring_threshold_ = 0.1;
  return bool(getNettoVariometer() > soaring_threshold_);
}

double ThermalEstimator::getNettoVariometer(){
  double netto_variometer, vz, e_dot;
  double phi;
  
  vz = getDragPolarCurve(velocity_.norm(), phi);
  e_dot = getSpecificEnergyRate();
  netto_variometer = e_dot + vz;
  
  return netto_variometer;
}

double ThermalEstimator::getDragPolarCurve(double airspeed, double bank_angle){
  double v_z, C_L;
  
  C_L = K_ / std::pow(airspeed, 2);
  v_z = airspeed * (C_D0_ / C_L + B_ * C_L / std::pow(cos(bank_angle), 2));
  
  return v_z;
}

double ThermalEstimator::getSpecificEnergyRate(){
  double e_dot, v_dot, h_dot;
  double dt = 1.0;
  Eigen::Vector3d prev_velocity_;
  Eigen::Vector3d prev_position_;

  v_dot = (velocity_ - prev_velocity_).norm() / dt;
  h_dot = (position_(2) - prev_position_(2)) / dt;
  e_dot = h_dot + velocity_.norm() * v_dot / g_;

  return e_dot;
}

Eigen::Vector3d ThermalEstimator::getThermalPosition(){
  return thermal_center_;
}
