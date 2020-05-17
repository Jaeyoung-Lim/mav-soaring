//  October/2019, Auterion AG, Jaeyoung Lim, jaeyoung@auterion.com

#include "thermal_soaring/thermal_estimator.h"

using namespace Eigen;
using namespace std;
//Constructor
ThermalEstimator::ThermalEstimator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private),
  vehicle_in_thermal_(false) {

  //Vehicle Specific Parameters
  mass_ = 1.5;
  A_wing_ = 0.3;

  K_ = 2 * mass_ * g_ / (rho_ * A_wing_); //SOAR_POLAR_K
  C_D0_ = 0.067; //SOAR_POLAR_C_D0 0 - 0.5
  B_ = 0.037;  //SOAR_POLAR_B 0 - 0.5
  F_ = Eigen::Matrix4d::Identity();   //Process Dynamics

  //TODO: Read noise configurations from parameters
  R_ = 0.01;
  Eigen::Vector4d Q_vector;
  Q_vector << 1.0, 1.0, 1.0, 1.0;
  Q_ = Q_vector.asDiagonal();

  thermal_state_ << 100.0, 10.0, 0.0, 0.0;

  status_pub_ = nh_.advertise<soaring_msgs::ThermalEstimatorStatus>("/soaring/thermal_estimator/status", 1);
}

ThermalEstimator::~ThermalEstimator() {
  //Destructor
}

void ThermalEstimator::UpdateState(Eigen::Vector3d position, Eigen::Vector3d velocity, Eigen::Vector4d attitude, Eigen::Vector3d wind_velocity){
  //Update States
  Eigen::Vector3d current_position = position;
  Eigen::Vector3d current_velocity = velocity;

  //Detect Thermal
  double netto_rate = getNettoVariometer(current_velocity);
  vehicle_in_thermal_ = bool(netto_rate > soaring_threshold_);

  //Estimate Thermal states
  ///State transition
  Eigen::Vector4d thermal_state_hat = thermal_state_;
  Eigen::Matrix4d thermal_state_covariance_hat = thermal_state_covariance_ + Q_;

  ///Compute Kalman gains
  Eigen::Vector4d H = ObservationProcess(thermal_state_);
  double den = H.transpose() * thermal_state_covariance_hat * H + R_;
  Eigen::Vector4d K_kalman_ = thermal_state_covariance_hat * H / den;

  //Update
  thermal_state_ = thermal_state_hat + K_kalman_ * (netto_rate - ObservationFunction(thermal_state_));
  thermal_state_covariance_ = (Eigen::Matrix4d::Identity() - K_kalman_ * H.transpose())*thermal_state_covariance_ + Q_;

  prev_position_ = current_position;
  prev_velocity_ = current_velocity;

  PublishEstimatorStatus(thermal_state_);
}

void ThermalEstimator::reset() {
  thermal_state_ = Eigen::Vector4d::Zero();


}

bool ThermalEstimator::IsInThermal(){
  return vehicle_in_thermal_;
}

double ThermalEstimator::getNettoVariometer(Eigen::Vector3d velocity){
  double netto_variometer, vz, e_dot;
  double phi = 0.0;

  //TODO: Subscribe to bank angles
  //TODO: Get airspeed properly
  vz = getDragPolarCurve(velocity.norm(), phi);
  e_dot = getSpecificEnergyRate(velocity, prev_velocity_);
  netto_variometer = e_dot + vz;

  // std::cout << "e_dot: "<< e_dot  << std::endl;
  // std::cout << "vz: "<< vz  << std::endl;
  // std::cout << "Netto Variometer: "<< netto_variometer  << std::endl;
  return netto_variometer;
}

double ThermalEstimator::getDragPolarCurve(double airspeed, double bank_angle){
  double v_z, C_L;
  
  C_L = K_ / std::pow(airspeed, 2);
  C_L = std::min(std::max(C_L, 0.0), 5.0);
  
  v_z = airspeed * (C_D0_ / C_L + B_ * C_L / std::pow(cos(bank_angle), 2));

  // std::cout << "CL: " << C_L << std::endl;
  // std::cout << "vz: " << v_z << std::endl;
  // std::cout << "velocity_z: " << prev_velocity_(2) << std::endl;
  return v_z;
}

double ThermalEstimator::getSpecificEnergyRate(Eigen::Vector3d velocity, Eigen::Vector3d prev_velocity){
  double e_dot, v_dot, h_dot;
  double dt = 1.0;

  v_dot = (velocity.norm() - prev_velocity_.norm()) / dt;
  h_dot = velocity(2);
  e_dot = h_dot + velocity.norm() * v_dot / g_;

  return e_dot;
}

Eigen::Vector3d ThermalEstimator::getThermalPosition(){
  Eigen::Vector3d thermal_center;
  thermal_center << thermal_state_(2), thermal_state_(3), 0.0;
  return thermal_center;
}

Eigen::Vector4d ThermalEstimator::ObservationProcess(Eigen::Vector4d state){
  Eigen::Vector4d H;
  const double W_th = state(0);
  const double R_th = state(1);
  const double x = state(2);
  const double y = state(3);

  H(0) = std::exp(- (x*x + y*y)/(R_th*R_th));
  H(1) = 2 * W_th * (x*x + y*y) * H(0) / (std::pow(R_th, 3));
  H(2) = 2 * W_th * x * H(0) / (std::pow(R_th, 2));
  H(3) = 2 * W_th * y * H(0) / (std::pow(R_th, 2));
  
  return H;
}

double ThermalEstimator::ObservationFunction(Eigen::Vector4d state){
  const double W_th = state(0);
  const double R_th = state(1);
  const double x = state(2);
  const double y = state(3);

  return W_th * std::exp( - (x*x + y*y)/(R_th*R_th));
}

void ThermalEstimator::PublishEstimatorStatus(Eigen::Vector4d state) {
  soaring_msgs::ThermalEstimatorStatus msg;

  msg.header.stamp = ros::Time::now();
  msg.position.x = state(2);
  msg.position.y = state(3);
  msg.radius = state(1);
  msg.strength = state(0);
  status_pub_.publish(msg);
}
