//  October/2019, Auterion AG, Jaeyoung Lim, jaeyoung@auterion.com

#include "thermal_soaring/thermal_estimator.h"

using namespace Eigen;
using namespace std;
// Constructor
ThermalEstimator::ThermalEstimator() {
  F_ = Eigen::Matrix4d::Identity();  // Process Dynamics

  // TODO: Read noise configurations from parameters
  R_ = 0.01;
  Eigen::Vector4d Q_vector;
  Q_vector << 1.0, 1.0, 1.0, 1.0;
  Q_ = Q_vector.asDiagonal();

  thermal_state_ << 100.0, 10.0, 0.0, 0.0;
}

ThermalEstimator::~ThermalEstimator() {
  // Destructor
}

void ThermalEstimator::UpdateState(Eigen::Vector3d position, Eigen::Vector3d velocity, Eigen::Vector4d attitude,
                                   Eigen::Vector3d wind_velocity, double netto_vario) {
  // Update States
  Eigen::Vector3d current_position = position;
  Eigen::Vector3d current_velocity = velocity;

  // Estimate Thermal states
  /// Prior Update
  Eigen::Vector4d predicted_thermal_state;
  Eigen::Matrix4d predicted_thermal_covariance;
  PriorUpdate(thermal_state_, thermal_state_covariance_, predicted_thermal_state, predicted_thermal_covariance);

  // MeaurementUpdate
  MeasurementUpdate(predicted_thermal_state, predicted_thermal_covariance, thermal_state_, thermal_state_covariance_,
                    netto_vario);
}

void ThermalEstimator::reset() { thermal_state_ = Eigen::Vector4d::Zero(); }

void ThermalEstimator::PriorUpdate(const Eigen::Vector4d &state, const Eigen::Matrix4d &covariance,
                                   Eigen::Vector4d &predicted_state, Eigen::Matrix4d &predicted_covariance) {
  predicted_state = state;
  predicted_covariance = covariance + Q_;
}

void ThermalEstimator::MeasurementUpdate(const Eigen::Vector4d &state, const Eigen::Matrix4d &covariance,
                                         Eigen::Vector4d &updated_state, Eigen::Matrix4d &updated_covariance,
                                         double measurement) {
  /// Compute Kalman gains
  double predicted_measurement;
  Eigen::Vector4d H;
  ObservationProcess(thermal_state_, H, predicted_measurement);
  double den = H.transpose() * covariance * H + R_;
  Eigen::Vector4d K_kalman_ = covariance * H / den;

  // Update
  // TODO: Publish innovations
  updated_state = state + K_kalman_ * (measurement - predicted_measurement);
  updated_covariance = (Eigen::Matrix4d::Identity() - K_kalman_ * H.transpose()) * covariance + Q_;
}

Eigen::Vector3d ThermalEstimator::getThermalPosition() {
  Eigen::Vector3d thermal_center;
  thermal_center << thermal_state_(2), thermal_state_(3), 0.0;
  return thermal_center;
}

void ThermalEstimator::ObservationProcess(Eigen::Vector4d state, Eigen::Vector4d &H, double &predicted_measurement) {
  const double W_th = state(0);
  const double R_th = state(1);
  const double x = state(2);
  const double y = state(3);

  predicted_measurement = W_th * std::exp(-(x * x + y * y) / (R_th * R_th));

  H(0) = std::exp(-(x * x + y * y) / (R_th * R_th));
  H(1) = 2 * W_th * (x * x + y * y) * H(0) / (std::pow(R_th, 3));
  H(2) = 2 * W_th * x * H(0) / (std::pow(R_th, 2));
  H(3) = 2 * W_th * y * H(0) / (std::pow(R_th, 2));
}
