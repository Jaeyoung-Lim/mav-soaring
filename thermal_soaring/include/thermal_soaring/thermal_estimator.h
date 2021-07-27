//  October/2019, Auterion AG, Jaeyoung Lim, jaeyoung@auterion.com

#ifndef THERMAL_ESTIMATOR_H
#define THERMAL_ESTIMATOR_H

#include <stdio.h>
#include <cstdlib>
#include <sstream>
#include <string>

#include <math.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class ThermalEstimator {
 private:
  // Parameters
  double R_;

  // thermal state vector is defined as below
  // W_th : Thermal Strength
  // R_th : Thermal Radius
  // x    : Thermal Center x
  // y    : Thermal Cetner y
  Eigen::Vector4d thermal_state_;
  Eigen::Matrix4d thermal_state_covariance_;

  Eigen::Matrix4d F_;
  Eigen::Matrix4d Q_;

 protected:
  void ObservationProcess(Eigen::Vector4d state, Eigen::Vector4d &H, double &predicted_measurement);
  void PriorUpdate(const Eigen::Vector4d &state, const Eigen::Matrix4d &covariance, Eigen::Vector4d &predicted_state,
                   Eigen::Matrix4d &predicted_covariance);
  void MeasurementUpdate(const Eigen::Vector4d &state, const Eigen::Matrix4d &covariance,
                         Eigen::Vector4d &updated_state, Eigen::Matrix4d &updated_covariance, double measurement);

 public:
  ThermalEstimator();
  virtual ~ThermalEstimator();
  void UpdateState(Eigen::Vector3d position, Eigen::Vector3d velocity, Eigen::Vector4d attitude,
                   Eigen::Vector3d wind_velocity, double netto_vario);
  void reset();
  Eigen::Vector3d getThermalPosition();
  Eigen::Vector4d getThermalState() { return thermal_state_; };
};

#endif