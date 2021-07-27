//  October/2020, Auterion AG, Jaeyoung Lim, jaeyoung@auterion.com

#ifndef THERMAL_DETECTOR_H
#define THERMAL_DETECTOR_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>

#include <stdio.h>
#include <cstdlib>
#include <sstream>
#include <string>

#include <math.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class ThermalDetector {
 private:
  // Parameters
  double K_;
  const double g_{9.8};
  double mass_;
  const double rho_{1.225};
  double A_wing_;
  double C_D0_;
  double C_L_MAX_{5.0};
  double B_;
  bool vehicle_in_thermal_;
  double netto_variometer_;

  // Vehicle State
  Eigen::Vector3d prev_velocity_;

 protected:
  double getDragPolarCurve(double airspeed, double bank_angle);
  double getSpecificEnergyRate(Eigen::Vector3d velocity, Eigen::Vector3d prev_velocity, double dt);

 public:
  ThermalDetector();
  virtual ~ThermalDetector();
  void UpdateState(const Eigen::Vector3d &velocity, const Eigen::Vector4d &attitude);
  bool IsInThermal() { return vehicle_in_thermal_; };
  double getNettoVariometer() { return netto_variometer_; };
};

#endif