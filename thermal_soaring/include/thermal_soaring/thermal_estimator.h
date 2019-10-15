//  October/2019, Auterion AG, Jaeyoung Lim, jaeyoung@auterion.com

#ifndef THERMAL_ESTIMATOR_H
#define THERMAL_ESTIMATOR_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class ThermalEstimator
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    bool thermal_detected_;

    double K_;
    const double g_ = 9.8;
    double mass_;
    const double rho_ = 1.225;
    double A_wing_;
    double C_D0_;
    double B_;

    Eigen::Vector3d position_;
    Eigen::Vector3d velocity_;

    double getNettoVariometer();
    double getDragPolarCurve(double airspeed, double bank_angle);
    double getSpecificEnergyRate();

  public:
    ThermalEstimator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~ ThermalEstimator();
    void UpdateState(Eigen::Vector3d position, Eigen::Vector3d velocity);
    bool IsInThermal();
};


#endif