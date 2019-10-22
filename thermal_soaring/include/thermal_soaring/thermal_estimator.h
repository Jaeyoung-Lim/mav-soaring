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
#include <math.h>

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

    double R_;

    Eigen::Vector3d position_;
    Eigen::Vector3d velocity_;
    Eigen::Vector3d thermal_center_;
    Eigen::Vector3d wind_velocity_;
    Eigen::Vector4d thermal_state_;
    Eigen::Matrix4d thermal_state_covariance_;
    Eigen::Vector4d Q_vector_;
    Eigen::Vector4d K_kalman_;
    Eigen::Vector4d H_;
    Eigen::Matrix4d F_;
    Eigen::Matrix4d Q_;

    double getNettoVariometer();
    double getDragPolarCurve(double airspeed, double bank_angle);
    double getSpecificEnergyRate();
    double ObservationFunction(Eigen::Vector4d state);
    Eigen::Vector4d ObservationProcess(Eigen::Vector4d state);
    Eigen::Vector4d computeKalmanGains(Eigen::Vector4d state);


  public:
    ThermalEstimator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~ ThermalEstimator();
    void UpdateState(Eigen::Vector3d position, Eigen::Vector3d velocity, Eigen::Vector3d wind_velocity);
    bool IsInThermal();
    Eigen::Vector3d getThermalPosition();
};


#endif