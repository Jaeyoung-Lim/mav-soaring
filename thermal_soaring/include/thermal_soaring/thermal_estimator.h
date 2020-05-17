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

#include "soaring_msgs/ThermalEstimatorStatus.h"

using namespace std;
using namespace Eigen;

class ThermalEstimator
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher  status_pub_;
  
    //Parameters
    double K_;
    const double g_ = 9.8;
    double mass_;
    const double rho_ = 1.225;
    double A_wing_;
    double C_D0_;
    double B_;
    double R_;
    double soaring_threshold_ = 0.1;


    //Estimator States
    bool vehicle_in_thermal_;
    // thermal state vector is defined as below
    // W_th : Thermal Strength
    // R_th : Thermal Radius
    // x    : Thermal Center x
    // y    : Thermal Cetner y
    Eigen::Vector4d thermal_state_;
    Eigen::Matrix4d thermal_state_covariance_;

    //Vehicle State
    Eigen::Vector3d prev_position_;
    Eigen::Vector3d prev_velocity_;

    Eigen::Matrix4d F_;
    Eigen::Matrix4d Q_;

    double getNettoVariometer(Eigen::Vector3d velocity);
    double getDragPolarCurve(double airspeed, double bank_angle);
    double getSpecificEnergyRate(Eigen::Vector3d velocity, Eigen::Vector3d prev_velocity);
    double ObservationFunction(Eigen::Vector4d state);
    Eigen::Vector4d ObservationProcess(Eigen::Vector4d state);
    void PublishEstimatorStatus(Eigen::Vector4d state);


  public:
    ThermalEstimator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~ ThermalEstimator();
    void UpdateState(Eigen::Vector3d position, Eigen::Vector3d velocity, Eigen::Vector4d attitude, Eigen::Vector3d wind_velocity);
    void reset();
    bool IsInThermal();
    Eigen::Vector3d getThermalPosition();
};


#endif