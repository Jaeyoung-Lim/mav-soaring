//  October/2019, Auterion AG, Jaeyoung Lim, jaeyoung@auterion.com

#ifndef THERMAL_SOARING_H
#define THERMAL_SOARING_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>

#include <stdio.h>
#include <cstdlib>
#include <sstream>
#include <string>

#include <Eigen/Dense>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <mavros_msgs/PositionTarget.h>

#include "thermal_soaring/thermal_detector.h"
#include "thermal_soaring/thermal_estimator.h"

#include <mavros_msgs/PositionTarget.h>
#include "soaring_msgs/ThermalEstimatorStatus.h"

using namespace std;
using namespace Eigen;

uint16_t SETPOINT_MODE_SOAR = mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_VZ |
                              mavros_msgs::PositionTarget::IGNORE_AFZ;
uint16_t SETPOINT_MODE_CRUISE = 0x3000;
double SOAR_ALT_CUTOFF = 70.0;
double SOAR_ALT_MAX = 100.0;
double SOAR_ALT_MIN = 30.0;

enum class MAV_STATE {
  MAV_STATE_UNINIT,
  MAV_STATE_BOOT,
  MAV_STATE_CALIBRATIN,
  MAV_STATE_STANDBY,
  MAV_STATE_ACTIVE,
  MAV_STATE_CRITICAL,
  MAV_STATE_EMERGENCY,
  MAV_STATE_POWEROFF,
  MAV_STATE_FLIGHT_TERMINATION,
};

enum class CONTROLLER_STATE {
  STATE_FREE_SOAR,
  STATE_THERMAL_SOAR,
  STATE_REACH_ALTITUDE,
};

class ThermalSoaring {
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher setpointraw_pub_;
  ros::Publisher status_pub_;
  ros::Subscriber mavpose_sub_;
  ros::Subscriber mavtwist_sub_;
  ros::Subscriber windest_sub_;

  ros::Timer cmdloop_timer_, statusloop_timer_;

  bool is_in_thermal_;

  uint16_t flight_mode_ = SETPOINT_MODE_SOAR;

  CONTROLLER_STATE controller_state_ = CONTROLLER_STATE::STATE_FREE_SOAR;

  Eigen::Vector3d mavPos_, mavVel;
  Eigen::Vector3d mavVel_, mavRate_;
  Eigen::Vector4d mavAtt_;
  Eigen::Vector3d thermal_position_;
  Eigen::Vector3d target_position_;
  Eigen::Vector3d wind_velocity_;

  ThermalEstimator thermal_estimator_;
  ThermalDetector thermal_detector_;

  void cmdloopCallback(const ros::TimerEvent& event);
  void statusloopCallback(const ros::TimerEvent& event);
  void runFreeSoar();
  void runReachAltitude();
  void runThermalSoar();
  void PubPositionSetpointRaw();
  void PublishEstimatorStatus();
  void mavposeCallback(const geometry_msgs::PoseStamped& msg) {
    mavPos_(0) = msg.pose.position.x;
    mavPos_(1) = msg.pose.position.y;
    mavPos_(2) = msg.pose.position.z;
    mavAtt_(0) = msg.pose.orientation.w;
    mavAtt_(1) = msg.pose.orientation.x;
    mavAtt_(2) = msg.pose.orientation.y;
    mavAtt_(3) = msg.pose.orientation.z;
  };
  void mavtwistCallback(const geometry_msgs::TwistStamped& msg) {
    mavVel_(0) = msg.twist.linear.x;
    mavVel_(1) = msg.twist.linear.y;
    mavVel_(2) = msg.twist.linear.z;
    mavRate_(0) = msg.twist.angular.x;
    mavRate_(1) = msg.twist.angular.y;
    mavRate_(2) = msg.twist.angular.z;
  };
  void windestimationCallback(const geometry_msgs::TwistWithCovarianceStamped& msg) {
    wind_velocity_(0) = msg.twist.twist.linear.x;
    wind_velocity_(1) = msg.twist.twist.linear.y;
    wind_velocity_(2) = msg.twist.twist.linear.z;
  };

 public:
  ThermalSoaring(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  virtual ~ThermalSoaring();
};

#endif
