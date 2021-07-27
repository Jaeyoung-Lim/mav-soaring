#include "thermal_soaring/thermal_detector.h"

#include <gtest/gtest.h>
#include <memory.h>
#include <ros/ros.h>
#include <iostream>

class ThermalDetectorTest : public ThermalDetector, public ::testing::Test {
 public:
  void SetUp() override {
    // det = std::make_unique<ThermalDetector>();
  }

  void TearDown() override {}
};

TEST_F(ThermalDetectorTest, TestSpecificEnergyRate) {
  // Specific Energy Rate is zero when stationary
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d prev_velocity = Eigen::Vector3d::Zero();
  EXPECT_NEAR(getSpecificEnergyRate(velocity, prev_velocity, 1.0), 0.0, 0.0001);
  prev_velocity << 10.0, 0.0, 0.0;  // Descending
  velocity << 10.0, 0.0, -1.0;      // Descending
  // Simultaneuous Querying
  EXPECT_NEAR(getSpecificEnergyRate(velocity, prev_velocity, 0.0), 0.0, 0.0001);
  // Wrong timestamp
  EXPECT_NEAR(getSpecificEnergyRate(velocity, prev_velocity, -1.0), 0.0, 0.0001);
  // Descending specific energy rate
  EXPECT_TRUE(getSpecificEnergyRate(velocity, prev_velocity, 1.0) < 0.0);
  velocity << 10.0, 0.0, 1.0;  // Ascending
  // Ascending specific energy rate
  EXPECT_TRUE(getSpecificEnergyRate(velocity, prev_velocity, 1.0) > 0.0);
}

TEST_F(ThermalDetectorTest, TestDragPolarCurve) {
  // Specific Energy Rate is zero when stationary
  double airspeed = 0.0;
  double bank_angle = 0.0;
  EXPECT_NEAR(getDragPolarCurve(airspeed, bank_angle), 0.0, 0.0001);
  airspeed = 10.0;
  bank_angle = 0.0;
  EXPECT_TRUE(getDragPolarCurve(airspeed, bank_angle) > 0.0);
  airspeed = 10.0;
  bank_angle = 0.1;
  EXPECT_TRUE(getDragPolarCurve(airspeed, bank_angle) > 0.0);
}

TEST_F(ThermalDetectorTest, TestNettoVariometer) {
  // Test NettoVariometer when stationary
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector4d attitude = Eigen::Vector4d::Zero();
  UpdateState(velocity, attitude);
  UpdateState(velocity, attitude);
  EXPECT_NEAR(getNettoVariometer(), 0.0, 0.0001);
}