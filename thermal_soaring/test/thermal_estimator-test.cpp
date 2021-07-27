#include "thermal_soaring/thermal_estimator.h"

#include <gtest/gtest.h>
#include <memory.h>
#include <ros/ros.h>
#include <iostream>

class ThermalEstimatorTest : public ThermalEstimator, public ::testing::Test {
 public:
  void SetUp() override {}

  void TearDown() override {}
};

TEST_F(ThermalEstimatorTest, TestPriorUpdate) {
  Eigen::Vector4d state, predicted_state;
  Eigen::Matrix4d covariance, predicted_covariance;

  // No particular update in prior updates
  PriorUpdate(state, covariance, predicted_state, predicted_covariance);
  EXPECT_TRUE(state.isApprox(predicted_state));
}

TEST_F(ThermalEstimatorTest, TestMeasurementUpdate) {
  Eigen::Vector4d state, predicted_state;
  EXPECT_TRUE(true);
}

TEST_F(ThermalEstimatorTest, ObservationProcess) { EXPECT_TRUE(true); }
