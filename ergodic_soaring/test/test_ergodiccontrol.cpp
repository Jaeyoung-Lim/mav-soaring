#include "ergodic_soaring/ergodic_controller.h"

#include <gtest/gtest.h>
#include <memory.h>
#include <ros/ros.h>
#include <iostream>

// class ErgodicControllerTest : public ErgodicController, public ::testing::Test {
//  public:
//   void SetUp() override {
//     // det = std::make_unique<ThermalDetector>();
//   }

//   void TearDown() override {}
// };

// TEST_F(ErgodicControllerTest, TestOptimization) { EXPECT_TRUE(1.0 > 0.0); }

/**
 * @brief Test Linearize Dynamics
 *
 */
TEST(ErgodicControllerTest, LinearizeDynmaics) {
  const double cruise_speed = 1.0;
  const double dt = 0.1;

  Eigen::Matrix<double, NUM_STATES, NUM_STATES> A;
  Eigen::Matrix<double, NUM_STATES, NUM_INPUTS> B;
  Eigen::Matrix<double, NUM_STATES, NUM_INPUTS> B_ref;
  B_ref << 0.0, 0.0, dt;
  // Test cases
  Eigen::Vector3d position;
  position << 0.0, 0.0, 0.0;
  ErgodicController::LinearizeDynamics(cruise_speed, dt, position, A, B);
  EXPECT_TRUE(B.isApprox(B_ref));
}
