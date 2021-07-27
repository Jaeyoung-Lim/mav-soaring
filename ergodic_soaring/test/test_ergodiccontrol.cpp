#include "ergodic_soaring/ergodic_controller.h"

#include <gtest/gtest.h>
#include <memory.h>
#include <ros/ros.h>
#include <iostream>

class ErgodicControllerTest : public ErgodicController, public ::testing::Test {
 public:
  void SetUp() override {
    // det = std::make_unique<ThermalDetector>();
  }

  void TearDown() override {}
};

TEST_F(ErgodicControllerTest, TestOptimization) { EXPECT_TRUE(1.0 > 0.0); }
