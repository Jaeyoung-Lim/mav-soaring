/****************************************************************************
 *
 *   Copyright (c) 2021 Jaeyoung Lim. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#ifndef ERGODIC_CONTROLLER_H
#define ERGODIC_CONTROLLER_H

#define NUM_STATES 4
#define NUM_INPUTS 1

#include "ergodic_soaring/fourier_coefficient.h"

#include <Eigen/Dense>
#include <vector>

class ErgodicController {
 public:
  ErgodicController();
  virtual ~ErgodicController();
  bool Solve(FourierCoefficient &distribution);
  void SolveSingleIter(FourierCoefficient &distribution, int i);
  void setInitialTrajectory();
  std::vector<State> getTrajectory() { return trajectory_; };
  std::vector<State> getPreprojectedTrajectory() { return preprojected_trajectory_; };
  std::vector<State> getLastTrajectory() { return last_trajectory_; };

  static void LinearizeDynamics(const double cruise_speed, const double dt, const Eigen::Vector3d &pos,
                                Eigen::Matrix<double, NUM_STATES, NUM_STATES> &A,
                                Eigen::Matrix<double, NUM_STATES, NUM_INPUTS> &B);
  static void LinearizeTrajectory(const double cruise_speed, const std::vector<State> &trajectory,
                                  std::vector<Eigen::Matrix<double, NUM_STATES, NUM_STATES>> &A,
                                  std::vector<Eigen::Matrix<double, NUM_STATES, NUM_INPUTS>> &B);

 private:
  State Dynamics(const State &state, const Eigen::Matrix<double, NUM_INPUTS, 1> &input);
  void DescentDirection(const std::vector<State> &trajectory, FourierCoefficient &distribution,
                        std::vector<Eigen::Matrix<double, NUM_STATES, NUM_STATES>> &A,
                        std::vector<Eigen::Matrix<double, NUM_STATES, NUM_INPUTS>> &B,
                        std::vector<Eigen::Matrix<double, NUM_STATES, 1>> &z,
                        std::vector<Eigen::Matrix<double, NUM_INPUTS, 1>> &v,
                        std::vector<Eigen::Matrix<double, NUM_INPUTS, NUM_STATES>> &K);
  Eigen::Matrix<double, NUM_STATES, 1> getCostGradient(const int N, const State &state,
                                                       FourierCoefficient &trajectory_distribution,
                                                       FourierCoefficient &distribution);
  void DescentTrajectory(const std::vector<State> &trajectory, std::vector<Eigen::Matrix<double, NUM_STATES, 1>> &alpha,
                         std::vector<Eigen::Matrix<double, NUM_INPUTS, 1>> &mu,
                         std::vector<Eigen::Matrix<double, NUM_STATES, 1>> &z,
                         std::vector<Eigen::Matrix<double, NUM_INPUTS, 1>> &v, const double gamma);
  void ProjectionOperator(std::vector<State> &trajectory, std::vector<Eigen::Matrix<double, NUM_STATES, 1>> &alpha,
                          std::vector<Eigen::Matrix<double, NUM_INPUTS, 1>> &mu,
                          std::vector<Eigen::Matrix<double, NUM_INPUTS, NUM_STATES>> &K);
  std::shared_ptr<FourierCoefficient> distribution_coefficients;
  std::vector<State> trajectory_;
  std::vector<State> last_trajectory_;
  std::vector<State> preprojected_trajectory_;
  double cruise_speed_{15.0};
};

#endif
