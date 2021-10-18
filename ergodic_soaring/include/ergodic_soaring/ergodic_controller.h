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

#define NUM_STATES 3
#define NUM_INPUTS 1

#include "ergodic_soaring/fourier_coefficient.h"

#include <Eigen/Dense>
#include <vector>

class ErgodicController {
 public:
  ErgodicController();
  virtual ~ErgodicController();
  bool Solve(FourierCoefficient &distribution);
  bool SolveSingleIter(FourierCoefficient &distribution);
  void setInitialTrajectory();
  std::vector<State> getTrajectory() { return trajectory_; };

 private:
  void LinearizeDynamics(std::vector<State> &trajectory, std::vector<Eigen::Matrix<double, NUM_STATES, NUM_STATES>> &A,
                         std::vector<Eigen::Matrix<double, NUM_STATES, NUM_INPUTS>> &B);
  void DescentDirection(std::vector<State> &trajectory, FourierCoefficient &distribution,
                        std::vector<Eigen::Matrix<double, NUM_STATES, NUM_STATES>> &A,
                        std::vector<Eigen::Matrix<double, NUM_STATES, NUM_INPUTS>> &B,
                        std::vector<Eigen::Matrix<double, NUM_STATES, 1>> &z,
                        std::vector<Eigen::Matrix<double, NUM_INPUTS, 1>> &v);
  Eigen::Matrix<double, NUM_STATES, 1> getCostGradient(State &state, FourierCoefficient &trajectory_distribution,
                                                       FourierCoefficient &distribution);
  void DescentTrajectory(std::vector<State> &trajectory, std::vector<Eigen::Matrix<double, NUM_STATES, 1>> &z,
                         std::vector<Eigen::Matrix<double, NUM_INPUTS, 1>> &v);
  std::shared_ptr<FourierCoefficient> distribution_coefficients;
  std::vector<State> trajectory_;
};

#endif
