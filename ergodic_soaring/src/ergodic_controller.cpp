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
#include "ergodic_soaring/ergodic_controller.h"

ErgodicController::ErgodicController() {}

ErgodicController::~ErgodicController() {}

void ErgodicController::LinearizeDynamics(
    std::vector<State> &trajectory, std::vector<Eigen::Matrix<double, NUM_STATES, NUM_STATES>> &A,
    std::vector<Eigen::Matrix<double, NUM_STATES, NUM_INPUTS>> &B) {  // Dubins plane
  double v0 = 15.0;
  double dt = 0.1;
  std::cout << "[LinearizeDynamics] trajectory size: " << trajectory.size() << std::endl;

  for (auto state : trajectory) {
    Eigen::Matrix<double, NUM_STATES, NUM_STATES> An;
    Eigen::Matrix<double, NUM_STATES, NUM_INPUTS> Bn;
    double theta = state.position(2);
    // Linearize dynamics
    An << 1.0, 0.0, -sin(theta) * v0 * dt, 0.0, 1.0, cos(theta) * v0 * dt, 0.0, 0.0, 1.0;
    Bn << 0.0, 0.0, dt;

    A.push_back(An);
    B.push_back(Bn);
  }
}

Eigen::Matrix<double, NUM_STATES, 1> ErgodicController::getCostGradient(State &state,
                                                                        FourierCoefficient &trajectory_distribution,
                                                                        FourierCoefficient &distribution) {
  Eigen::Matrix<double, NUM_STATES, 1> gradient;
  double c_b = 0.0;  // Boundary barrier weight

  /// TODO: Get Boundary from map
  double max_boundary = 100;
  double min_boundary = -100;
  // Eigen::Matrix<double, NUM_STATES, 1> boundary_gradient = Eigen::MatrixXd::Zero();
  /// TODO: Boundary penalty gradient
  // for (int i = 0; i < 2; i++) {
  //   boundary_gradient+= std::max(state.position(i) - max_boundary, 0.0) + std::min(state.position(i), min_boundary)
  // }

  /// TODO: Ergodic gradient
  // FourierCoefficient trajectory_coefficients(20);
  Eigen::Matrix<double, NUM_STATES, 1> ergodic_gradient;
  ergodic_gradient = distribution.getErgodicGradient(trajectory_distribution.getCoefficients());
  // c_b * boundary_gradient
  gradient = ergodic_gradient;
  return gradient;
}

void ErgodicController::DescentDirection(std::vector<State> &trajectory, FourierCoefficient &distribution,
                                         std::vector<Eigen::Matrix<double, NUM_STATES, NUM_STATES>> &A,
                                         std::vector<Eigen::Matrix<double, NUM_STATES, NUM_INPUTS>> &B,
                                         std::vector<Eigen::Matrix<double, NUM_STATES, 1>> &z,
                                         std::vector<Eigen::Matrix<double, NUM_INPUTS, 1>> &v) {
  FourierCoefficient trajectory_distribution(20);
  trajectory_distribution.FourierTransform(trajectory);  // Transform trajectory distribution after update

  // Cost function
  Eigen::Matrix<double, NUM_INPUTS, NUM_INPUTS> R = Eigen::MatrixXd::Identity(NUM_INPUTS, NUM_INPUTS);

  // Solve gradient discent direction that minimizes the cost function
  const int N = trajectory.size();
  const int dt = trajectory[0].dt;
  Eigen::Matrix<double, NUM_STATES, NUM_STATES> Q_D = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);
  Eigen::Matrix<double, NUM_INPUTS, NUM_INPUTS> R_D = Eigen::MatrixXd::Identity(NUM_INPUTS, NUM_INPUTS);

  std::vector<Eigen::Matrix<double, NUM_STATES, NUM_STATES>> P(N);
  std::vector<Eigen::Matrix<double, NUM_STATES, 1>> r(N);
  std::vector<Eigen::Matrix<double, NUM_STATES, 1>> a(N);
  std::vector<Eigen::Matrix<double, NUM_INPUTS, NUM_STATES>> K(N);
  std::vector<Eigen::Matrix<double, NUM_INPUTS, NUM_INPUTS>> b(N);

  // Calculate cost gradients
  for (size_t m = 0; m < N; m++) {
    /// TODO: Populate a[n]
    a[m] = getCostGradient(trajectory[m], trajectory_distribution, distribution);
    b[m] = dt * R * trajectory[m].input;
  }

  P[N - 1] = Q_D;
  r[N - 1] = a[N - 1];  /// TODO: get cost gradient

  // Solve LQ Problem with backward pass
  for (size_t n = N - 2; n > -1; n--) {
    Eigen::Matrix<double, NUM_INPUTS, NUM_INPUTS> gamma_n = R_D + (B[n].transpose() * P[n + 1] * B[n]);
    K[n] = gamma_n.inverse() * B[n].transpose() * P[n + 1] * A[n];
    P[n] = Q_D + A[n].transpose() * P[n + 1] * A[n] - K[n].transpose() * gamma_n * K[n];
    r[n] = a[n] + (A[n].transpose() - K[n].transpose() * B[n].transpose()) * r[n + 1] - K[n + 1].transpose() * b[n];
  }
  // Get Descent direction with forward pass
}

bool ErgodicController::Solve(FourierCoefficient &distribution) {
  std::cout << "Starting to solve ergodic control!" << std::endl;
  std::vector<State> trajectory;
  double dt = 0.1;
  for (int i = 0; i < 10; i++) {
    State state;
    trajectory.push_back(state);
  }
  /// TODO: Generate initial trajectory

  bool exit = false;
  int max_iteration = 2;
  int iter{0};
  while (!exit) {
    // Linearize dynamics along the trajectory
    std::vector<Eigen::Matrix<double, NUM_STATES, NUM_STATES>> A;
    std::vector<Eigen::Matrix<double, NUM_STATES, NUM_INPUTS>> B;
    LinearizeDynamics(trajectory, A, B);

    /// Compute Descent direction
    std::vector<Eigen::Matrix<double, NUM_STATES, 1>> Z;  // State Descent direction
    std::vector<Eigen::Matrix<double, NUM_INPUTS, 1>> U;  // Input Descent direction
    DescentDirection(trajectory, distribution, A, B, Z, U);

    /// TODO:  determine step size and descend
    // 		step_size = get_step_size(tm.descender, em, tm, xd, ud, zd, vd, ad, bd, K, i)

    /// TODO: Combine gradient descent
    // 		# descend and project
    // 		xd, ud = project(em, tm, K, xd, ud, zd, vd, step_size)
    if (iter < max_iteration) {
      exit = true;
    }
    iter++;
  }
  // 	return xd, ud

  return true;
}
