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
  ergodic_gradient = distribution.getErgodicGradient(state, trajectory_distribution.getCoefficients());
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
  trajectory_distribution.setGridMap(distribution.getGridMap());
  trajectory_distribution.FourierTransform(trajectory);  // Transform trajectory distribution after update

  // Cost function
  Eigen::Matrix<double, NUM_INPUTS, NUM_INPUTS> R = Eigen::MatrixXd::Identity(NUM_INPUTS, NUM_INPUTS);

  // Solve gradient discent direction that minimizes the cost function
  const int N = trajectory.size();
  Eigen::Matrix<double, NUM_STATES, NUM_STATES> Q_D = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);
  Eigen::Matrix<double, NUM_INPUTS, NUM_INPUTS> R_D = Eigen::MatrixXd::Identity(NUM_INPUTS, NUM_INPUTS);

  std::vector<Eigen::Matrix<double, NUM_STATES, NUM_STATES>> P(N);
  std::vector<Eigen::Matrix<double, NUM_STATES, 1>> r(N);
  std::vector<Eigen::Matrix<double, NUM_STATES, 1>> a(N);
  std::vector<Eigen::Matrix<double, NUM_INPUTS, NUM_STATES>> K(N);
  std::vector<Eigen::Matrix<double, NUM_INPUTS, NUM_INPUTS>> b(N);
  std::vector<Eigen::Matrix<double, NUM_INPUTS, NUM_INPUTS>> gamma(N);

  // Calculate cost gradients
  for (size_t m = 0; m < N; m++) {
    a[m] = getCostGradient(trajectory[m], trajectory_distribution, distribution);
    b[m] = trajectory[m].dt * R * trajectory[m].input;
    // std::cout << " cost gradients " << m << std::endl;
    // std::cout << " - cost gradient    a[" << m << "]: " << a[m].transpose() << std::endl;
    // std::cout << " - control gradient b[" << m << "]: " << b[m].transpose() << std::endl;
  }

  P[N - 1] = Q_D;
  r[N - 1] = a[N - 1];  /// TODO: get cost gradient

  // Solve LQ Problem with backward pass
  for (int n = N - 2; n > -1; n--) {
    gamma[n] = R_D + (B[n].transpose() * P[n + 1] * B[n]);
    K[n] = gamma[n].inverse() * B[n].transpose() * P[n + 1] * A[n];
    P[n] = Q_D + A[n].transpose() * P[n + 1] * A[n] - K[n].transpose() * gamma[n] * K[n];
    r[n] = a[n] + (A[n].transpose() - K[n].transpose() * B[n].transpose()) * r[n + 1] - K[n + 1].transpose() * b[n];
  }
  // Get Descent direction with forward pass
  z.resize(N);
  v.resize(N);
  for (size_t n = 0; n < N - 1; n++) {
    if (n < 1) {
      z[0] = Eigen::Matrix<double, NUM_STATES, 1>(trajectory[0].position(0), trajectory[0].position(1),
                                                  trajectory[0].position(2));
    }
    v[n] = -K[n] * z[n] - 0.5 * gamma[n].inverse() * (b[n] + B[n].transpose() * r[n]);
    z[n + 1] = A[n] * z[n] + B[n] * v[n];
  }
  std::cout << "Ergodicity: " << distribution.getErgodicity(trajectory_distribution.getCoefficients()) << std::endl;
}

bool ErgodicController::Solve(FourierCoefficient &distribution) {
  std::cout << "Starting to solve ergodic control!" << std::endl;
  if (trajectory_.size() == 0) {
    return false;
  }
  bool exit = false;
  int max_iteration = 100;
  int iter{0};
  while (!exit) {
    SolveSingleIter(distribution);

    std::cout << "Iteration: " << iter << std::endl;
    if (iter > max_iteration) {
      exit = true;
    }
    iter++;
  }
  // 	return xd, ud

  return true;
}

bool ErgodicController::SolveSingleIter(FourierCoefficient &distribution) {
    // Linearize dynamics along the trajectory
    std::vector<Eigen::Matrix<double, NUM_STATES, NUM_STATES>> A;
    std::vector<Eigen::Matrix<double, NUM_STATES, NUM_INPUTS>> B;
    LinearizeDynamics(trajectory_, A, B);

    /// Compute Descent direction
    std::vector<Eigen::Matrix<double, NUM_STATES, 1>> Z;  // State Descent direction
    std::vector<Eigen::Matrix<double, NUM_INPUTS, 1>> U;  // Input Descent direction
    DescentDirection(trajectory_, distribution, A, B, Z, U);
    DescentTrajectory(trajectory_, Z, U);

    /// TODO:  determine step size and descend
    // 		step_size = get_step_size(tm.descender, em, tm, xd, ud, zd, vd, ad, bd, K, i)

    /// TODO: Combine gradient descent
    // 		# descend and project
    // 		xd, ud = project(em, tm, K, xd, ud, zd, vd, step_size)
}

void ErgodicController::DescentTrajectory(std::vector<State> &trajectory,
                                          std::vector<Eigen::Matrix<double, NUM_STATES, 1>> &z,
                                          std::vector<Eigen::Matrix<double, NUM_INPUTS, 1>> &v) {
  double gamma = 1.0;
  for (size_t n = 0; n < trajectory.size(); n++) {
    trajectory[n].position(0) = trajectory[n].position(0) + gamma * z[n](0);
    trajectory[n].position(1) = trajectory[n].position(1) + gamma * z[n](1);
    trajectory[n].position(2) = trajectory[n].position(2) + gamma * z[n](2);
    trajectory[n].input = trajectory[n].input + gamma * v[n](0);
  }
}

void ErgodicController::setInitialTrajectory() {
  // Initial trajectory
  std::vector<State> trajectory;
  double dt = 0.1;
  double T = 10.0;
  double radius = 3.0;
  double omega = 10.0;
  trajectory_.clear();
  for (double t = 0.0; t < T; t += dt) {
    State state;
    state.position = Eigen::Vector3d(radius * std::cos(t * omega), radius * std::sin(t * omega), t * omega);
    state.input = omega;
    state.dt = dt;
    trajectory_.push_back(state);
  }
}