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

void ErgodicController::setInitialTrajectory() {
  // Initial trajectory
  std::vector<State> trajectory;
  double dt = 0.1;
  double T = 10.0;
  Eigen::Matrix<double, NUM_INPUTS, 1> omega;
  omega << 1.0;
  trajectory_.clear();
  State initial_state;
  initial_state.position = Eigen::Vector3d(20.0, 20.0, 0.0);
  initial_state.input = omega(0);
  initial_state.dt = dt;
  trajectory_.push_back(initial_state);
  for (double t = 0.0; t < 0.5 * T; t += dt) {
    trajectory_.push_back(Dynamics(trajectory_.back(), omega));
  }
  for (double t = 0.5 * T; t < T; t += dt) {
    trajectory_.push_back(Dynamics(trajectory_.back(), -omega));
  }
}

void ErgodicController::LinearizeDynamics(const double cruise_speed, const double dt, const Eigen::Vector3d &pos, const double input,
                                          Eigen::Matrix<double, NUM_STATES, NUM_STATES> &A,
                                          Eigen::Matrix<double, NUM_STATES, NUM_INPUTS> &B) {  // Dubins plane
  double yaw = pos(2);
  // Linearize dynamics
  A << 1.0, 0.0, dt * cruise_speed * std::cos(yaw), 0.0, 1.0, dt * cruise_speed * std::sin(yaw), 0.0, 0.0, 1.0;
  B << 0.0, 0.0, dt;
}

void ErgodicController::LinearizeTrajectory(
    const double cruise_speed, const std::vector<State> &trajectory,
    std::vector<Eigen::Matrix<double, NUM_STATES, NUM_STATES>> &A,
    std::vector<Eigen::Matrix<double, NUM_STATES, NUM_INPUTS>> &B) {  // Dubins plane

  for (const auto &state : trajectory) {
    Eigen::Matrix<double, NUM_STATES, NUM_STATES> An;
    Eigen::Matrix<double, NUM_STATES, NUM_INPUTS> Bn;
    LinearizeDynamics(cruise_speed, state.dt, state.position, state.input, An, Bn);
    A.push_back(An);
    B.push_back(Bn);
  }
}

State ErgodicController::Dynamics(const State &state,
                                  const Eigen::Matrix<double, NUM_INPUTS, 1> &input) {  // Dubins plane
  double dt = state.dt;
  Eigen::Vector3d current_position = state.position;
  /// TODO: Implement yaw as a seperate state
  State next_state;
  Eigen::Vector3d next_position;

  double yaw = current_position(2);
  double next_yaw = yaw + input(0) * dt;
  if (std::abs(input(0)) <= __FLT_EPSILON__) {
    next_position(0) = current_position(0) + cruise_speed_ * dt * std::cos(yaw);
    next_position(1) = current_position(1) + cruise_speed_ * dt * std::sin(yaw);
  } else {
    next_position(0) = current_position(0) + (cruise_speed_ / input(0)) * (std::sin(next_yaw) - std::sin(yaw));
    next_position(1) = current_position(1) + (cruise_speed_ / input(0)) * (-std::cos(next_yaw) + std::cos(yaw));
  }
  next_position(2) = next_yaw;

  next_state.position = next_position;
  next_state.dt = dt;
  next_state.input = input(0);

  return next_state;
}

Eigen::Matrix<double, NUM_STATES, 1> ErgodicController::getCostGradient(const int N, const State &state,
                                                                        FourierCoefficient &trajectory_distribution,
                                                                        FourierCoefficient &distribution) {
  Eigen::Matrix<double, NUM_STATES, 1> gradient;
  /// TODO: Tune Boundary from map
  double c_b = 1.0;  // Boundary barrier weight
  double q = 1.0;    // Ergodicity weight

  double max_boundary = 100;
  double min_boundary = 0;
  Eigen::Matrix<double, NUM_STATES, 1> boundary_gradient = Eigen::Matrix<double, NUM_STATES, 1>::Zero();
  for (int i = 0; i < 2; i++) {
    boundary_gradient(i) +=
        2.0 * (std::max(state.position(i) - max_boundary, 0.0) + std::min(state.position(i), min_boundary));
  }

  Eigen::Matrix<double, NUM_STATES, 1> ergodic_gradient;
  ergodic_gradient = trajectory_distribution.getErgodicGradient(N, state, distribution.getCoefficients());
  // std::cout << "   - ergodic_gradient: " << ergodic_gradient.transpose() << std::endl;
  gradient = q * ergodic_gradient + c_b * boundary_gradient;
  // std::cout << "ergodic_gradient: " << ergodic_gradient.transpose() << " boundary_gradient: " <<
  // boundary_gradient.transpose() << std::endl;
  // Since ergodicity is only calculated in R^2 we need to project it down
  gradient(2) = 0.0;
  return gradient;
}

void ErgodicController::DescentDirection(const std::vector<State> &trajectory, FourierCoefficient &distribution,
                                         std::vector<Eigen::Matrix<double, NUM_STATES, NUM_STATES>> &A,
                                         std::vector<Eigen::Matrix<double, NUM_STATES, NUM_INPUTS>> &B,
                                         std::vector<Eigen::Matrix<double, NUM_STATES, 1>> &z,
                                         std::vector<Eigen::Matrix<double, NUM_INPUTS, 1>> &v,
                                         std::vector<Eigen::Matrix<double, NUM_INPUTS, NUM_STATES>> &K) {
  FourierCoefficient trajectory_distribution(distribution.getNumberOfCoefficients());
  trajectory_distribution.setGridMap(distribution.getGridMap());
  trajectory_distribution.FourierTransform(trajectory);  // Transform trajectory distribution after update

  // Cost function
  Eigen::Matrix<double, NUM_INPUTS, NUM_INPUTS> R = Eigen::MatrixXd::Identity(NUM_INPUTS, NUM_INPUTS);

  // Solve gradient discent direction that minimizes the cost function
  const int N = trajectory.size();
  Eigen::Matrix<double, NUM_STATES, NUM_STATES> Q_D = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);
  Eigen::Matrix<double, NUM_INPUTS, NUM_INPUTS> R_D = 0.01 * Eigen::MatrixXd::Identity(NUM_INPUTS, NUM_INPUTS);

  std::vector<Eigen::Matrix<double, NUM_STATES, NUM_STATES>> P(N);
  std::vector<Eigen::Matrix<double, NUM_STATES, 1>> r(N);
  std::vector<Eigen::Matrix<double, NUM_STATES, 1>> a(N);
  std::vector<Eigen::Matrix<double, NUM_INPUTS, NUM_INPUTS>> b(N);
  std::vector<Eigen::Matrix<double, NUM_INPUTS, NUM_INPUTS>> gamma(N);

  // Calculate cost gradients
  for (size_t m = 0; m < N; m++) {
    // std::cout << " cost gradients " << m << std::endl;
    a[m] = getCostGradient(N, trajectory[m], trajectory_distribution, distribution);
    b[m] = trajectory[m].dt * R * trajectory[m].input;
  }

  P[N - 1] = Q_D;
  r[N - 1] = a[N - 1];

  // Solve LQ Problem with backward pass
  K.resize(N);
  for (int n = N - 2; n > -1; n--) {
    gamma[n] = R_D + (B[n].transpose() * P[n + 1] * B[n]);
    K[n] = gamma[n].inverse() * B[n].transpose() * P[n + 1] * A[n];
    P[n] = Q_D + A[n].transpose() * P[n + 1] * A[n] - K[n].transpose() * gamma[n] * K[n];
    r[n] = a[n] + (A[n].transpose() - K[n].transpose() * B[n].transpose()) * r[n + 1] - K[n].transpose() * b[n];
  }
  // Get Descent direction with forward pass
  z.resize(N);
  v.resize(N);
  for (size_t n = 0; n < N - 1; n++) {
    if (n < 1) {
      z[0] = Eigen::Matrix<double, NUM_STATES, 1>::Zero();
    }
    v[n] = -K[n] * z[n] - 0.5 * gamma[n].inverse() * (b[n] + B[n].transpose() * r[n + 1]);
    z[n + 1] = A[n] * z[n] + B[n] * v[n];
  }
}

bool ErgodicController::Solve(FourierCoefficient &distribution) {
  std::cout << "Starting to solve ergodic control!" << std::endl;
  if (trajectory_.size() == 0) {
    return false;
  }
  bool exit = false;
  int max_iteration = 100;
  int iter{1};
  while (!exit) {
    SolveSingleIter(distribution, iter);

    std::cout << "Iteration: " << iter << std::endl;
    if (iter > max_iteration) {
      exit = true;
    }
    iter++;
  }
  return true;
}

void ErgodicController::SolveSingleIter(FourierCoefficient &distribution, int i) {
  // Linearize dynamics along the trajectory
  std::vector<Eigen::Matrix<double, NUM_STATES, NUM_STATES>> A;
  std::vector<Eigen::Matrix<double, NUM_STATES, NUM_INPUTS>> B;
  LinearizeTrajectory(cruise_speed_, trajectory_, A, B);

  /// Compute Descent direction
  std::vector<Eigen::Matrix<double, NUM_INPUTS, NUM_STATES>> K;
  std::vector<Eigen::Matrix<double, NUM_STATES, 1>> Z;  // State Descent direction
  std::vector<Eigen::Matrix<double, NUM_INPUTS, 1>> U;  // Input Descent direction
  DescentDirection(trajectory_, distribution, A, B, Z, U, K);

  // Update trajectory with gradient descent direction
  double gamma_0 = 1.0;
  // double gamma = gamma_0 * std::pow(0.5, i);
  double gamma = gamma_0;
  std::vector<Eigen::Matrix<double, NUM_STATES, 1>> Alpha = Z;  // State Descent direction
  std::vector<Eigen::Matrix<double, NUM_INPUTS, 1>> Mu = U;     // Input Descent direction
  DescentTrajectory(trajectory_, Alpha, Mu, Z, U, gamma);
  /// TODO: Why is the last trajectory and the preprojected trajectory the same?
  preprojected_trajectory_.clear();
  for (size_t k = 0; k < trajectory_.size(); k++) {
    State state;
    state.position(0) = Alpha[k](0);
    state.position(1) = Alpha[k](1);
    state.position(2) = Alpha[k](2);
    state.input = Mu[k](0);
    preprojected_trajectory_.push_back(state);
  }
  last_trajectory_ = trajectory_;
  ProjectionOperator(trajectory_, Alpha, Mu, K);
}

void ErgodicController::DescentTrajectory(const std::vector<State> &trajectory,
                                          std::vector<Eigen::Matrix<double, NUM_STATES, 1>> &alpha,
                                          std::vector<Eigen::Matrix<double, NUM_INPUTS, 1>> &mu,
                                          std::vector<Eigen::Matrix<double, NUM_STATES, 1>> &z,
                                          std::vector<Eigen::Matrix<double, NUM_INPUTS, 1>> &v, const double gamma) {
  for (size_t n = 0; n < trajectory.size(); n++) {
    alpha[n](0) = trajectory[n].position(0) + gamma * z[n](0);
    alpha[n](1) = trajectory[n].position(1) + gamma * z[n](1);
    alpha[n](2) = trajectory[n].position(2) + gamma * z[n](2);
    mu[n](0) = trajectory[n].input + gamma * v[n](0);
  }
}

void ErgodicController::ProjectionOperator(std::vector<State> &trajectory,
                                           std::vector<Eigen::Matrix<double, NUM_STATES, 1>> &alpha,
                                           std::vector<Eigen::Matrix<double, NUM_INPUTS, 1>> &mu,
                                           std::vector<Eigen::Matrix<double, NUM_INPUTS, NUM_STATES>> &K) {
  // Projection operator to handle nonlinear dynamics constraint
  const int N = trajectory.size();
  double max_u = 2.0;
  double min_u = -2.0;

  for (size_t n = 0; n < N - 1; n++) {
    Eigen::Matrix<double, NUM_INPUTS, 1> u_n = mu[n] + K[n] * (alpha[n] - trajectory[n].position);
    u_n(0) = std::min(std::max(u_n(0), min_u), max_u);
    trajectory[n + 1] = Dynamics(trajectory[n], u_n);
  }
}
