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

ErgodicController::ErgodicController() {
  grid_map_ = grid_map::GridMap({"distribution", "reconstruction"});
  // Set Gridmap properties
  Settings settings;
  settings.center_lat = 0.0;
  settings.center_lon = 0.0;
  settings.resolution = 0.125;
  settings.delta_easting = 10.0;
  settings.delta_northing = 10.0;
  grid_map_.setFrameId("world");
  grid_map_.setGeometry(grid_map::Length(settings.delta_easting, settings.delta_northing), settings.resolution,
                        grid_map::Position(settings.center_lat, settings.center_lon));
  printf(
      "Created map with size %f x %f m (%i x %i cells).\n The center of the "
      "map is located at (%f, %f) in the %s frame.",
      grid_map_.getLength().x(), grid_map_.getLength().y(), grid_map_.getSize()(0), grid_map_.getSize()(1),
      grid_map_.getPosition().x(), grid_map_.getPosition().y(), grid_map_.getFrameId().c_str());

  // Initialize gridmap layers
  grid_map_["distribution"].setConstant(0);
  grid_map::Matrix &layer_elevation = grid_map_["distribution"];

  double sum{0.0};
  double cell_area = std::pow(grid_map_.getResolution(), 2);
  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridMapIndex = *iterator;
    grid_map::Position cell_pos;
    grid_map_.getPosition(gridMapIndex, cell_pos);

    double sigma = 5.0;
    grid_map::Position mean = Eigen::Vector2d::Zero();
    Eigen::Matrix2d variance = sigma * Eigen::Matrix2d::Identity();
    Eigen::Vector2d error_pos = cell_pos - mean;
    double point_distribution =
        1 / (sigma * std::sqrt(2 * M_PI)) * std::exp(-0.5 * error_pos.transpose() * variance * error_pos);
    sum += point_distribution * cell_area;
    layer_elevation(gridMapIndex(0), gridMapIndex(1)) = point_distribution;
  }

  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridMapIndex = *iterator;
    layer_elevation(gridMapIndex(0), gridMapIndex(1)) = layer_elevation(gridMapIndex(0), gridMapIndex(1)) / sum;
  }
}

ErgodicController::~ErgodicController() {}

FourierCoefficients ErgodicController::FourierTransform(grid_map::GridMap &distribution_map) {
  int K = 10;  // Number of Fourier coefficients
  double L_1 = distribution_map.getLength()[0];
  double L_2 = distribution_map.getLength()[1];
  double cell_area = std::pow(distribution_map.getResolution(), 2);

  FourierCoefficients fourier;
  fourier.coefficents.resize(K * K);
  fourier.normalization.resize(K * K);

  for (size_t i = 0; i < K; i++) {
    for (size_t j = 0; j < K; j++) {
      double h_k_sum{0.0};  // fourier coefficients
      for (grid_map::GridMapIterator iterator(distribution_map); !iterator.isPastEnd(); ++iterator) {
        const grid_map::Index gridMapIndex = *iterator;
        Eigen::Vector2d cell_pos;
        distribution_map.getPosition(gridMapIndex, cell_pos);
        // TODO: Reconstructed sum is always off by a factor of 4
        h_k_sum = h_k_sum + 4 * std::pow(std::cos(i * cell_pos(0) * M_PI / L_1), 2) *
                                std::pow(std::cos(j * cell_pos(1) * M_PI / L_2), 2) * cell_area;
      }
      double h_k = std::sqrt(h_k_sum);

      double phi_k{0.0};  // fourier coefficients
      for (grid_map::GridMapIterator iterator(distribution_map); !iterator.isPastEnd(); ++iterator) {
        const grid_map::Index gridMapIndex = *iterator;
        Eigen::Vector3d cell_pos;
        distribution_map.getPosition3("distribution", gridMapIndex, cell_pos);
        double F_k = (1 / h_k) * BasisFunction(i, L_1, cell_pos(0)) * BasisFunction(j, L_2, cell_pos(1));
        phi_k += cell_pos(2) * F_k * cell_area;
      }
      fourier.normalization[K * i + j] = h_k;
      fourier.coefficents[K * i + j] = phi_k;
    }
  }
  return fourier;
}

FourierCoefficients ErgodicController::FourierTransform(std::vector<Eigen::Vector2d> trajectory) {
  /// WIP: Fourier transformation of trajectory
  int K = 10;  // Number of Fourier coefficients
  FourierCoefficients fourier;
  fourier.coefficents.resize(K * K);
  fourier.normalization.resize(K * K);
  double dt = 0.1;
  double L_1 = grid_map_.getLength()[0];
  double L_2 = grid_map_.getLength()[1];

  fourier.coefficents.resize(K * K);
  fourier.normalization.resize(K * K);

  for (size_t i = 0; i < K; i++) {
    for (size_t j = 0; j < K; j++) {
      double h_k_sum{0.0};  // fourier coefficients
      for (auto pos : trajectory) {
        h_k_sum = h_k_sum + 4 * std::pow(std::cos(i * pos(0) * M_PI / L_1), 2) *
                                std::pow(std::cos(j * pos(1) * M_PI / L_2), 2) * dt;
      }
      double h_k = std::sqrt(h_k_sum);

      double phi_k{0.0};  // fourier coefficients
      for (auto pos : trajectory) {
        double F_k = (1 / h_k) * BasisFunction(i, L_1, pos(0)) * BasisFunction(j, L_2, pos(1));
        phi_k += F_k * dt;
      }
      fourier.normalization[K * i + j] = h_k;
      fourier.coefficents[K * i + j] = phi_k;
    }
  }
  return fourier;
}

void ErgodicController::InverseFourierTransform(FourierCoefficients &fourier, const std::string layer) {
  int K = 10;  // Number of Fourier coefficients
  double L_1 = grid_map_.getLength()[0];
  double L_2 = grid_map_.getLength()[1];

  double cell_area = std::pow(grid_map_.getResolution(), 2);
  grid_map_.add(layer);
  /// TODO: Reconstruct map from fourier transform
  grid_map::Matrix &layer_reconstruction = grid_map_[layer];
  double sum{0.0};
  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridMapIndex = *iterator;
    Eigen::Vector2d cell_pos;
    grid_map_.getPosition(gridMapIndex, cell_pos);

    double phi{0.0};
    for (size_t i = 0; i < K; i++) {
      for (size_t j = 0; j < K; j++) {
        double h_k = fourier.normalization[K * i + j];
        phi += fourier.coefficents[K * i + j] * (1 / h_k) * BasisFunction(i, L_1, cell_pos(0)) *
               BasisFunction(j, L_2, cell_pos(1));
      }
    }
    layer_reconstruction(gridMapIndex(0), gridMapIndex(1)) = phi;
    sum += phi * cell_area;
  }
  // TODO: Reconstructed sum is always off by a factor of 4
  std::cout << "Reconstructed sum: " << sum << std::endl;
}

bool ErgodicController::Solve() {
  // From
  // https://github.com/dressel/ErgodicControl.jl/blob/9db31ac4d113f58f60b0bdb6a659698bb867b95d/src/trajectory_generation/pto.jl
  // 	while not_finished

  State state;

  bool exit = false;

  while (exit) {
    // Linearize dynamics
    // 		A, B = linearize(tm.dynamics, xd, ud, tm.h)
    std::vector<Eigen::Matrix3d> A;
    std::vector<Eigen::Vector3d> B;
    LinearizeDynamics(state, A[0], B[0]);

    /// TODO: Compute Descent direction
    // 		K, C = LQ(A, B, ad, bd, tm.Qn, tm.Rn, tm.N)
    // 		zd, vd = apply_LQ_gains(A, B, K, C)
    std::vector<Eigen::Matrix3d> K, C;
    LQ(state, A, B, K, C);

    /// TODO: Compute Gains for the projection operator

    /// TODO: Step size and descent direction is selected
    // 		# determine step size and descend
    // 		step_size = get_step_size(tm.descender, em, tm, xd, ud, zd, vd, ad, bd, K, i)

    /// TODO: Combine gradient descent
    // 		# descend and project
    // 		xd, ud = project(em, tm, K, xd, ud, zd, vd, step_size)

    exit = true;
  }
  // 	return xd, ud

  return true;
}

void ErgodicController::LinearizeDynamics(State state, Eigen::Matrix3d &A, Eigen::Vector3d &B) {
  double v0 = 15.0;
  double theta = state.heading;
  double dt = 0.1;
  // tilda(A_n)
  A << 1.0, 0.0, -sin(theta) * v0 * dt, 0.0, 1.0, cos(theta) * v0 * dt, 0.0, 0.0, 1.0;
  // tilda(B_n)
  B << 0.0, 0.0, dt;
}

void ErgodicController::LQ(State state, std::vector<Eigen::Matrix3d> &A, std::vector<Eigen::Vector3d> &B,
                           std::vector<Eigen::Matrix3d> &K, std::vector<Eigen::Matrix3d> &C) {
  /// TODO: Solve LQ problem
  const int N = 10;
  Eigen::Matrix3d Q_D = Eigen::Matrix3d::Identity();
  double R_D = 1.0;

  std::vector<Eigen::Matrix3d> P;
  std::vector<double> G;
  std::vector<Eigen::Matrix3d> r;
  std::vector<Eigen::Matrix3d> b;
  P.resize(N);
  P[N - 1] = Q_D;
  // r[N] = a_N;

  for (size_t n = N - 2; n > -1; n--) {
    // G[n] = R_D + (B[n].transpose() * P[n + 1] * B[n]);
    // K[n] = 1/G[n] * B[n].transpose() * P[n + 1] * A[n];
    // P[n] = Q_D + (A[n].transpose() * P[n + 1] * A[n]) - (K[n].transpose() * G[n] * K[n]);
    // r[n] = (A[n].transpose() - K[n] * B[n].transpose()) * r[n + 1] - K[n + 1].transpose() * b[n];
  }
}
