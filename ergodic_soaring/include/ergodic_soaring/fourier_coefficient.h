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
#ifndef FOURIER_COEFFICIENT_H
#define FOURIER_COEFFICIENT_H

#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>

#include <Eigen/Dense>
#include <vector>

struct Settings {
  double center_lat{0.0};
  double center_lon{0.0};
  double delta_easting{100.0};
  double delta_northing{100.0};
  double resolution{10.0};
};

struct State {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};  // x, y, heading
  double input{0.0};                                  // yaw rate
  double dt{0.1};                                     // Time step
};

class FourierCoefficient {
 public:
  FourierCoefficient(int num_coefficients);
  virtual ~FourierCoefficient(){};
  grid_map::GridMap &getGridMap() { return grid_map_; };
  void FourierTransform(grid_map::GridMap &distribution_map);
  void FourierTransform(const std::vector<State> &trajectory);
  void InverseFourierTransform(const std::string layer);
  void setGridMap(grid_map::GridMap &grid_map) { grid_map_ = grid_map; };
  Eigen::ArrayXXd getCoefficients() { return coefficients_; };
  Eigen::ArrayXXd getNormalization() { return normalization_; };
  double getErgodicity(Eigen::ArrayXXd trajectory_coefficients);
  Eigen::Matrix<double, 3, 1> getErgodicGradient(State &state, Eigen::ArrayXXd trajectory_coefficients);

 private:
  inline double BasisFunction(const int k, const double length, const double x) {
    return std::cos(k * M_PI * x / length);
  };

  inline Eigen::Matrix<double, 3, 1> GradientBasisFunction(const int k1, const double length1, const double x1,
                                                           const int k2, const double length2, const double x2) {
    // Gradient Descent for SE(2)
    Eigen::Matrix<double, 3, 1> gradient;
    gradient(0) = -(M_PI / normalization_(k1, k2)) * (k1 / length1) * std::sin(k1 * M_PI * x1 / length1) *
                  std::cos(k2 * M_PI * x2 / length2);
    gradient(1) = -(M_PI / normalization_(k1, k2)) * (k2 / length2) * std::cos(k1 * M_PI * x1 / length1) *
                  std::sin(k2 * M_PI * x2 / length2);
    gradient(2) = 0.0;
    return gradient;
  };

  grid_map::GridMap grid_map_;
  Eigen::ArrayXXd coefficients_;
  Eigen::ArrayXXd normalization_;
  int K_{20};
};

#endif
