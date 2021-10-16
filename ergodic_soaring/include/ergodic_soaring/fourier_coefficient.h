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
  void FourierTransform(std::vector<State> trajectory);
  void InverseFourierTransform(const std::string layer);
  Eigen::ArrayXXd getCoefficients() { return coefficients_; };
  Eigen::ArrayXXd getNormalization() { return normalization_; };
  Eigen::Matrix<double, 3, 1> getErgodicGradient(Eigen::ArrayXXd trajectory_coefficients);

 private:
  inline double BasisFunction(const int k, const double length, const double x) {
    return std::cos(k * M_PI * x / length);
  };
  void generateGaussianDistribution();

  grid_map::GridMap grid_map_;
  Eigen::ArrayXXd coefficients_;
  Eigen::ArrayXXd normalization_;
  int K_{20};
  double v_c_{0.0};
};

#endif
