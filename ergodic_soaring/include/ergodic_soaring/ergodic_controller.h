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
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  double heading{0.0};
};

class ErgodicController {
 public:
  ErgodicController();
  virtual ~ErgodicController();
  grid_map::GridMap &getGridMap() { return grid_map_; };
  void FourierTransform();
  void InverseFourierTransform();

 private:
  bool Solve();
  void LinearizeDynamics(State state, Eigen::Matrix3d &A, Eigen::Vector3d &B);
  void LQ(State state, std::vector<Eigen::Matrix3d> &A, std::vector<Eigen::Vector3d> &B,
          std::vector<Eigen::Matrix3d> &K, std::vector<Eigen::Matrix3d> &C);

  grid_map::GridMap grid_map_;
  std::vector<double> fourier_coefficents_;
  std::vector<double> fourier_normalization_;
};

#endif