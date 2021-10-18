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
#include "ergodic_soaring/fourier_coefficient.h"

#include <ros/ros.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

void generateGaussianDistribution(grid_map::GridMap &grid_map) {
  double sum{0.0};
  double v_c_{0.0};
  double cell_area = std::pow(grid_map.getResolution(), 2);

  grid_map::Matrix &layer_elevation = grid_map["distribution"];

  for (grid_map::GridMapIterator iterator(grid_map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridMapIndex = *iterator;
    grid_map::Position cell_pos;
    grid_map.getPosition(gridMapIndex, cell_pos);

    double sigma = 5.0;
    grid_map::Position mean = Eigen::Vector2d::Zero();
    Eigen::Matrix2d variance = sigma * Eigen::Matrix2d::Identity();
    Eigen::Vector2d error_pos = cell_pos - mean;
    double point_distribution = 1 / (std::sqrt(std::pow(2 * M_PI, 2) * variance.norm())) *
                                std::exp(-0.5 * error_pos.transpose() * variance * error_pos);
    sum += point_distribution;
    layer_elevation(gridMapIndex(0), gridMapIndex(1)) = point_distribution;
  }
  v_c_ = 1 / sum;
  double normailized_sum{0.0};

  for (grid_map::GridMapIterator iterator(grid_map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridMapIndex = *iterator;
    layer_elevation(gridMapIndex(0), gridMapIndex(1)) = layer_elevation(gridMapIndex(0), gridMapIndex(1)) * v_c_;
    normailized_sum += layer_elevation(gridMapIndex(0), gridMapIndex(1));
  }
  std::cout << std::endl;
  std::cout << "Genearating sample distribution=====" << std::endl;
  std::cout << "  - Sum of distribution: " << sum << std::endl;
  std::cout << "  - normalized_sum: " << normailized_sum << std::endl;
  std::cout << "  - V_c: :" << v_c_ << std::endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_ergodic_map");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  ros::Publisher grid_map_pub = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  std::shared_ptr<ErgodicController> ergodic_controller = std::make_shared<ErgodicController>();
  // Fourier coefficients of the distribution
  FourierCoefficient target_distribution = FourierCoefficient(20);

  // Generate Target distribution
  grid_map::GridMap grid_map_ = grid_map::GridMap({"distribution"});

  // Set Gridmap properties
  Settings settings;
  settings.center_lat = 0.0;
  settings.center_lon = 0.0;
  settings.resolution = 0.1;
  settings.delta_easting = 10.0;
  settings.delta_northing = 10.0;
  grid_map_.setFrameId("world");
  grid_map_.setGeometry(grid_map::Length(settings.delta_easting, settings.delta_northing), settings.resolution,
                        grid_map::Position(settings.center_lat, settings.center_lon));

  generateGaussianDistribution(grid_map_);

  target_distribution.FourierTransform(grid_map_);

  // Create trajectory that matches the target distribution
  ergodic_controller->Solve(target_distribution);

  while (true) {
    target_distribution.getGridMap().setTimestamp(ros::Time::now().toNSec());
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(target_distribution.getGridMap(), message);
    grid_map_pub.publish(message);
    ros::Duration(10.0).sleep();
  }

  ros::spin();
  return 0;
}
