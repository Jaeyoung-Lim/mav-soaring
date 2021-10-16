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

#include "ergodic_soaring/fourier_coefficient.h"

#include <ros/ros.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_ergodic_map");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  ros::Publisher grid_map_pub = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

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

  /// Generate circular trajectory as an example
  double T = 20.0;
  double radius = 3.0;
  double omega = 10.0;
  double dt = 0.1;
  std::vector<State> trajectory;

  for (double t = 0.0; t < T; t += dt) {
    State state;
    state.position = Eigen::Vector3d(radius * std::cos(t * omega), radius * std::sin(t * omega), t * omega);
    trajectory.push_back(state);
  }

  std::shared_ptr<FourierCoefficient> fourier_coefficient = std::make_shared<FourierCoefficient>(20);
  fourier_coefficient->setGridMap(grid_map_);

  fourier_coefficient->FourierTransform(trajectory);
  fourier_coefficient->InverseFourierTransform("distribution");

  while (true) {
    fourier_coefficient->getGridMap().setTimestamp(ros::Time::now().toNSec());
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(fourier_coefficient->getGridMap(), message);
    grid_map_pub.publish(message);
    ros::Duration(10.0).sleep();
  }

  ros::spin();
  return 0;
}
