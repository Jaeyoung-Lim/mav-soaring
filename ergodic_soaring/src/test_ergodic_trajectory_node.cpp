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

#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <grid_map_ros/GridMapRosConverter.hpp>

geometry_msgs::PoseStamped vector3d2PoseStampedMsg(const Eigen::Vector3d position, const Eigen::Vector4d orientation) {
  geometry_msgs::PoseStamped encode_msg;

  encode_msg.header.stamp = ros::Time::now();
  encode_msg.header.frame_id = "map";
  encode_msg.pose.orientation.w = orientation(0);
  encode_msg.pose.orientation.x = orientation(1);
  encode_msg.pose.orientation.y = orientation(2);
  encode_msg.pose.orientation.z = orientation(3);
  encode_msg.pose.position.x = position(0);
  encode_msg.pose.position.y = position(1);
  encode_msg.pose.position.z = position(2);
  return encode_msg;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_ergodic_map");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  ros::Publisher grid_map_pub = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  ros::Publisher trajectory_pub_ = nh.advertise<nav_msgs::Path>("path", 10);

  grid_map::GridMap grid_map_ = grid_map::GridMap({"distribution"});

  // Set Gridmap properties
  grid_map_.setFrameId("world");
  double resolution = 0.1;
  grid_map_.setGeometry(grid_map::Length(10.0, 10.0), resolution, grid_map::Position(5.0, 5.0));

  /// Generate circular trajectory as an example
  double T = 40.0;
  double radius = 3.0;
  double omega = 2 * M_PI / (4 * T);
  double dt = 0.1;
  std::vector<State> trajectory;

  for (double t = 0.0; t < T; t += dt) {
    State state;
    state.position = Eigen::Vector3d(radius * std::cos(t * omega) + 5.0, radius * std::sin(t * omega) + 5.0, 0.0);
    state.dt = dt;
    trajectory.push_back(state);
  }

  std::shared_ptr<FourierCoefficient> fourier_coefficient = std::make_shared<FourierCoefficient>(20);
  fourier_coefficient->setGridMap(grid_map_);

  fourier_coefficient->FourierTransform(trajectory);
  fourier_coefficient->InverseFourierTransform("distribution");

  while (true) {
    std::vector<geometry_msgs::PoseStamped> trajectory_vector;
    Eigen::Vector4d vehicle_attitude(1.0, 0.0, 0.0, 0.0);
    for (auto state : trajectory) {
      trajectory_vector.insert(
          trajectory_vector.end(),
          vector3d2PoseStampedMsg(Eigen::Vector3d(state.position(0), state.position(1), 2.0), vehicle_attitude));
    }

    nav_msgs::Path msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.poses = trajectory_vector;
    trajectory_pub_.publish(msg);

    fourier_coefficient->getGridMap().setTimestamp(ros::Time::now().toNSec());
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(fourier_coefficient->getGridMap(), message);
    grid_map_pub.publish(message);
    ros::Duration(10.0).sleep();
  }

  ros::spin();
  return 0;
}
