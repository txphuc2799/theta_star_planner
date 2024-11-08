// Copyright 2020 Anshumaan Singh
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef THETA_STAR_PLANNER__THETA_STAR_PLANNER_HPP_
#define THETA_STAR_PLANNER__THETA_STAR_PLANNER_HPP_

#include <iostream>
#include <cmath>
#include <string>
#include <chrono>
#include <queue>
#include <algorithm>
#include <memory>
#include <vector>
#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/cost_values.h>
#include <theta_star_planner/theta_star.hpp>
#include <theta_star_planner/utils/geometry_utils.hpp>
#include <smoother/smoother.hpp>


namespace theta_star_planner
{

class ThetaStarPlanner : public nav_core::BaseGlobalPlanner
{
public:
  ThetaStarPlanner();
  ~ThetaStarPlanner();

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  bool makePlan(const geometry_msgs::PoseStamped& start, 
    const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

  std::vector<geometry_msgs::PoseStamped> pathToPosesStamped(
    const nav_msgs::Path &path,
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal);

  bool execSmoothPath(std::vector<geometry_msgs::PoseStamped> &plan);

protected:
  tf2_ros::Buffer* tf_;
  std::string global_frame_, name_;
  ros::Publisher pub_raw_path_;
  ros::Publisher pub_smooth_path_;
  bool use_final_approach_orientation_;
  bool initialized_;
  bool use_smoother_;
  std::string smooth_plugin_name_;

  pluginlib::ClassLoader<smoother::Smoother> smoother_loader_;
  std::shared_ptr<smoother::Smoother> smoother_;

  std::unique_ptr<theta_star::ThetaStar> planner_;

  /**
   * @brief the function responsible for calling the algorithm and retrieving a path from it
   * @param cancel_checker is a function to check if the action has been canceled
   * @return global_path is the planned path to be taken
   */
  bool getPlan(nav_msgs::Path & global_path);

  /**
   * @brief interpolates points between the consecutive waypoints of the path
   * @param raw_path is used to send in the path received from the planner
   * @param dist_bw_points is used to send in the interpolation_resolution (which has been set as the costmap resolution)
   * @return the final path with waypoints at a distance of the value of interpolation_resolution of each other
   */
  static nav_msgs::Path linearInterpolation(
    const std::vector<coordsW> & raw_path,
    const double & dist_bw_points);
};
}   //  namespace theta_star_planner

#endif  //  THETA_STAR_PLANNER__THETA_STAR_PLANNER_HPP_
