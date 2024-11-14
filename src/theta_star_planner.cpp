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

#include <vector>
#include <memory>
#include <string>
#include <theta_star_planner/theta_star_planner.hpp>
#include <theta_star_planner/theta_star.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>

PLUGINLIB_EXPORT_CLASS(theta_star_planner::ThetaStarPlanner, nav_core::BaseGlobalPlanner)

namespace theta_star_planner
{
ThetaStarPlanner::ThetaStarPlanner()
: smoother_loader_("smoother", "smoother::Smoother"),
  initialized_(false)
{
}

ThetaStarPlanner::~ThetaStarPlanner()
{
  planner_.reset();
  smoother_.reset();
}

void ThetaStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!initialized_) {
    ros::NodeHandle private_nh("~/" + name);
    planner_ = std::make_unique<theta_star::ThetaStar>();
    planner_->costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();

    private_nh.param("how_many_corners", planner_->how_many_corners_, 8);

    if (planner_->how_many_corners_ != 8 && planner_->how_many_corners_ != 4) {
      planner_->how_many_corners_ = 8;
      ROS_WARN("Your value for - how_many_corners  was overridden, and is now set to 8");
    }

    private_nh.param("allow_unknown", planner_->allow_unknown_, true);
    private_nh.param("w_euc_cost", planner_->w_euc_cost_, 1.0);
    private_nh.param("w_traversal_cost", planner_->w_traversal_cost_, 2.0);
    planner_->w_heuristic_cost_ = planner_->w_euc_cost_ < 1.0 ? planner_->w_euc_cost_ : 1.0;

    private_nh.param("terminal_checking_interval", planner_->terminal_checking_interval_, 5000);
    private_nh.param("use_final_approach_orientation", use_final_approach_orientation_, false);

    // Plugins
    private_nh.param("use_smoother", use_smoother_, true);
    private_nh.param("smooth_plugin_name", smooth_plugin_name_, std::string("smoother::SimpleSmoother"));
    ROS_INFO("ThetaStarPlanner: Using Smoother [%s]", smooth_plugin_name_.c_str());
    smoother_ = std::move(smoother_loader_.createUniqueInstance(smooth_plugin_name_));
    smoother_->initialize(private_nh, costmap_ros->getCostmap());

    pub_raw_path_ = private_nh.advertise<nav_msgs::Path>("raw_path", 1);
    pub_smooth_path_ = private_nh.advertise<nav_msgs::Path>("smooth_path", 1);

    ROS_INFO("Initialized Theta star planner.");
    initialized_ = true;
  }
}

bool ThetaStarPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
  const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
  boost::mutex::scoped_lock lock(mutex_);
  nav_msgs::Path global_path;

  //clear the plan, just in case
  plan.clear();

  //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
  if(goal.header.frame_id != global_frame_){
    ROS_ERROR("ThetaStarPlanner: "
    "The goal pose passed to this planner must be in the %s frame. "
    "It is instead in the %s frame.", 
    global_frame_.c_str(), goal.header.frame_id.c_str());
    return false;
  }

  if(start.header.frame_id != global_frame_){
    ROS_ERROR("ThetaStarPlanner: "
    "The start pose passed to this planner must be in the %s frame. "
    "It is instead in the %s frame.", 
    global_frame_.c_str(), start.header.frame_id.c_str());
    return false;
  }

  // Corner case of start and goal beeing on the same cell
  unsigned int mx_start, my_start, mx_goal, my_goal;
  if (!planner_->costmap_->worldToMap(
      start.pose.position.x, start.pose.position.y, mx_start, my_start))
  {
    ROS_WARN_THROTTLE(
    1.0, "ThetaStarPlanner: Start Coordinates of(%f, %f) was outside bounds",
    start.pose.position.x, start.pose.position.y);
    return false;
  }

  if (!planner_->costmap_->worldToMap(
      goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal))
  {
    ROS_WARN_THROTTLE(
    1.0, "ThetaStarPlanner: Goal Coordinates of(%f, %f) was outside bounds",
    goal.pose.position.x, goal.pose.position.y);
    return false;
  }

  if (planner_->costmap_->getCost(mx_goal, my_goal) == costmap_2d::LETHAL_OBSTACLE) {
    ROS_WARN_THROTTLE(
    1.0, "ThetaStarPlanner: Goal Coordinates of(%f, %f) was in lethal cost!",
    goal.pose.position.x, goal.pose.position.y);
    return false;
  }

  if (start.pose.position.x == goal.pose.position.x
      && start.pose.position.y == goal.pose.position.y) {
    global_path.header.stamp = ros::Time::now();
    global_path.header.frame_id = global_frame_;
    geometry_msgs::PoseStamped pose;
    pose.header = global_path.header;
    pose.pose.position.z = 0.0;
    pose.pose = goal.pose;
    plan.push_back(pose);

  } else {
    planner_->clearStart();
    planner_->setStartAndGoal(start, goal);
    ROS_DEBUG(
      "ThetaStarPlanner: Got the src and dst... (%i, %i) && (%i, %i)",
      planner_->src_.x, planner_->src_.y, planner_->dst_.x, planner_->dst_.y);
    if (!getPlan(global_path)) {
      plan.clear();
      return false;
    }

    // Publish raw path
    pub_raw_path_.publish(global_path);

    plan = pathToPosesStamped(global_path, start, goal);

    if (use_smoother_ && !plan.empty()) {
      if (!execSmoothPath(plan)) {
        return false;
      }
    }
  }
  
  return !plan.empty();
}

std::vector<geometry_msgs::PoseStamped>
ThetaStarPlanner::pathToPosesStamped(
  const nav_msgs::Path &path,
  const geometry_msgs::PoseStamped& start,
  const geometry_msgs::PoseStamped& goal)
{
  std::vector<geometry_msgs::PoseStamped> plan;
  
  // check if a plan is generated
  size_t plan_size = path.poses.size();
  if (plan_size > 0) {
    ros::Time plan_time = ros::Time::now();
    for(int i = 0; i <= path.poses.size()-1; ++i){
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = plan_time;
      pose.header.frame_id = global_frame_;
      pose.pose.position.x = path.poses[i].pose.position.x;
      pose.pose.position.y = path.poses[i].pose.position.y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      plan.push_back(pose);
    }
    
    geometry_msgs::PoseStamped goal_copy;
    goal_copy = goal;
    goal_copy.header.stamp = ros::Time::now();
    plan.push_back(goal_copy);

    // If use_final_approach_orientation=true, interpolate the last pose orientation from the
    // previous pose to set the orientation to the 'final approach' orientation of the robot so
    // it does not rotate.
    // And deal with corner case of plan of length 1
    if (use_final_approach_orientation_) {
      if (plan_size == 1) {
        plan.back().pose.orientation = start.pose.orientation;
      } else if (plan_size > 1) {
        double dx, dy, theta;
        auto last_pose = plan.back().pose.position;
        auto approach_pose = plan[plan_size - 2].pose.position;
        dx = last_pose.x - approach_pose.x;
        dy = last_pose.y - approach_pose.y;
        theta = atan2(dy, dx);
        plan.back().pose.orientation =
          theta_star_planner::geometry_utils::orientationAroundZAxis(theta);
      }
    }
  } else {
    ROS_ERROR("ThetaStarPlanner: Can't find path because plan size = 0");
  }
  return plan;
}

bool ThetaStarPlanner::execSmoothPath(std::vector<geometry_msgs::PoseStamped> &plan)
{
  nav_msgs::Path path;
  path.poses.resize(plan.size());
  path.header = plan[0].header;

  for (unsigned int i = 0; i < plan.size(); i++) {
    path.poses[i] = plan[i];
  }

  if (!smoother_->execSmoothPath(path)) {
    return false;
  }

  pub_smooth_path_.publish(path);
  plan = path.poses;
  
  return true;
}

bool ThetaStarPlanner::getPlan(nav_msgs::Path & global_path)
{
  std::vector<coordsW> path;
  if (planner_->isUnsafeToPlan()) {
    global_path.poses.clear();
    ROS_WARN("ThetaStarPlanner: Either of the start or goal pose are an obstacle!");
    return false;
  } else if (planner_->generatePath(path)) {
    global_path = linearInterpolation(path, planner_->costmap_->getResolution());
  } else {
    global_path.poses.clear();
    ROS_ERROR("ThetaStarPlanner: Could not generate path between the given poses");
    return false;
  }
  global_path.header.stamp = ros::Time::now();
  global_path.header.frame_id = global_frame_;
  return true;
}

nav_msgs::Path ThetaStarPlanner::linearInterpolation(
  const std::vector<coordsW> & raw_path,
  const double & dist_bw_points)
{
  nav_msgs::Path pa;

  geometry_msgs::PoseStamped p1;
  for (unsigned int j = 0; j < raw_path.size() - 1; j++) {
    coordsW pt1 = raw_path[j];
    p1.pose.position.x = pt1.x;
    p1.pose.position.y = pt1.y;
    pa.poses.push_back(p1);

    coordsW pt2 = raw_path[j + 1];
    double distance = std::hypot(pt2.x - pt1.x, pt2.y - pt1.y);
    int loops = static_cast<int>(distance / dist_bw_points);
    double sin_alpha = (pt2.y - pt1.y) / distance;
    double cos_alpha = (pt2.x - pt1.x) / distance;
    for (int k = 1; k < loops; k++) {
      p1.pose.position.x = pt1.x + k * dist_bw_points * cos_alpha;
      p1.pose.position.y = pt1.y + k * dist_bw_points * sin_alpha;
      pa.poses.push_back(p1);
    }
  }

  return pa;
}

}  // namespace theta_star_planner