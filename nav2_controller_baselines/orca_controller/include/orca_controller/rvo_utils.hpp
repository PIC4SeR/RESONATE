// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ORCA_CONTROLLER__RVO_UTILS_HPP_
#define ORCA_CONTROLLER__RVO_UTILS_HPP_

#include <algorithm>
#include <chrono>
#include <exception>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "RVO.h"
#include "angles/angles.h"
#include "builtin_interfaces/msg/time.hpp"
#include "common_utils/utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "people_msgs/msg/people.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

typedef RVO::Vector2 Vector2d;

namespace orca_controller::rvo_utils
{

struct AgentConfig
{
  AgentConfig(
    double neighbor_dist, int max_neighbors, double time_horizon, double time_horizon_obst,
    double radius, double max_speed)
  : neighbor_dist(neighbor_dist),
    max_neighbors(max_neighbors),
    time_horizon(time_horizon),
    time_horizon_obst(time_horizon_obst),
    radius(radius),
    max_speed(max_speed)
  {
  }
  AgentConfig()
  : neighbor_dist(0.0),
    max_neighbors(0),
    time_horizon(0.0),
    time_horizon_obst(0.0),
    radius(0.0),
    max_speed(0.0)
  {
  }

public:
  double neighbor_dist;
  int max_neighbors;
  double time_horizon;
  double time_horizon_obst;
  double radius;
  double max_speed;
};

class RVOAgent
{
public:
  RVOAgent()
  : id_(UINT_MAX), position_(0.0, 0.0), velocity_(0.0, 0.0), goal_(0.0, 0.0), config_()
  {
  }
  RVOAgent(
    const uint id, const Vector2d & position, const Vector2d & velocity,
    const float naive_goal_time, const AgentConfig & config)
  : id_(id), position_(position), velocity_(velocity), config_(config)
  {
    goal_ = position + velocity * naive_goal_time;
  }

  RVOAgent(
    const uint id, const Vector2d & position, const Vector2d & velocity, const Vector2d & goal,
    const AgentConfig & config)
  : id_(id), position_(position), velocity_(velocity), goal_(goal), config_(config)
  {
  }

  void updateAgent(
    const uint id, const Vector2d & position, const Vector2d & velocity,
    const float naive_goal_time, const AgentConfig & config)
  {
    id_ = id;
    position_ = position;
    velocity_ = velocity;
    config_ = config;
    goal_ = position + velocity * naive_goal_time;
  }
  void updateAgent(
    const uint id, const Vector2d & position, const Vector2d & velocity, const Vector2d & goal,
    const AgentConfig & config)
  {
    id_ = id;
    position_ = position;
    velocity_ = velocity;
    config_ = config;
    goal_ = goal;
  }

  void updateGoal(const Vector2d & goal) {goal_ = goal;}

  void updateGoal(const float naive_goal_time) {goal_ = position_ + velocity_ * naive_goal_time;}
  Vector2d getGoal() const {return goal_;}

  Vector2d getPosition() const {return position_;}

  Vector2d getVelocity() const {return velocity_;}

  double getOrientation() const {return atan2(velocity_.y(), velocity_.x());}

  Vector2d getPrefVelocity() const
  {
    if (goalReached()) {
      return Vector2d(0.0, 0.0);
    }
    return normalize(goal_ - position_) * config_.max_speed;
  }
  AgentConfig getConfig() const {return config_;}

  uint getId() const {return id_;}

  void setId(uint id) {id_ = id;}

  void addAgentToSimulator(RVO::RVOSimulator & rvo_simulator) const
  {
    rvo_simulator.addAgent(
      position_, config_.neighbor_dist, config_.max_neighbors, config_.time_horizon,
      config_.time_horizon_obst, config_.radius, config_.max_speed, velocity_);
    rvo_simulator.setAgentPrefVelocity(id_, getPrefVelocity());
  }

private:
  bool goalReached() const
  {
    return absSq(goal_ - position_) < (config_.radius * config_.radius - 0.1);
  }

  uint id_;
  Vector2d position_;
  Vector2d velocity_;
  Vector2d goal_;
  AgentConfig config_;
};

/**
 * @brief convert a geometry_msgs::msg::Point to a Vector2d
 * @param point The point to convert
 * @return The Vector2d
 */
inline Vector2d convertToVector2d(const geometry_msgs::msg::Point & point)
{
  Vector2d pt(point.x, point.y);
  return pt;
}
/**
 * @brief convert a Vector2d to a geometry_msgs::msg::Point
 * @param point The Vector2d to convert
 * @return The geometry_msgs::msg::Point
 */
inline geometry_msgs::msg::Point convertToPoint(const Vector2d & point)
{
  geometry_msgs::msg::Point pt;
  pt.x = point.x();
  pt.y = point.y();
  return pt;
}

/**
 * @brief convert a Vector2d to a geometry_msgs::msg::Vector3
  * @param point The Vector2d to convert
  * @return The geometry_msgs::msg::Vector3
  */
inline geometry_msgs::msg::Vector3 convertToVector3(const Vector2d & point)
{
  geometry_msgs::msg::Vector3 pt;
  pt.x = point.x();
  pt.y = point.y();
  pt.z = 0.0;
  return pt;
}
/**
 * @brief convert a geometry_msgs::msg::Vector3 to a Vector2d
 * @param point The point to convert
 * @return The Vector2d
 */
inline Vector2d convertToVector2d(const geometry_msgs::msg::Vector3 & point)
{
  Vector2d pt(point.x, point.y);
  return pt;
}

/**
 * @brief convert a std::vector<Vector2d> to a std::vector<geometry_msgs::msg::Point>
 * @param points The vector of Vector2d to convert
 * @return The vector of geometry_msgs::msg::Point
 */
inline std::vector<geometry_msgs::msg::Point> convertToPoints(const std::vector<Vector2d> & points)
{
  std::vector<geometry_msgs::msg::Point> points_out;
  points_out.reserve(points.size());
  for (const auto & point : points) {
    geometry_msgs::msg::Point pt = convertToPoint(point);
    points_out.push_back(pt);
  }
  return points_out;
}
/**
 * @brief convert a std::vector<geometry_msgs::msg::Point> to a std::vector<Vector2d>
 * @param points The vector of geometry_msgs::msg::Point to convert
 * @return The vector of Vector2d
 */
inline std::vector<Vector2d> convertToVector2d(
  const std::vector<geometry_msgs::msg::Point> & points)
{
  std::vector<Vector2d> points_out;
  points_out.reserve(points.size());
  for (const auto & point : points) {
    Vector2d pt = convertToVector2d(point);
    points_out.push_back(pt);
  }
  return points_out;
}

/**
 * @brief Create an agent from a person
 * @param person The person to convert
 * @param t The timestamp to use for the transformation
 * @param in_frame_id The frame id of the person
 * @param out_frame_id The frame id of the agent
 * @param agent_config The agent config
 * @param naive_goal_time The naive goal time
 * @param tf_buffer The tf buffer
 */
inline bool createAgentFromPerson(
  const people_msgs::msg::Person & person, RVOAgent & agent, const std::string & in_frame_id,
  const std::string & out_frame_id, std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const builtin_interfaces::msg::Time & t, const AgentConfig & agent_config,
  float naive_goal_time = 0.5f)
{
  geometry_msgs::msg::PoseStamped pose_agent;
  pose_agent.header.frame_id = in_frame_id;
  pose_agent.header.stamp = t;
  pose_agent.pose.position = person.position;
  pose_agent.pose.position.z = 0.0;

  tf2::Quaternion quat;
  quat.setRPY(0, 0, person.position.z);
  pose_agent.pose.orientation = tf2::toMsg(quat);

  geometry_msgs::msg::Vector3 velocity;
  velocity.x = person.velocity.x;
  velocity.y = person.velocity.y;
  velocity.z = 0.0;

  if (in_frame_id != out_frame_id) {
    if (!common_utils::utils::transformPoseAndVelocity(
        pose_agent, velocity, out_frame_id, t, tf_buffer))
    {
      return false;  // Transform failed
    }
  }
  uint agent_id = 0;
  if (!person.tags.empty()) {
    try {
      agent_id = static_cast<uint>(std::stoul(person.tags[0]));
    } catch (const std::exception &) {
      RCLCPP_WARN(
        rclcpp::get_logger("RVOAgent"),
        "Person tag '%s' is not a valid unsigned integer. Defaulting to sequential ID.",
        person.tags[0].c_str());
    }
  }
  agent.updateAgent(
    agent_id, convertToVector2d(pose_agent.pose.position), convertToVector2d(velocity),
    naive_goal_time, agent_config);
  return true;
}

RVOAgent getAgent(uint id, RVO::RVOSimulator & rvo_simulator)
{
  if (id > rvo_simulator.getNumAgents()) {
    RCLCPP_ERROR(rclcpp::get_logger("RVOAgent"), "Agent %d does not exist", id);
    return RVOAgent(UINT_MAX, Vector2d(0.0, 0.0), Vector2d(0.0, 0.0), 0.0, AgentConfig());
  }
  return RVOAgent(
    id, rvo_simulator.getAgentPosition(id), rvo_simulator.getAgentVelocity(id),
    rvo_simulator.getAgentPrefVelocity(id), AgentConfig());
}

/**
   * @brief get velocity from the and the velocity of the agent
   * @param agent The RVO agent
   * @param current_pose The current pose of the agent
    * @return The velocity of the agent
    */
inline geometry_msgs::msg::TwistStamped getVelocity(
  const RVOAgent & agent, const geometry_msgs::msg::PoseStamped & current_pose)
{
  geometry_msgs::msg::TwistStamped velocity;
  velocity.header = current_pose.header;

  // get the velocity of the agent from the RVO simulator
  Vector2d velocity_rvo = agent.getVelocity();
  // get the target orientation
  double target_yaw = agent.getOrientation();
  double angle_error =
    angles::normalize_angle(target_yaw - tf2::getYaw(current_pose.pose.orientation));

  // speed
  double speed = hypotf(velocity_rvo.x(), velocity_rvo.y()) * cos(angle_error);
  // angular speed
  double omega = 2.0 * angle_error;

  velocity.twist.linear.x = speed;
  velocity.twist.linear.y = 0.0;
  velocity.twist.angular.z = omega;
  return velocity;
}

}  // namespace orca_controller::rvo_utils

#endif  // ORCA_CONTROLLER__RVO_UTILS_HPP_
