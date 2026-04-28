#include "orca_controller/orca_controller.hpp"

#include <angles/angles.h>
#include <math.h>
#include <stdint.h>

#include <chrono>
#include <cmath>

#include "dwb_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_2d_utils/path_ops.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "pluginlib/class_list_macros.hpp"

// #define BENCHMARK_TESTING
using nav2_util::declare_parameter_if_not_declared;

namespace orca_controller
{

void ORCAController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }
  costmap_ros_ = costmap_ros;
  tf_buffer_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  RCLCPP_INFO(logger_, "Configuring controller: %s of type ORCAController", name.c_str());

  // Get high-level controller parameters
  parameters_handler_ = std::make_unique<common_utils::ParametersHandler>(parent);
  auto getParam = parameters_handler_->getParamGetter(name);
  getParam(robot_radius_, "robot_radius", 0.5);
  getParam(person_radius_, "person_radius", 0.5);
  getParam(person_velocity_, "person_velocity", 0.5);
  getParam(naive_goal_time_, "naive_goal_time", 0.5);
  getParam(fov_angle_, "fov_angle", M_PI / 2);
  getParam(new_resolution_, "new_resolution", 0.2);
  getParam(max_vel_x_initial_, "max_vel_x", 0.5);
  getParam(max_ang_vel_initial_, "max_ang_vel", 1.4);
  getParam(transform_tolerance_, "transform_tolerance", 0.5);
  getParam(visualize_, "visualize", false);
  getParam(max_robot_pose_search_dist_, "max_robot_pose_search_dist", 3.0);
  getParam(local_goal_dist_, "local_goal_dist", 2.0);
  getParam(n_steps_, "n_steps", 10);  // number of steps to simulate the robot pose (visualization)

  // Get ORCA parameters
  getParam(neighbor_dist_, "neighbor_dist", 1.0);
  getParam(max_neighbors_, "max_neighbors", 10);
  getParam(time_horizon_, "time_horizon", 1.0);
  getParam(time_horizon_obst_, "time_horizon_obst", 1.0);

  // Configure composed objects
  person_config_ = rvo_utils::AgentConfig(
    neighbor_dist_, max_neighbors_, time_horizon_, time_horizon_obst_, person_radius_,
    person_velocity_);
  robot_config_ = rvo_utils::AgentConfig(
    neighbor_dist_, max_neighbors_, time_horizon_, time_horizon_obst_, robot_radius_,
    max_vel_x_initial_);
  agents_ = std::make_shared<std::vector<rvo_utils::RVOAgent>>();
  auto robot = rvo_utils::RVOAgent(0, Vector2d(0.0, 0.0), Vector2d(0.0, 0.0), 0.0, robot_config_);
  agents_->push_back(robot);

  people_sub_ = node->create_subscription<people_msgs::msg::People>(
    "/people", 10, std::bind(&ORCAController::peopleCallback, this, std::placeholders::_1));

  marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("obstacles", 10);
  transformed_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("transformed_plan", 10);
  goal_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("local_goal", 10);
  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 10);

  path_handler_ = std::make_unique<common_utils::PathHandler>(
    tf2::durationFromSec(transform_tolerance_), tf_buffer_, costmap_ros_);

  max_vel_x_ = max_vel_x_initial_;
  max_ang_vel_ = max_ang_vel_initial_;

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(
    *(costmap_ros_->getCostmap()->getMutex()));
  common_utils::utils::createPooledCostmap(
    costmap_ros_->getCostmap(), pool_costmap_, new_resolution_);
  costmap_lock.unlock();

  RCLCPP_DEBUG(logger_, "Pooled costmap created");
  if (pool_costmap_ == nullptr) {
    throw std::runtime_error{"Failed to create pooled costmap"};
  }
  RCLCPP_DEBUG(
    logger_, "Pooled costmap initialized: (%d, %d)", pool_costmap_->getSizeInCellsX(),
    pool_costmap_->getSizeInCellsY());

  RCLCPP_INFO(logger_, "Configured controller: %s of type ORCAController", plugin_name_.c_str());
}

void ORCAController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up controller: %s of type ORCAController", plugin_name_.c_str());

  marker_pub_.reset();
  transformed_plan_pub_.reset();
  goal_pub_.reset();
  local_plan_pub_.reset();
}

void ORCAController::activate()
{
  RCLCPP_INFO(logger_, "Activating controller: %s of type ORCAController", plugin_name_.c_str());
  marker_pub_->on_activate();
  transformed_plan_pub_->on_activate();
  goal_pub_->on_activate();
  local_plan_pub_->on_activate();
}

void ORCAController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating controller: %s of type ORCAController", plugin_name_.c_str());
  marker_pub_->on_deactivate();
  transformed_plan_pub_->on_deactivate();
  goal_pub_->on_deactivate();
  local_plan_pub_->on_deactivate();
}

geometry_msgs::msg::TwistStamped ORCAController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose, const geometry_msgs::msg::Twist & robot_speed,
  nav2_core::GoalChecker *)
{
  auto dt =
    rclcpp::Duration(rclcpp::Time(robot_pose.header.stamp) - rclcpp::Time(last_time_)).seconds();
  if (dt < 0.01) {
    RCLCPP_DEBUG(logger_, "Robot pose not updated, skipping control");
    return last_cmd_vel_;
  }
  nav_msgs::msg::Path transformed_plan =
    path_handler_->transformGlobalPlan(robot_pose, max_robot_pose_search_dist_);
  transformed_plan_pub_->publish(transformed_plan);

  // Get the particular point on the path at the motion target distance and publish it
  auto goal_point =
    path_handler_->getTransformedGoal(local_goal_dist_, transformed_plan, robot_pose);

  // Set the local goal
  RVO::Vector2 g;

  std::vector<RVO::Vector2> obs_points;
  std::vector<geometry_msgs::msg::Point> obstacles;

  auto agents = getAgents();
  if (agents.size() == 0) {
    RCLCPP_ERROR(logger_, "No agents detected");
    return geometry_msgs::msg::TwistStamped();
  }
  auto & robot = agents[0];
  // auto agents_in_fov = std::vector<RVO::Vector2>();
  std::vector<geometry_msgs::msg::Point> agent_positions;

  // Select a coarser resolution for the local costmap and retrieve the obstacles
  // select a resolution that is a multiple of the global costmap resolution
  common_utils::utils::fastResampleCostmap(costmap_ros_->getCostmap(), pool_costmap_);

  // Check if the agent is in the local costmap and update the obstacles
  common_utils::utils::getObstacles(pool_costmap_, agent_positions, person_radius_ * 2, obstacles);

  RCLCPP_DEBUG(logger_, "Number of obstacles: %lu", obs_points.size());

  robot.updateAgent(
    0, rvo_utils::convertToVector2d(robot_pose.pose.position),
    Vector2d(
      robot_speed.linear.x * cos(tf2::getYaw(robot_pose.pose.orientation)) -
      robot_speed.linear.y * sin(tf2::getYaw(robot_pose.pose.orientation)),
      robot_speed.linear.x * sin(tf2::getYaw(robot_pose.pose.orientation)) +
      robot_speed.linear.y * cos(tf2::getYaw(robot_pose.pose.orientation))),
    rvo_utils::convertToVector2d(goal_point.point), robot_config_);
  // Compute the velocity commands
  // create the new simulator
  RVO::RVOSimulator rvo_simulator;
  rvo_simulator.setTimeStep(dt);
  for (const auto & agent : agents) {
    agent.addAgentToSimulator(rvo_simulator);
  }
  rvo_simulator.doStep();
  nav_msgs::msg::Path local_plan;
  local_plan = simulateRobotPose(agents, dt, n_steps_);
  local_plan.header = transformed_plan.header;
  local_plan.header.frame_id = costmap_ros_->getGlobalFrameID();

  RCLCPP_DEBUG(
    logger_, "Robot agent after %d: (%f, %f)", agents[0].getId(), agents[0].getPosition().x(),
    agents[0].getPosition().y());

  if (visualize_) {
    visualize(transformed_plan, local_plan, goal_point, agent_positions, obstacles);
  }
  // Compute the velocity commands
  geometry_msgs::msg::Twist robot_velocity;
  rvo_utils::RVOAgent robot_updated = rvo_utils::getAgent(robot.getId(), rvo_simulator);

  // compute the linear and angular velocity
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel = rvo_utils::getVelocity(robot_updated, robot_pose);
  // publish the velocity commands
  last_time_ = robot_pose.header.stamp;
  last_cmd_vel_ = cmd_vel;
  return cmd_vel;
}

nav_msgs::msg::Path ORCAController::simulateRobotPose(
  std::vector<rvo_utils::RVOAgent> agents, const double & dt, const int & n_steps)
{
  nav_msgs::msg::Path robot_poses;
  RVO::RVOSimulator rvo_simulator;
  rvo_simulator.setTimeStep(dt);
  for (const auto & agent : agents) {
    agent.addAgentToSimulator(rvo_simulator);
  }
  robot_poses.poses.resize(n_steps);
  for (int i = 0; i < n_steps; i++) {
    rvo_simulator.doStep();
    robot_poses.poses[i].pose.position.x = agents[0].getPosition().x();
    robot_poses.poses[i].pose.position.y = agents[0].getPosition().y();
    robot_poses.poses[i].pose.orientation =
      nav2_util::geometry_utils::orientationAroundZAxis(agents[0].getOrientation());
    for (auto & agent : agents) {
      agent.updateAgent(
        agent.getId(), rvo_simulator.getAgentPosition(agent.getId()),
        rvo_simulator.getAgentVelocity(agent.getId()), agent.getGoal(), agent.getConfig());
      rvo_simulator.setAgentPrefVelocity(agent.getId(), agent.getPrefVelocity());
    }
  }
  return robot_poses;
}

void ORCAController::visualize(
  nav_msgs::msg::Path transformed_plan, nav_msgs::msg::Path local_plan,
  geometry_msgs::msg::PointStamped goal, const std::vector<geometry_msgs::msg::Point> & agents,
  const std::vector<geometry_msgs::msg::Point> & obstacles)
{
  // Publish the goal
  auto goal_pub = goal_pub_->get_subscription_count();
  if (goal_pub > 0) {
    goal_pub_->publish(goal);
  }

  // Publish the transformed plan
  auto transformed_plan_pub = transformed_plan_pub_->get_subscription_count();
  if (transformed_plan_pub > 0) {
    transformed_plan_pub_->publish(transformed_plan);
  }

  // Publish the obstacles
  auto marker_pub = marker_pub_->get_subscription_count();
  if (marker_pub > 0) {
    marker_pub_->publish(common_utils::utils::visualizeMarkers(agents, obstacles));
  }
  // Publish the local plan
  auto local_plan_pub = local_plan_pub_->get_subscription_count();
  if (local_plan_pub > 0) {
    local_plan_pub_->publish(local_plan);
  }
}

void ORCAController::setPlan(const nav_msgs::msg::Path & path) {path_handler_->setPlan(path);}

void ORCAController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    max_vel_x_ = max_vel_x_initial_;
    max_ang_vel_ = max_ang_vel_initial_;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot

      max_vel_x_ = max_vel_x_initial_ * speed_limit / 100.0;
      max_ang_vel_ = max_ang_vel_initial_ * speed_limit / 100.0;

    } else {
      // Speed limit is expressed in m/s
      max_vel_x_ = std::max(speed_limit, 0.0);
      max_ang_vel_ = std::max(speed_limit, 0.0);
      // Limit the angular velocity to be proportional to the linear velocity
      max_ang_vel_ = max_ang_vel_initial_ * speed_limit / max_vel_x_initial_;
    }
  }
}

void ORCAController::peopleCallback(const people_msgs::msg::People::SharedPtr people_msg)
{
  // Update the agents with the people detected

  if (people_msg->people.empty()) {
    RCLCPP_DEBUG(logger_, "No people detected");
    return;
  }
  builtin_interfaces::msg::Time t = people_msg->header.stamp;
  std::vector<rvo_utils::RVOAgent> agents;
  geometry_msgs::msg::PoseStamped pose_agent;
  RCLCPP_DEBUG(logger_, "People frame: %s", people_msg->header.frame_id.c_str());

  if (people_msg->people.size() == 0) {
    RCLCPP_DEBUG(logger_, "No people detected");
    return;
  }

  for (const auto & person : people_msg->people) {
    // Create the agent
    rvo_utils::RVOAgent agent;
    if (!rvo_utils::createAgentFromPerson(
        person, agent, people_msg->header.frame_id, costmap_ros_->getGlobalFrameID(), tf_buffer_,
        t, person_config_, naive_goal_time_))
    {
      return;  // Transform failed
    }
    agents.push_back(agent);
  }

  updateAgents(agents);
  RCLCPP_DEBUG(logger_, "Number of agents: %lu", agents.size());
  for (unsigned int i = 0; i < agents.size(); i++) {
    RCLCPP_DEBUG(
      logger_, "Agent %d: (%f, %f)", agents[i].getId(), agents[i].getPosition().x(),
      agents[i].getPosition().y());
  }
}

void ORCAController::updateAgents(const std::vector<rvo_utils::RVOAgent> & agents)
{
  if (agents_ == nullptr || agents_->empty()) {
    throw std::runtime_error{"Robot agent not initialized"};
  }

  std::lock_guard<std::mutex> lock(agents_mutex_);
  auto robot = (*agents_)[0];  // save robot
  robot.setId(0);
  agents_->resize(agents.size() + 1);
  (*agents_)[0] = robot;
  for (size_t i = 0; i < agents.size(); ++i) {
    auto agent = agents[i];
    agent.setId(static_cast<uint>(i + 1));
    (*agents_)[i + 1] = agent;
  }
}

std::vector<rvo_utils::RVOAgent> ORCAController::getAgents()
{
  if (agents_ == nullptr) {
    throw std::runtime_error{"Agents are not set"};
  }
  std::lock_guard<std::mutex> lock(agents_mutex_);
  return *agents_;
}

}  // namespace orca_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(orca_controller::ORCAController, nav2_core::Controller)
