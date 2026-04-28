#include "sfm_controller/sfm_controller.hpp"

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

namespace sfm_controller
{

void SFMController::configure(
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
  RCLCPP_INFO(logger_, "Configuring controller: %s of type SFMController", name.c_str());

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
  getParam(naive_goal_time_, "naive_goal_time", 1.0);
  getParam(n_steps_, "n_steps", 10);  // number of steps to simulate the robot pose (visualization)

  getParam(parameters_.forceFactorDesired, "force_factor_desired", 2.0);
  getParam(parameters_.forceFactorObstacle, "force_factor_obstacle", 10.0);
  getParam(parameters_.forceSigmaObstacle, "force_sigma_obstacle", 0.2);
  getParam(parameters_.forceFactorSocial, "force_factor_social", 2.1);
  // getParam(parameters.lambda, "lambda", 2.0);
  // getParam(parameters.gamma, "gamma", 0.5);
  // getParam(parameters.n, "n", 10);
  // getParam(parameters.epsilon, "epsilon", 0.1);
  // getParam(parameters.nPrime, "n_prime", 5);

  // Configure composed objects

  agents_ = std::make_shared<std::vector<sfm::Agent>>();
  auto robot = sfm::Agent();

  initializeRobot(robot);
  robot.params = parameters_;
  agents_->push_back(robot);

  people_sub_ = node->create_subscription<people_msgs::msg::People>(
    "/people", 10, std::bind(&SFMController::peopleCallback, this, std::placeholders::_1));

  marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("obstacles", 10);
  forces_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("forces", 10);
  transformed_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("transformed_plan", 10);
  goal_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("local_goal", 10);
  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 10);

  path_handler_ =
    std::make_unique<common_utils::PathHandler>(
    tf2::durationFromSec(
      transform_tolerance_), tf_buffer_, costmap_ros_);

  max_vel_x_ = max_vel_x_initial_;
  max_ang_vel_ = max_ang_vel_initial_;

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap_ros_->getCostmap()->
    getMutex()));
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

  RCLCPP_INFO(logger_, "Configured controller: %s of type SFMController", plugin_name_.c_str());
}

void SFMController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up controller: %s of type SFMController", plugin_name_.c_str());

  marker_pub_.reset();
  forces_pub_.reset();
  transformed_plan_pub_.reset();
  goal_pub_.reset();
  local_plan_pub_.reset();
}

void SFMController::activate()
{
  RCLCPP_INFO(logger_, "Activating controller: %s of type SFMController", plugin_name_.c_str());
  marker_pub_->on_activate();
  forces_pub_->on_activate();
  transformed_plan_pub_->on_activate();
  goal_pub_->on_activate();
  local_plan_pub_->on_activate();
}

void SFMController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating controller: %s of type SFMController", plugin_name_.c_str());
  marker_pub_->on_deactivate();
  forces_pub_->on_deactivate();
  transformed_plan_pub_->on_deactivate();
  goal_pub_->on_deactivate();
  local_plan_pub_->on_deactivate();
}

geometry_msgs::msg::TwistStamped
SFMController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed, nav2_core::GoalChecker *)
{
  auto dt =
    rclcpp::Duration(rclcpp::Time(robot_pose.header.stamp) - rclcpp::Time(last_time_)).seconds();
  if (dt < 0.01) {
    RCLCPP_DEBUG(logger_, "Robot pose not updated, skipping control");
    return last_cmd_vel_;
  }
  nav_msgs::msg::Path transformed_plan = path_handler_->transformGlobalPlan(
    robot_pose,
    max_robot_pose_search_dist_);
  transformed_plan_pub_->publish(transformed_plan);

  // Get the particular point on the path at the motion target distance and publish it
  auto goal_point =
    path_handler_->getTransformedGoal(local_goal_dist_, transformed_plan, robot_pose);

  // Set the local goal
  sfm::Goal g;
  g.center.set(goal_point.point.x, goal_point.point.y);
  g.radius = 0.10;

  std::vector<geometry_msgs::msg::Point> obstacles;

  auto agents = getAgents();
  if (agents.size() == 0) {
    RCLCPP_ERROR(logger_, "No agents detected");
    return geometry_msgs::msg::TwistStamped();
  }
  auto & robot = agents[0];
  // auto agents_in_fov = std::vector<sfm::Agent>();
  std::vector<geometry_msgs::msg::Point> agent_positions =
    common_utils::hunav_utils::convertToPoint(agents);

  // Select a coarser resolution for the local costmap and retrieve the obstacles
  // select a resolution that is a multiple of the global costmap resolution
  common_utils::utils::fastResampleCostmap(costmap_ros_->getCostmap(), pool_costmap_);

  // Check if the agent is in the local costmap and update the obstacles
  common_utils::utils::getObstacles(pool_costmap_, agent_positions, person_radius_ * 2, obstacles);

  const std::vector<utils::Vector2d> obs_points = common_utils::hunav_utils::convertToVector2d(
    obstacles);
  RCLCPP_DEBUG(logger_, "Number of obstacles: %lu", obs_points.size());

  // update the robot agent
  robot.position.set(robot_pose.pose.position.x, robot_pose.pose.position.y);
  robot.yaw = utils::Angle::fromRadian(tf2::getYaw(robot_pose.pose.orientation));
  double robot_speed_norm = hypotf(robot_speed.linear.x, robot_speed.linear.y);
  robot.linearVelocity = robot_speed_norm;
  robot.angularVelocity = robot_speed.angular.z;
  robot.velocity.set(
    robot_speed_norm * cos(robot.yaw.toRadian()),
    robot_speed_norm * sin(robot.yaw.toRadian()));

  robot.desiredVelocity = max_vel_x_;
  robot.goals.clear();
  robot.goals.push_back(g);
  robot.obstacles1.clear();
  robot.obstacles1 = obs_points;
  robot.teleoperated = false;

  // fill the agents obstacles
  for (size_t i = 1; i < agents.size(); ++i) {
    agents[i].obstacles1.clear();
    agents[i].obstacles1 = obs_points;
    // agents[i].obstacles1 = common_utils::hunav_utils::convertToVector2d(obstacle_map[i]);
  }

  sfm::SFM.computeForces(agents);

  nav_msgs::msg::Path local_plan;
  local_plan = simulateRobotPose(agents, dt, n_steps_);
  local_plan.header = transformed_plan.header;
  local_plan.header.frame_id = costmap_ros_->getGlobalFrameID();
  sfm::SFM.updatePosition(robot, dt);
  RCLCPP_DEBUG(
    logger_, "Robot agent after %d: (%f, %f)", agents[0].id, agents[0].position.getX(),
    agents[0].position.getY());

  if (visualize_) {
    auto forces_pub = forces_pub_->get_subscription_count();
    if (forces_pub > 0) {
      forces_pub_->publish(
        common_utils::hunav_utils::createAgentsMarkerArray(
          agents,
          transformed_plan.header));
    }
    visualize(transformed_plan, local_plan, goal_point, agent_positions, obstacles);
  }
  // Compute the velocity commands
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = robot_pose.header.stamp;
  cmd_vel.header.frame_id = costmap_ros_->getGlobalFrameID();
  cmd_vel.twist.linear.x = robot.linearVelocity;
  cmd_vel.twist.angular.z = robot.angularVelocity;

  // publish the velocity commands
  last_time_ = robot_pose.header.stamp;
  last_cmd_vel_ = cmd_vel;
  return cmd_vel;
}

nav_msgs::msg::Path SFMController::simulateRobotPose(
  std::vector<sfm::Agent> agents, const double & dt,
  const int & n_steps)
{
  nav_msgs::msg::Path robot_poses;
  robot_poses.poses.resize(n_steps);
  // auto& robot = agents[0];
  for (int i = 0; i < n_steps; i++) {
    // sfm::SFM.updatePosition(agents, dt);
    updatePosition(agents, dt);
    // sfm::SFM.updatePosition(robot, dt);
    sfm::SFM.computeForces(agents);
    robot_poses.poses[i].pose.position.x = agents[0].position.getX();
    robot_poses.poses[i].pose.position.y = agents[0].position.getY();
    robot_poses.poses[i].pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
      agents[0].yaw.toRadian());
  }

  return robot_poses;
}

void SFMController::updatePosition(std::vector<sfm::Agent> & agents, const double & dt)
{
  for (auto & agent : agents) {
    agent.velocity += agent.forces.globalForce * dt;
    agent.linearVelocity = hypotf(agent.velocity.getX(), agent.velocity.getY());
    double new_yaw = atan2(agent.velocity.getY(), agent.velocity.getX());

    agent.angularVelocity = 0.5 * (agent.yaw.toRadian() - new_yaw) / dt;

    agent.yaw = utils::Angle::fromRadian(new_yaw);
    // Limit the linear velocity
    agent.linearVelocity = std::clamp(agent.linearVelocity, 0.0, max_vel_x_);
    agent.angularVelocity = std::clamp(agent.angularVelocity, -max_ang_vel_, max_ang_vel_);

    agent.velocity.set(
      agent.linearVelocity * cos(agent.yaw.toRadian()),
      agent.linearVelocity * sin(agent.yaw.toRadian()));

    // Update the position of the agent
    agent.position.set(
      agent.position.getX() + agent.velocity.getX() * dt,
      agent.position.getY() + agent.velocity.getY() * dt);
  }
}

void SFMController::visualize(
  nav_msgs::msg::Path transformed_plan, nav_msgs::msg::Path local_plan,
  geometry_msgs::msg::PointStamped goal,
  const std::vector<geometry_msgs::msg::Point> & agents,
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

void SFMController::setPlan(const nav_msgs::msg::Path & path)
{
  path_handler_->setPlan(path);
}

void SFMController::setSpeedLimit(const double & speed_limit, const bool & percentage)
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

void SFMController::peopleCallback(const people_msgs::msg::People::SharedPtr people_msg)
{
  // Update the agents with the people detected

  if (people_msg->people.empty()) {
    RCLCPP_DEBUG(logger_, "No people detected");
    return;
  }
  builtin_interfaces::msg::Time t = people_msg->header.stamp;
  std::vector<sfm::Agent> agents;

  geometry_msgs::msg::PoseStamped pose_agent;
  RCLCPP_DEBUG(logger_, "People frame: %s", people_msg->header.frame_id.c_str());

  if (people_msg->people.size() == 0) {
    RCLCPP_DEBUG(logger_, "No people detected");
    return;
  }

  for (const auto & person : people_msg->people) {
    // Create the agent
    sfm::Agent agent;
    if (!common_utils::hunav_utils::createAgentFromPerson(
        person, agent, people_msg->header.frame_id, costmap_ros_->getGlobalFrameID(), tf_buffer_, t,
        person_radius_,
        person_velocity_, naive_goal_time_, parameters_))
    {
      return;  // Transform failed
    }
    agents.push_back(agent);
  }

  updateAgents(agents);
  RCLCPP_DEBUG(logger_, "Number of agents: %lu", agents.size());
  for (unsigned int i = 0; i < agents.size(); i++) {
    RCLCPP_DEBUG(
      logger_, "Agent %d: (%f, %f)", agents[i].id,
      agents[i].position.getX(), agents[i].position.getY());
  }
}

void SFMController::initializeRobot(sfm::Agent & agent)
{
  agent.id = 0;
  agent.groupId = -1;
  agent.position.set(0.0, 0.0);
  agent.velocity.set(0.0, 0.0);
  agent.linearVelocity = 0.0;
  agent.yaw = utils::Angle::fromRadian(0.0);
  agent.angularVelocity = 0.0;
  agent.radius = robot_radius_;
  agent.teleoperated = false;
  agent.desiredVelocity = max_vel_x_;
}

void SFMController::updateAgents(const std::vector<sfm::Agent> & agents)
{
  if (agents_ == nullptr || agents_->empty()) {
    throw std::runtime_error{"Robot agent not initialized"};
  }

  std::lock_guard<std::mutex> lock(agents_mutex_);
  auto robot = (*agents_)[0];  // save robot
  agents_->resize(agents.size() + 1);
  (*agents_)[0] = robot;
  for (size_t i = 0; i < agents.size(); ++i) {
    (*agents_)[i + 1] = agents[i];
  }
}

std::vector<sfm::Agent> SFMController::getAgents()
{
  if (agents_ == nullptr) {
    throw std::runtime_error{"Agents are not set"};
  }
  std::lock_guard<std::mutex> lock(agents_mutex_);
  return *agents_;
}

}  // namespace sfm_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(sfm_controller::SFMController, nav2_core::Controller)
