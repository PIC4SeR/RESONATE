#ifndef SFM_CONTROLLER__SFM_CONTROLLER_HPP_
#define SFM_CONTROLLER__SFM_CONTROLLER_HPP_
#include <lightsfm/angle.hpp>
#include <lightsfm/sfm.hpp>
#include <lightsfm/vector2d.hpp>
#include <memory>
#include <mutex>
#include <string>

#include "common_utils/hunav_utils.hpp"
#include "common_utils/parameter_handler.hpp"
#include "common_utils/path_handler.hpp"
#include "common_utils/utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "people_msgs/msg/people.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace sfm_controller
{

/**
 * @class sfm_controller::SFMController
 * @brief Main plugin controller for SFM Controller
 */
class SFMController : public nav2_core::Controller
{
public:
  /**
   * @brief Constructor for SFMController
   */
  SFMController() = default;

  /**
   * @brief Configure controller on bringup
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer to use
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup resources
   */
  void cleanup() override;

  /**
   * @brief Activate controller
   */
  void activate() override;

  /**
   * @brief Deactivate controller
   */
  void deactivate() override;

  /**
   * @brief Reset the controller state between tasks
   */
  void reset();

  /**
   * @brief Main method to compute velocities using the optimizer
   * @param robot_pose Robot pose
   * @param robot_speed Robot speed
   * @param goal_checker Pointer to the goal checker for awareness if completed task
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    nav2_core::GoalChecker * goal_checker) override;

  /**
   * @brief Set new reference path to track
   * @param path Path to track
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Set new speed limit from callback
   * @param speed_limit Speed limit to use
   * @param percentage Bool if the speed limit is absolute or relative
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  /**
   * @brief People callback:
   * @param people_msg The people message
   */
  void peopleCallback(const std::shared_ptr<people_msgs::msg::People> msg);

  /**
   * @brief  get the agents
   */
  std::vector<sfm::Agent> getAgents();

  /**
   * @brief  initialize the robot agent
   * @param agent The robot agent
   */
  void initializeRobot(sfm::Agent & agent);

  /**
   * @brief  update the agents
   * @param agents The vector of social agents
   */
  void updateAgents(const std::vector<sfm::Agent> & agents);

  /**
   * @brief Simulate the robot pose
   * @param agents The vector of social agents
   * @param dt The time step
   * @param n_steps The number of steps
   * @return The simulated path
   */
  nav_msgs::msg::Path simulateRobotPose(
    const std::vector<sfm::Agent> agents, const double & dt,
    const int & n_steps);

protected:
  /**
   * @brief Visualize trajectories
   * @param transformed_plan Transformed input plan
   */
  void visualize(
    nav_msgs::msg::Path transformed_plan, nav_msgs::msg::Path local_plan,
    geometry_msgs::msg::PointStamped goal, const std::vector<geometry_msgs::msg::Point> & agents,
    const std::vector<geometry_msgs::msg::Point> & obstacles);

  /**
   * @brief Update the robot pose
   * @param agents The vector of social agents
   * @param dt The time step
   */
  void updatePosition(std::vector<sfm::Agent> & agents, const double & dt);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_costmap_2d::Costmap2D> pool_costmap_;
  rclcpp::Logger logger_{rclcpp::get_logger("SFMController")};

  rclcpp::Subscription<people_msgs::msg::People>::SharedPtr people_sub_;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr transformed_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr forces_pub_;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;

  std::unique_ptr<common_utils::ParametersHandler> parameters_handler_;
  std::unique_ptr<common_utils::PathHandler> path_handler_;

  std::shared_ptr<std::vector<sfm::Agent>> agents_;

  geometry_msgs::msg::TwistStamped last_cmd_vel_;
  std::mutex agents_mutex_;
  double robot_radius_, person_radius_, person_velocity_, naive_goal_time_, fov_angle_;
  double transform_tolerance_, max_robot_pose_search_dist_, local_goal_dist_;
  bool visualize_;
  sfm::Parameters parameters_;
  int n_steps_;  // number of steps to simulate the robot pose (visualization)
  double new_resolution_, max_vel_x_, max_ang_vel_;
  builtin_interfaces::msg::Time last_time_;
  double max_vel_x_initial_, max_ang_vel_initial_;
};

}  // namespace sfm_controller

#endif  // SFM_CONTROLLER__SFM_CONTROLLER_HPP_
