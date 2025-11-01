// go_home_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <chrono>
#include <memory>
#include <algorithm>

class GoHomeNode : public rclcpp::Node
{
public:
  GoHomeNode() : rclcpp::Node("go_home_node")
  {
    // ── Parameters ──────────────────────────────────────────────────────────────
    group_name_     = this->declare_parameter<std::string>("group_name", "manipulator");
    base_frame_     = this->declare_parameter<std::string>("base_frame", "base_link");
    eef_link_       = this->declare_parameter<std::string>("eef_link", "tool0");  // "" = use SRDF default
    home_named_     = this->declare_parameter<std::string>("home_named", "home");
    use_named_home_ = this->declare_parameter<bool>("use_named_home", true);
    v_scale_        = this->declare_parameter<double>("velocity_scaling", 0.2);
    a_scale_        = this->declare_parameter<double>("acceleration_scaling", 0.2);

    // Cartesian fallback for Home (if named state missing/fails)
    home_pose_.position.x = this->declare_parameter<double>("home_pose.x",  0.5);
    home_pose_.position.y = this->declare_parameter<double>("home_pose.y",  0.4);
    home_pose_.position.z = this->declare_parameter<double>("home_pose.z",  0.65);
    home_pose_.orientation.x = 0.0;
    home_pose_.orientation.y = 0.0;
    home_pose_.orientation.z = 0.0;
    home_pose_.orientation.w = 1.0;

    // ── MoveIt interface ────────────────────────────────────────────────────────
    auto node_ptr = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_ptr, group_name_);
    move_group_->setPoseReferenceFrame(base_frame_);
    if (!eef_link_.empty()) move_group_->setEndEffectorLink(eef_link_);
    move_group_->setMaxVelocityScalingFactor(std::clamp(v_scale_, 0.0, 1.0));
    move_group_->setMaxAccelerationScalingFactor(std::clamp(a_scale_, 0.0, 1.0));

    // Start monitoring joint states so getCurrentState() is populated
    move_group_->startStateMonitor();
    // Give it a moment to receive the first joint_state
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // move_group_->waitForCurrentState(rclcpp::Duration::from_seconds(5.0));

    // ── Trigger subscriber ──────────────────────────────────────────────────────
    // Publish std_msgs/Empty on /go_home to trigger a move to Home
    sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "go_home", 10,
      [this](const std_msgs::msg::Empty&) {
        RCLCPP_INFO(this->get_logger(), "Trigger received: going to Home…");
        if (goHome()) {
          RCLCPP_INFO(this->get_logger(), "Reached Home.");
        } else {
          RCLCPP_WARN(this->get_logger(), "Failed to reach Home.");
        }
      });

    RCLCPP_INFO(this->get_logger(),
      "GoHomeNode ready. Publish std_msgs/Empty on /go_home to move to Home.");
  }

private:
  bool goHome()
  {
    // Make sure we start from the current state
    move_group_->setStartStateToCurrentState();
    // Try SRDF named state first
    if (use_named_home_) {
      const bool set_ok = move_group_->setNamedTarget(home_named_);
      if (set_ok) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        const bool planned = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (planned) {
          const auto res = move_group_->execute(plan);
          if (res == moveit::core::MoveItErrorCode::SUCCESS) return true;
        }
        RCLCPP_WARN(this->get_logger(),
          "Named Home '%s' failed; falling back to Cartesian Home pose.", home_named_.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(),
          "Named Home '%s' not defined; falling back to Cartesian Home pose.", home_named_.c_str());
      }
      move_group_->clearPoseTargets();
    }

    // Fallback: Cartesian Home in base_frame
    geometry_msgs::msg::PoseStamped target;
    target.header.stamp = now();
    target.header.frame_id = base_frame_;
    target.pose = home_pose_;

    move_group_->setPoseTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!planned) {
      move_group_->clearPoseTargets();
      return false;
    }
    const auto res = move_group_->execute(plan);
    move_group_->clearPoseTargets();
    return (res == moveit::core::MoveItErrorCode::SUCCESS);
  }

  // Members
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_;

  std::string group_name_, base_frame_, eef_link_, home_named_;
  bool use_named_home_{true};
  double v_scale_{0.2}, a_scale_{0.2};

  geometry_msgs::msg::Pose home_pose_;
};
  
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoHomeNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}