// pick_and_place.cpp (no Cartesian Home fallback)
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <algorithm>

class PickPlaceNode : public rclcpp::Node
{
public:
  PickPlaceNode() : rclcpp::Node("pick_and_place")
  {
    // ---- Parameters (with sensible defaults) ----
    group_name_      = this->declare_parameter<std::string>("group_name", "ur_manipulator");
    base_frame_      = this->declare_parameter<std::string>("base_frame", "base_link");
    eef_link_        = this->declare_parameter<std::string>("eef_link", "tool0");   // "" → SRDF default
    v_scale_         = this->declare_parameter<double>("velocity_scaling", 0.8);
    a_scale_         = this->declare_parameter<double>("acceleration_scaling", 0.8);
    dwell_sec_       = this->declare_parameter<int>("dwell_seconds", 1);
    home_named_      = this->declare_parameter<std::string>("home_named", "home");
    use_named_home_  = this->declare_parameter<bool>("use_named_home", true);

    // ---- MoveGroupInterface (safe construction) ----
    auto node_ptr = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_ptr, group_name_);
    move_group_->setPlanningTime(5.0);
    move_group_->setNumPlanningAttempts(5);

    // ---- Add joint-home parameter and a default value ----
    home_joints_ = this->declare_parameter<std::vector<double>>(
      "home_joints",
      std::vector<double>{0.0, -1.5708, 1.5708, 0.0, 1.5708, 0.0}  // default
    );
    if (home_joints_.size() != 6) {
      RCLCPP_WARN(this->get_logger(),
        "Parameter 'home_joints' must have 6 elements (got %zu). Using default.", home_joints_.size());
      home_joints_ = {0.0, -1.5708, 1.5708, 0.0, 1.5708, 0.0};
    }

    // Configure MoveIt interface
    move_group_->setPoseReferenceFrame(base_frame_);
    if (!eef_link_.empty()) move_group_->setEndEffectorLink(eef_link_);
    move_group_->setMaxVelocityScalingFactor(std::clamp(v_scale_, 0.0, 1.0));
    move_group_->setMaxAccelerationScalingFactor(std::clamp(a_scale_, 0.0, 1.0));
    move_group_->startStateMonitor();
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // ---- I/O ----
    sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/goal_poses", 10,
      std::bind(&PickPlaceNode::goalPosesCallback, this, std::placeholders::_1));

    // Start worker thread (detached loop that waits for goals)
    worker_thread_ = std::thread([this](){ this->workerLoop(); });

    RCLCPP_INFO(this->get_logger(),
      "PickPlaceNode ready. Publish geometry_msgs/PoseArray on /goal_poses. Home: named='%s' (joint fallback enabled).",
      home_named_.c_str());

  }

  ~PickPlaceNode() override
  {
    // Signal worker thread to exit and join
    {
      std::lock_guard<std::mutex> lk(mtx_);
      shutting_down_ = true;
      cv_.notify_all();
    }
    if (worker_thread_.joinable())
      worker_thread_.join();
  }

private:

  // Fallback home in joint space (UR5e order: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3)
  std::vector<double> home_joints_{0.0, -1.5708, 1.5708, 0.0, 1.5708, 0.0};
  
  // --------- Callbacks / Worker ---------
  void goalPosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    goal_poses_ = msg->poses;
    RCLCPP_INFO(this->get_logger(), "Received %zu goal poses.", goal_poses_.size());
    cv_.notify_all();
  }

  void workerLoop()
  {
    std::unique_lock<std::mutex> lk(mtx_);
    while (!shutting_down_) {
      cv_.wait(lk, [this](){ return shutting_down_ || (!goal_poses_.empty() && !executing_); });
      if (shutting_down_) break;

      // Copy goals and mark executing
      std::vector<geometry_msgs::msg::Pose> goals = goal_poses_;
      goal_poses_.clear();
      executing_ = true;
      lk.unlock();

      bool ok = executeSequence(goals);

      lk.lock();
      executing_ = false;
      if (!ok)
        RCLCPP_WARN(this->get_logger(), "Sequence ended with a failure.");
      else
        RCLCPP_INFO(this->get_logger(), "Sequence completed.");
    }
  }

  bool executeSequence(const std::vector<geometry_msgs::msg::Pose>& goals)
  {
    for (size_t i = 0; i < goals.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), " [%zu/%zu] Moving to goal pose...", i+1, goals.size());
      if (!goToPose(goals[i])) {
        RCLCPP_WARN(this->get_logger(), "Planning to goal pose failed.");
        return false;
      }
      dwell(dwell_sec_);

      RCLCPP_INFO(this->get_logger(), " Returning to Home (named '%s')...", home_named_.c_str());
      if (!goHome()) {
        RCLCPP_ERROR(this->get_logger(),
          "Failed to return to named Home '%s' (no Cartesian fallback).", home_named_.c_str());
        return false;
      }
      dwell(dwell_sec_);
    }
    return true;
  }

  // --------- Motion Helpers ---------
  bool goToPose(const geometry_msgs::msg::Pose &pose)
  {
    move_group_->setStartStateToCurrentState();

    geometry_msgs::msg::PoseStamped target;
    target.header.stamp = now();
    target.header.frame_id = base_frame_;
    target.pose = pose;

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

  bool goHome()
  {
    move_group_->setStartStateToCurrentState();

    // Try named home if allowed
    if (use_named_home_) {
      if (move_group_->setNamedTarget(home_named_)) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
          if (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            return true;
          } else {
            RCLCPP_ERROR(this->get_logger(),
              "Execution to named Home '%s' failed. Falling back to joint HOME.", home_named_.c_str());
          }
        } else {
          RCLCPP_ERROR(this->get_logger(),
            "Planning to named Home '%s' failed. Falling back to joint HOME.", home_named_.c_str());
        }
      } else {
        RCLCPP_ERROR(this->get_logger(),
          "Named Home '%s' not defined in SRDF for group '%s'. Falling back to joint HOME.",
          home_named_.c_str(), group_name_.c_str());
      }
    } else {
      RCLCPP_WARN(this->get_logger(),
        "use_named_home is false. Using joint HOME fallback.");
    }

    // Fallback: joint-space home
    return goHomeJoints();
  }

  bool goHomeJoints()
  {
    move_group_->setStartStateToCurrentState();
    move_group_->setJointValueTarget(home_joints_);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Planning to joint fallback HOME failed.");
      return false;
    }
    const auto res = move_group_->execute(plan);
    if (res != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Execution to joint fallback HOME failed.");
      return false;
    }
    return true;
  }

  static void dwell(int seconds)
  {
    if (seconds <= 0) return;
    std::this_thread::sleep_for(std::chrono::seconds(seconds));
  }

  // --------- Members ---------
  // ROS I/O
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;

  // MoveIt
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  // Params / config
  std::string group_name_, base_frame_, eef_link_, home_named_;
  double v_scale_{0.2}, a_scale_{0.2};
  int dwell_sec_{3};
  bool use_named_home_{true};

  // State & data
  std::vector<geometry_msgs::msg::Pose> goal_poses_;
  std::mutex mtx_;
  std::condition_variable cv_;
  std::thread worker_thread_;
  std::atomic<bool> executing_{false};
  bool shutting_down_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickPlaceNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}