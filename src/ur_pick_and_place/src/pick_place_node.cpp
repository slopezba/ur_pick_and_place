#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <chrono>
#include <thread>

class PickPlaceNode : public rclcpp::Node
{
public:
  PickPlaceNode()
  : Node("pick_place_node"),
    move_group_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}), "ur_manipulator")
  {
    // Pose home predeterminada
    home_pose_.position.x = -0.5;
    home_pose_.position.y = 0.4;
    home_pose_.position.z = 0.3;
    home_pose_.orientation.w = 1.0;

    // Publicador para la pose home
    home_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/home_pose", 10);

    // Suscriptor a /goal_poses
    sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/goal_poses", 10,
      std::bind(&PickPlaceNode::goalPosesCallback, this, std::placeholders::_1));

    // Timer para ejecutar la secuencia
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&PickPlaceNode::executeSequence, this));

    // Timer para publicar home_pose cada 0.5s
    home_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&PickPlaceNode::publishHomePose, this));

    RCLCPP_INFO(this->get_logger(), "PickPlaceNode listo. Esperando poses en /goal_poses...");
  }

private:
  void goalPosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    goal_poses_ = msg->poses;
    RCLCPP_INFO(this->get_logger(), "Recibidas %zu poses", goal_poses_.size());
  }

  void goToPose(const geometry_msgs::msg::Pose &pose, int wait_seconds = 0)
  {
    move_group_.setPoseTarget(pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_.plan(plan));

    if (success)
    {
      move_group_.execute(plan);
      RCLCPP_INFO(this->get_logger(), "Movimiento ejecutado con éxito");

      if (wait_seconds > 0)
      {
        RCLCPP_INFO(this->get_logger(), "Esperando %d segundos...", wait_seconds);
        std::this_thread::sleep_for(std::chrono::seconds(wait_seconds));
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "No se pudo planificar hacia la pose objetivo");
    }

    move_group_.clearPoseTargets();
  }

  void executeSequence()
  {
    if (goal_poses_.empty())
      return;

    for (const auto &pose : goal_poses_)
    {
      RCLCPP_INFO(this->get_logger(), "Moviéndose a pose objetivo...");
      goToPose(pose, 3);  // Espera 3s después de llegar

      RCLCPP_INFO(this->get_logger(), "Regresando a pose home...");
      goToPose(home_pose_, 3);  // Espera 3s después de volver a home
    }
  }

  void publishHomePose()
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link"; 
    msg.pose = home_pose_;

    home_pub_->publish(msg);
  }

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr home_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr home_timer_;

  // MoveIt
  moveit::planning_interface::MoveGroupInterface move_group_;

  // Datos
  std::vector<geometry_msgs::msg::Pose> goal_poses_;
  geometry_msgs::msg::Pose home_pose_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<PickPlaceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
