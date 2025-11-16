#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/Geometry>

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  auto node = rclcpp::Node::make_shared("fk_example", opts);

  // UR5e defaults in the MoveIt UR drivers
  const std::string group_name = "manipulator";
  const std::string base_frame = "base_link";
  const std::string eef_link   = "tool0";   // use "tool0" (TCP = tool0 if no offset)

  moveit::core::RobotModelLoader loader(node, "robot_description"); // ensure param is loaded
  auto model = loader.getModel();
  moveit::core::RobotState state(model);
  state.setToDefaultValues();

  const moveit::core::JointModelGroup* jmg = model->getJointModelGroup(group_name);

  // Joint order for UR5e "manipulator":
  // shoulder_pan_joint, shoulder_lift_joint, elbow_joint,
  // wrist_1_joint, wrist_2_joint, wrist_3_joint
  std::vector<double> q = {0.0, -1.5708, 1.5708, 0.0, 1.5708, 0.0};

  state.setJointGroupPositions(jmg, q);
  state.update(); // computes forward kinematics

  Eigen::Isometry3d T = state.getGlobalLinkTransform(eef_link); // w.r.t. base_link
  Eigen::Vector3d p = T.translation();
  Eigen::Quaterniond qxyzw(T.rotation()); // x,y,z,w

  std::cout << "Position [m]: " << p.transpose() << std::endl;
  std::cout << "Orientation quaternion [x y z w]: "
            << qxyzw.x() << " " << qxyzw.y() << " "
            << qxyzw.z() << " " << qxyzw.w() << std::endl;

  rclcpp::shutdown();
  return 0;
}