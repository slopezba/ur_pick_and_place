// src/add_obstacle_once.cpp
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("add_box_obstacle");

  moveit::planning_interface::PlanningSceneInterface psi;

  moveit_msgs::msg::CollisionObject box;
  box.header.frame_id = "base_link";        // reference frame
  box.id = "table_box";

  shape_msgs::msg::SolidPrimitive prim;
  prim.type = prim.BOX;
  prim.dimensions = {0.50, 0.65, 0.80};     // x,y,z size (m)

  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;                 // no rotation
  pose.position.x = 0.0;                    // you didn’t specify x → assume 0
  pose.position.y = -0.2;                   // -0.2 m
  pose.position.z = -0.4;        // place center 0.4 m below base_link
                                            // object pose is at its center
  box.primitives.push_back(prim);
  box.primitive_poses.push_back(pose);
  box.operation = box.ADD;

  psi.applyCollisionObject(box);            // push to planning scene
  RCLCPP_INFO(node->get_logger(), "Added obstacle 'table_box'");
  // give the async apply a moment and exit
  rclcpp::sleep_for(std::chrono::milliseconds(300));
  rclcpp::shutdown();
  return 0;
}