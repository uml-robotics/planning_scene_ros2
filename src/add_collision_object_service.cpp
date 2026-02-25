#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "planning_scene_ros2/srv/add_collision_object.hpp"

class AddCollisionObjectNode : public rclcpp::Node
{
public:
  AddCollisionObjectNode()
  : Node("add_collision_object_node"),
    object_counter_(0)
  {
    // Frame to interpret req.x/y/z in (planning frame). Override via parameter if needed.
    frame_id_ = this->declare_parameter<std::string>("frame_id", "world");

    service_ = this->create_service<planning_scene_ros2::srv::AddCollisionObject>(
      "add_collision_object",
      std::bind(
        &AddCollisionObjectNode::handle_request,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Add Collision Object service ready. frame_id='%s'", frame_id_.c_str());
  }

private:
  rclcpp::Service<planning_scene_ros2::srv::AddCollisionObject>::SharedPtr service_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  std::string frame_id_;
  std::atomic<uint64_t> object_counter_;

  void handle_request(
    const std::shared_ptr<planning_scene_ros2::srv::AddCollisionObject::Request> req,
    std::shared_ptr<planning_scene_ros2::srv::AddCollisionObject::Response> res)
  {
    // Input validation, no negative values
    if (req->l <= 0.0 || req->w <= 0.0 || req->h <= 0.0) {
      res->success = false;
      res->message = "Box dimensions must be > 0.";
      RCLCPP_WARN(this->get_logger(), "%s (l=%.3f w=%.3f h=%.3f)", res->message.c_str(), req->l, req->w, req->h);
      return;
    }

    // Create a new collision object
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = frame_id_;

    // Give the object a unique name based on how many collision objects already exist
    obj.id = "box_" + std::to_string(object_counter_.fetch_add(1));

    // Define a BOX primitive (dimensions are [req->l, req->w, req->h])
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = req->l;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = req->w;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = req->h;

    // Define object pose (located at [req->x, req->y, req->z]), identity orientation
    geometry_msgs::msg::Pose pose;
    pose.position.x = req->x;
    pose.position.y = req->y;
    pose.position.z = req->z;
    pose.orientation.w = 1.0;

    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(pose);
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;

    // Apply to MoveIt's planning scene (synchronous call into move_group planning scene interface)
    bool ok = false;
    try {
      ok = planning_scene_interface_.applyCollisionObject(obj);
    } catch (const std::exception& e) {
      res->success = false;
      res->message = std::string("Exception while applying collision object: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
      return;
    }

    if (!ok) {
      res->success = false;
      res->message = "applyCollisionObject() returned false (move_group may be unavailable or scene update failed).";
      RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
      return;
    }

    res->success = true;
    res->message = "Added collision object id='" + obj.id + "' in frame '" + frame_id_ + "'.";
    RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AddCollisionObjectNode>());
  rclcpp::shutdown();
  return 0;
}