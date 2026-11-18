#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <filesystem>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "planning_scene_ros2/srv/load_planning_scene.hpp"

namespace
{
std::vector<double> read_vector(
  const YAML::Node & node, const std::string & field, std::size_t expected_size)
{
  if (!node || !node.IsSequence() || node.size() != expected_size) {
    throw std::runtime_error(
            "'" + field + "' must contain exactly " + std::to_string(expected_size) + " numbers");
  }

  std::vector<double> values;
  values.reserve(expected_size);
  for (const auto & value : node) {
    const double parsed = value.as<double>();
    if (!std::isfinite(parsed)) {
      throw std::runtime_error("'" + field + "' values must be finite");
    }
    values.push_back(parsed);
  }
  return values;
}
}  // namespace

class LoadPlanningSceneNode : public rclcpp::Node
{
public:
  LoadPlanningSceneNode()
  : Node("load_planning_scene_node")
  {
    scenes_directory_ = std::filesystem::path(
      ament_index_cpp::get_package_share_directory("planning_scene_ros2")) / "scenes";

    service_ = this->create_service<planning_scene_ros2::srv::LoadPlanningScene>(
      "load_planning_scene",
      std::bind(
        &LoadPlanningSceneNode::handle_request, this, std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      this->get_logger(), "Load Planning Scene service ready. scenes='%s'",
      scenes_directory_.string().c_str());
  }

private:
  rclcpp::Service<planning_scene_ros2::srv::LoadPlanningScene>::SharedPtr service_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  std::filesystem::path scenes_directory_;

  void handle_request(
    const std::shared_ptr<planning_scene_ros2::srv::LoadPlanningScene::Request> req,
    std::shared_ptr<planning_scene_ros2::srv::LoadPlanningScene::Response> res)
  {
    const std::filesystem::path scene_name(req->scene_name);
    if (req->scene_name.empty() || scene_name.is_absolute() || scene_name.has_parent_path() ||
      scene_name.filename() == "." || scene_name.filename() == "..")
    {
      res->success = false;
      res->message = "scene_name must be a filename without directory components";
      RCLCPP_WARN(this->get_logger(), "%s", res->message.c_str());
      return;
    }

    const std::filesystem::path scene_path = scenes_directory_ / scene_name;
    if (!std::filesystem::is_regular_file(scene_path)) {
      res->success = false;
      res->message = "Scene file not found: " + scene_path.string();
      RCLCPP_WARN(this->get_logger(), "%s", res->message.c_str());
      return;
    }

    std::vector<moveit_msgs::msg::CollisionObject> objects;
    std::string frame_id;
    try {
      const YAML::Node scene = YAML::LoadFile(scene_path.string());
      if (!scene.IsMap() || !scene["frame_id"] || !scene["frame_id"].IsScalar()) {
        throw std::runtime_error("'frame_id' must be a non-empty string");
      }
      frame_id = scene["frame_id"].as<std::string>();
      if (frame_id.empty()) {
        throw std::runtime_error("'frame_id' must be a non-empty string");
      }

      const YAML::Node yaml_objects = scene["objects"];
      if (!yaml_objects || !yaml_objects.IsSequence()) {
        throw std::runtime_error("'objects' must be a sequence");
      }

      std::set<std::string> object_ids;
      for (std::size_t index = 0; index < yaml_objects.size(); ++index) {
        const YAML::Node yaml_object = yaml_objects[index];
        if (!yaml_object.IsMap() || !yaml_object["id"] || !yaml_object["type"]) {
          throw std::runtime_error("object " + std::to_string(index) + " requires 'id' and 'type'");
        }

        const std::string id = yaml_object["id"].as<std::string>();
        if (id.empty() || !object_ids.insert(id).second) {
          throw std::runtime_error("object ids must be non-empty and unique (invalid id: '" + id +
            "')");
        }
        const std::string type = yaml_object["type"].as<std::string>();
        if (type != "box") {
          throw std::runtime_error("unsupported object type '" + type + "' for object '" + id +
            "'");
        }

        const auto dimensions = read_vector(yaml_object["dimensions"], "dimensions", 3);
        if (dimensions[0] <= 0.0 || dimensions[1] <= 0.0 || dimensions[2] <= 0.0) {
          throw std::runtime_error("box dimensions must be greater than zero for object '" + id +
            "'");
        }
        const auto position = read_vector(yaml_object["position"], "position", 3);
        std::vector<double> orientation{0.0, 0.0, 0.0, 1.0};
        if (yaml_object["orientation"]) {
          orientation = read_vector(yaml_object["orientation"], "orientation", 4);
        }

        moveit_msgs::msg::CollisionObject object;
        object.header.frame_id = frame_id;
        object.id = id;
        object.operation = moveit_msgs::msg::CollisionObject::ADD;

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        primitive.dimensions.assign(dimensions.begin(), dimensions.end());

        geometry_msgs::msg::Pose pose;
        pose.position.x = position[0];
        pose.position.y = position[1];
        pose.position.z = position[2];
        pose.orientation.x = orientation[0];
        pose.orientation.y = orientation[1];
        pose.orientation.z = orientation[2];
        pose.orientation.w = orientation[3];

        object.primitives.push_back(primitive);
        object.primitive_poses.push_back(pose);
        objects.push_back(object);
      }
    } catch (const YAML::Exception & e) {
      res->success = false;
      res->message = "Malformed YAML in '" + req->scene_name + "': " + e.what();
      RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
      return;
    } catch (const std::exception & e) {
      res->success = false;
      res->message = "Invalid scene '" + req->scene_name + "': " + e.what();
      RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
      return;
    }

    try {
      const auto known_objects = planning_scene_interface_.getKnownObjectNames();
      if (!known_objects.empty()) {
        planning_scene_interface_.removeCollisionObjects(known_objects);
      }
      if (!objects.empty() && !planning_scene_interface_.applyCollisionObjects(objects)) {
        res->success = false;
        res->message = "MoveIt failed to apply collision objects from '" + req->scene_name + "'";
        RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
        return;
      }
    } catch (const std::exception & e) {
      res->success = false;
      res->message = std::string("MoveIt failed while resetting the planning scene: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
      return;
    }

    res->success = true;
    res->message = "Loaded " + std::to_string(objects.size()) + " object(s) from '" +
      req->scene_name + "' in frame '" + frame_id + "'";
    RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoadPlanningSceneNode>());
  rclcpp::shutdown();
  return 0;
}
