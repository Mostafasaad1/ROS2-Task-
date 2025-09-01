#include "ros2_eval_task/gazebo_utils_client.hpp"
#include <rclcpp/utilities.hpp>
#include <fstream>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>

std::string read_file(const std::string& file_path)
{
  std::ifstream file(file_path);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open file: " + file_path);
  }
  
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GazeboUtilsClient>();
  
  try {
    // Get the package share directory
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("ros2_eval_task");
    
    // Load the battery_9v_leader model
    std::string model_path = package_share_dir + "/models/battery_9v_leader/model.sdf";
    std::string model_xml = read_file(model_path);
    
    // Set the pose for the model
    geometry_msgs::msg::Pose pose;
    pose.position.x = 1.0;
    pose.position.y = 0.0;
    pose.position.z = 1.0;
    pose.orientation.w = 1.0;

    // Spawn the model
    RCLCPP_INFO(node->get_logger(), "Attempting to spawn battery_9v_leader model...");
    bool spawn_success = node->spawn_model("battery_9v_leader", model_xml, pose);
    
    if (spawn_success) {
      RCLCPP_INFO(node->get_logger(), "Spawn successful!");
      
      // Wait a bit before deleting
      rclcpp::sleep_for(std::chrono::seconds(5));
      
      // Delete the model
      RCLCPP_INFO(node->get_logger(), "Attempting to delete battery_9v_leader model...");
      bool delete_success = node->delete_model("battery_9v_leader");
      
      if (delete_success) {
        RCLCPP_INFO(node->get_logger(), "Delete successful!");
      } else {
        RCLCPP_ERROR(node->get_logger(), "Delete failed!");
      }
    } else {
      RCLCPP_ERROR(node->get_logger(), "Spawn failed!");
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}