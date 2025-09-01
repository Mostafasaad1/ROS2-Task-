#include "ros2_eval_task/gazebo_utils_client.hpp"
#include "ros2_eval_task/model_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <random>
#include <chrono>
#include <fstream>
#include <sstream>

using namespace std::chrono_literals;

class CameraSpawnerNode : public rclcpp::Node
{
public:
  CameraSpawnerNode()
  : Node("camera_model_spawner_node"), image_index_(0)
  {
    // Initialize Gazebo helper
    gazebo_client_ = std::make_shared<GazeboUtilsClient>();

    // List of available battery models
    available_models_ = {
      "battery_9v_leader",
      "battery_energizer",
      "battery_varita",
      "lipo_battery"
    };

    // Load model XMLs in advance
    for (const auto &model_name : available_models_) {
      try {
        model_xmls_[model_name] = load_model_file(model_name);
        RCLCPP_INFO(get_logger(), "Model loaded: %s", model_name.c_str());
      } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "Could not load model %s: %s",
                     model_name.c_str(), e.what());
      }
    }

    // Random pose generator setup
    rng_ = std::mt19937(std::random_device{}());
    x_range_ = std::uniform_real_distribution<>(-0.21, 0.21);
    y_range_ = std::uniform_real_distribution<>(-0.43, 0.43);

    // Subscribe to camera feed
    camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10,
      std::bind(&CameraSpawnerNode::handle_camera_frame, this, std::placeholders::_1));

    // Timer for spawning and removing models
    spawn_timer_ = create_wall_timer(
      5s, std::bind(&CameraSpawnerNode::handle_timer_tick, this));

    RCLCPP_INFO(get_logger(), "Camera spawner node running. New model every 5 seconds.");
  }

private:
  /// Load SDF XML string of a given model
  std::string load_model_file(const std::string &model_name)
  {
    std::string share_dir = ament_index_cpp::get_package_share_directory("ros2_eval_task");
    std::string model_path = share_dir + "/models/" + model_name + "/model.sdf";

    std::ifstream file(model_path);
    if (!file.is_open()) {
      throw std::runtime_error("Could not open file: " + model_path);
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
  }

  /// Callback: receives camera image
  void handle_camera_frame(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      last_frame_ = cv_ptr->image.clone();
      frame_ready_ = true;
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge error: %s", e.what());
    }
  }

  /// Save last received camera frame to file
  bool save_last_frame()
  {
    if (last_frame_.empty() || !frame_ready_) {
      return false;
    }

    std::string filename = "image_" + std::to_string(image_index_++) + ".png";
    if (cv::imwrite(filename, last_frame_)) {
      RCLCPP_INFO(get_logger(), "Saved image: %s", filename.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to save: %s", filename.c_str());
    }

    frame_ready_ = false;
    return true;
  }

  /// Timer callback: removes previous model, spawns a new one, then captures image
  void handle_timer_tick()
  {
    // Remove previous model
    if (!current_model_.empty()) {
      RCLCPP_INFO(get_logger(), "Removing model: %s", current_model_.c_str());
      if (!gazebo_client_->delete_model(current_model_)) {
        RCLCPP_ERROR(get_logger(), "Failed to delete model: %s", current_model_.c_str());
      }
      current_model_.clear();
    }

    // Pick random model
    std::uniform_int_distribution<> model_selector(0, available_models_.size() - 1);
    std::string model_name = available_models_[model_selector(rng_)];

    // Define pose
    geometry_msgs::msg::Pose pose;
    pose.position.x = x_range_(rng_);
    pose.position.y = y_range_(rng_);
    pose.position.z = 1.1;
    pose.orientation.w = 1.0;

    RCLCPP_INFO(get_logger(), "Spawning model: %s at (%.2f, %.2f, %.2f)",
                model_name.c_str(),
                pose.position.x, pose.position.y, pose.position.z);

    // Spawn and save
    if (gazebo_client_->spawn_model(model_name, model_xmls_[model_name], pose)) {
      current_model_ = model_name;
      rclcpp::sleep_for(500ms);
      save_last_frame();
    } else {
      RCLCPP_ERROR(get_logger(), "Could not spawn model: %s", model_name.c_str());
    }
  }

  // Members
  std::shared_ptr<GazeboUtilsClient> gazebo_client_;
  std::vector<std::string> available_models_;
  std::map<std::string, std::string> model_xmls_;
  rclcpp::TimerBase::SharedPtr spawn_timer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  std::string current_model_;

  cv::Mat last_frame_;
  bool frame_ready_{false};
  int image_index_;

  std::mt19937 rng_;
  std::uniform_real_distribution<> x_range_;
  std::uniform_real_distribution<> y_range_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraSpawnerNode>());
  rclcpp::shutdown();
  return 0;
}
