#ifndef ROS2_EVAL_TASK__GAZEBO_UTILS_CLIENT_HPP_
#define ROS2_EVAL_TASK__GAZEBO_UTILS_CLIENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <gazebo_msgs/srv/delete_entity.hpp>
#include <geometry_msgs/msg/pose.hpp>

class GazeboUtilsClient : public rclcpp::Node
{
public:
  GazeboUtilsClient();

  bool spawn_model(
    const std::string &model_name,
    const std::string &xml,
    const geometry_msgs::msg::Pose &pose
  );

  bool delete_model(const std::string &model_name);

private:
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_client_;
  rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_client_;
};

#endif  // ROS2_EVAL_TASK__GAZEBO_UTILS_CLIENT_HPP_