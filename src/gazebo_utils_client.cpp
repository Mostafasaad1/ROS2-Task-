#include "ros2_eval_task/gazebo_utils_client.hpp"
#include <chrono>

using namespace std::chrono_literals;

GazeboUtilsClient::GazeboUtilsClient()
: Node("gazebo_utils_client_node")
{
    spawn_client_ = create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
    delete_client_ = create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");
    
    RCLCPP_INFO(get_logger(), "Gazebo utilities client initialized");
}

bool GazeboUtilsClient::spawn_model(
    const std::string& model_name,
    const std::string& model_xml,
    const geometry_msgs::msg::Pose& initial_pose)
{
    const auto service_timeout = 1s;
    
    // Check if the spawn service is available
    if (!spawn_client_->wait_for_service(service_timeout)) 
    {
        RCLCPP_ERROR(get_logger(), "Spawn entity service not available");
        return false;
    }

    // Prepare the spawn request
    auto spawn_request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    spawn_request->name = model_name;
    spawn_request->xml = model_xml;
    spawn_request->initial_pose = initial_pose;
    spawn_request->reference_frame = "world";

    // Send the request and wait for response
    auto future_result = spawn_client_->async_send_request(spawn_request);
    
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), future_result) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Failed to call spawn entity service");
        return false;
    }

    return future_result.get()->success;
}

bool GazeboUtilsClient::delete_model(const std::string& model_name)
{
    const auto service_timeout = 1s;
    
    // Check if the delete service is available
    if (!delete_client_->wait_for_service(service_timeout)) 
    {
        RCLCPP_ERROR(get_logger(), "Delete entity service not available");
        return false;
    }

    // Prepare the delete request
    auto delete_request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
    delete_request->name = model_name;

    // Send the request and wait for response
    auto future_result = delete_client_->async_send_request(delete_request);
    
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), future_result) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Failed to call delete entity service");
        return false;
    }

    return future_result.get()->success;
}