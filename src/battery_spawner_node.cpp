#include "ros2_eval_task/gazebo_utils_client.hpp"
#include "ros2_eval_task/model_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <random>
#include <chrono>
#include <fstream>
#include <sstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

class BatterySpawner : public rclcpp::Node
{
public:
    BatterySpawner() : Node("battery_spawner_node")
    {
        InitializeGazeboClient();
        LoadBatteryModels();
        SetupRandomNumberGenerator();
        StartSpawningTimer();
        
        RCLCPP_INFO(get_logger(), "Battery spawner initialized. Spawning models every 5 seconds");
    }

private:
    void InitializeGazeboClient()
    {
        gazebo_client_ = std::make_shared<GazeboUtilsClient>();
    }

    void LoadBatteryModels()
    {
        battery_models_ = {
            "battery_9v_leader",
            "battery_energizer", 
            "battery_varita",
            "lipo_battery"
        };

        for (const auto& model_name : battery_models_)
        {
            try
            {
                std::string model_xml = LoadModelXML(model_name);
                model_xmls_[model_name] = model_xml;
                RCLCPP_INFO(get_logger(), "Successfully loaded model: %s", model_name.c_str());
            }
            catch (const std::exception& error)
            {
                RCLCPP_ERROR(get_logger(), "Failed to load model %s: %s", 
                            model_name.c_str(), error.what());
            }
        }
    }

    std::string LoadModelXML(const std::string& model_name)
    {
        std::string package_share_dir = ament_index_cpp::get_package_share_directory("ros2_eval_task");
        std::string model_path = package_share_dir + "/models/" + model_name + "/model.sdf";
        
        std::ifstream model_file(model_path);
        if (!model_file.is_open())
        {
            throw std::runtime_error("Could not open model file at: " + model_path);
        }
        
        std::stringstream file_buffer;
        file_buffer << model_file.rdbuf();
        return file_buffer.str();
    }

    void SetupRandomNumberGenerator()
    {
        random_engine_ = std::mt19937(std::random_device{}());
        x_distribution_ = std::uniform_real_distribution<>(-0.21, 0.21);
        y_distribution_ = std::uniform_real_distribution<>(-0.43, 0.43);
    }

    void StartSpawningTimer()
    {
        timer_ = create_wall_timer(
            5s,
            std::bind(&BatterySpawner::TimerCallback, this)
        );
    }

    void TimerCallback()
    {
        RemovePreviousModel();
        SpawnNewModel();
    }

    void RemovePreviousModel()
    {
        if (!current_model_name_.empty())
        {
            RCLCPP_INFO(get_logger(), "Removing previous model: %s", current_model_name_.c_str());
            if (!gazebo_client_->delete_model(current_model_name_))
            {
                RCLCPP_ERROR(get_logger(), "Failed to remove model: %s", current_model_name_.c_str());
            }
            current_model_name_.clear();
        }
    }

    void SpawnNewModel()
    {
        std::string selected_model = SelectRandomModel();
        geometry_msgs::msg::Pose spawn_pose = GenerateRandomPose();

        RCLCPP_INFO(get_logger(), "Spawning %s at position (%.2f, %.2f, %.2f)", 
                   selected_model.c_str(),
                   spawn_pose.position.x,
                   spawn_pose.position.y,
                   spawn_pose.position.z);

        if (gazebo_client_->spawn_model(selected_model, model_xmls_[selected_model], spawn_pose))
        {
            RCLCPP_INFO(get_logger(), "Successfully spawned %s", selected_model.c_str());
            current_model_name_ = selected_model;
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Failed to spawn %s", selected_model.c_str());
        }
    }

    std::string SelectRandomModel()
    {
        std::uniform_int_distribution<> model_distribution(0, battery_models_.size() - 1);
        return battery_models_[model_distribution(random_engine_)];
    }

    geometry_msgs::msg::Pose GenerateRandomPose()
    {
        geometry_msgs::msg::Pose new_pose;
        new_pose.position.x = x_distribution_(random_engine_);
        new_pose.position.y = y_distribution_(random_engine_);
        new_pose.position.z = 1.1;
        new_pose.orientation.w = 1.0;
        return new_pose;
    }

    // Member variables
    std::shared_ptr<GazeboUtilsClient> gazebo_client_;
    std::vector<std::string> battery_models_;
    std::map<std::string, std::string> model_xmls_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string current_model_name_;
    
    // Random number generation components
    std::mt19937 random_engine_;
    std::uniform_real_distribution<> x_distribution_;
    std::uniform_real_distribution<> y_distribution_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto battery_spawner_node = std::make_shared<BatterySpawner>();
    rclcpp::spin(battery_spawner_node);
    rclcpp::shutdown();
    return 0;
}