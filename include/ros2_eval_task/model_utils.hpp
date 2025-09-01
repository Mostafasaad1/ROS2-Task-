#ifndef ROS2_EVAL_TASK__MODEL_UTILS_HPP_
#define ROS2_EVAL_TASK__MODEL_UTILS_HPP_

#include <string>
#include <vector>

namespace ros2_eval_task
{

struct ModelInfo {
  std::string name;
  std::string path;
};

std::vector<ModelInfo> get_available_models();
std::string load_model_sdf(const std::string& model_name);

}  // namespace ros2_eval_task

#endif  // ROS2_EVAL_TASK__MODEL_UTILS_HPP_