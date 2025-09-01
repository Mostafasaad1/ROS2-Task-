#include "ros2_eval_task/model_utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <sstream>
#include <dirent.h>
#include <iostream>

namespace ros2_eval_task
{

/// Discover all valid models inside the package's "models" folder
std::vector<ModelInfo> get_available_models()
{
  std::vector<ModelInfo> models;

  std::string share_dir = ament_index_cpp::get_package_share_directory("ros2_eval_task");
  std::string models_dir = share_dir + "/models";

  DIR *dir;
  struct dirent *entry;

  if ((dir = opendir(models_dir.c_str())) != nullptr) {
    while ((entry = readdir(dir)) != nullptr) {
      if (entry->d_type != DT_DIR) {
        continue;
      }

      std::string dir_name(entry->d_name);
      if (dir_name == "." || dir_name == "..") {
        continue;
      }

      ModelInfo model;
      model.name = dir_name;
      model.path = models_dir + "/" + model.name + "/model.sdf";

      // Accept only if model.sdf exists
      std::ifstream file(model.path);
      if (file.good()) {
        models.push_back(model);
      }
    }
    closedir(dir);
  } else {
    std::cerr << "⚠️  Could not open models directory: " << models_dir << std::endl;
  }

  return models;
}

/// Load the raw SDF string for a specific model
std::string load_model_sdf(const std::string &model_name)
{
  std::string share_dir = ament_index_cpp::get_package_share_directory("ros2_eval_task");
  std::string model_path = share_dir + "/models/" + model_name + "/model.sdf";

  std::ifstream file(model_path);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open model file: " + model_path);
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

} // namespace ros2_eval_task
