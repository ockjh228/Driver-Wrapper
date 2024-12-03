/**
 * @file parameter_manager.hpp
 * @author kyungpyo kim (kyungpyo.kim@acelab.ai)
 * @brief
 * @version 0.1
 * @date 2022-11-14
 *
 * @copyright Copyright (c)ACELAB 2022
 *
 */

#ifndef BASE_PARAMETER_MANAGER_HPP_  // NOLINT
#define BASE_PARAMETER_MANAGER_HPP_  // NOLINT

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <experimental/filesystem>
#include <memory>
#include <string>
#include <thread>  // NOLINT
#include <vector>

class ParameterManager {
public:
  /**
   * @brief Construct a new Parameter Manager object
   *
   * throw exception if config file is not vaild
   *
   * @param param_path
   * @param logger
   * @param check_duration
   */
  ParameterManager(const std::string &param_path,
                   std::shared_ptr<spdlog::logger> logger,
                   const double &check_duration = 1.0)
      : logger_(logger), path_(param_path), check_duration_(check_duration) {
    try {
      node_ = YAML::LoadFile(path_.string());
    } catch (const YAML::BadFile &e) {
      logger_->error("Failed to load parameter file: {}", e.what());
      logger_->error(" - check file path: {}", path_.string());
      throw;
    }

    last_write_time_ = check_time_ = std::chrono::system_clock::to_time_t(
      std::experimental::filesystem::last_write_time(path_));

    std::thread check_thread(&ParameterManager::check, this);
    check_thread.detach();
  }

  /**
   * @brief Destroy the Parameter Manager object
   *
   * termiate check thread using terminate_ and check_terminate_ flags
   *
   */
  ~ParameterManager() {
    terminate_ = true;
    while (!check_terminate_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    logger_->info("ParameterManager is terminated");
  }

public:
  /**
   * @brief update yaml paramter file
   *
   * if parameter file is updated, update node_ and last_write_time_ and return
   * true if parameter file is not updated, return false
   *
   * @return true if parameter file is updated
   * @return false if parameter file is not updated
   */
  bool update() {
    if (file_check_) {
      return false;
    }
    if (last_write_time_ != check_time_) {
      last_write_time_ = check_time_;
      node_ = YAML::LoadFile(path_.string());
    }
    return true;
  }

  /**
   * @brief get parameter value
   *
   * @tparam T type of parameter value
   * @param key key of parameter
   * @param default_value default value of parameter
   * @return const T parameter value
   */
  template <class T>
  const T get(const std::vector<std::string> &key,
              const T &default_value) const {
    try {
      return getNode(key).as<T>();
    } catch (const YAML::Exception &e) {
      logger_->warn("ParameterManager::get() failed: {}", e.what());
      std::string s;
      for (const auto &k : key) {
        s += "/" + k;
      }
      logger_->warn("key: {}", s);
      logger_->warn("return default value: {}", default_value);
      return default_value;
    }
  }

  /**
   * @brief get parameter value
   *
   * @tparam T type of parameter value
   * @param key key of parameter
   * @return T parameter value
   */
  template <class T>
  const T get(const std::vector<std::string> &key) const {
    try {
      return getNode(key).as<T>();
    } catch (const YAML::Exception &e) {
      logger_->error("ParameterManager::get() failed: {}", e.what());
      logger_->error(" - no default value is set");
      std::string s;
      for (const auto &k : key) {
        s += "/" + k;
      }
      logger_->error("key: {}", s);
      return T();
    }
  }

  /**
   * @brief get parameter node
   *
   * @param keys key of parameter
   *
   * @return node
   */
  const YAML::Node getNode(const std::vector<std::string> &key) const {
    try {
      std::vector<YAML::Node> n{node_};
      for (const auto &k : key) {
        auto child = n.back()[k];
        n.push_back(child);
      }
      return n.back();
    } catch (const YAML::Exception &e) {
      logger_->error("ParameterManager::get() failed: {}", e.what());
      logger_->error(" - no default value is set");
      std::string s;
      for (const auto &k : key) {
        s += "/" + k;
      }
      logger_->error("key: {}", s);
      return YAML::Node();
    }
  }

private:
  /**
   * @brief check parameter file
   *
   */
  void check() {
    while (!terminate_) {
      file_check_ = true;
      check_time_ = std::chrono::system_clock::to_time_t(
        std::experimental::filesystem::last_write_time(path_));
      file_check_ = false;

      std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(check_duration_ * 1000)));
    }
    logger_->info("ParameterManager::check() thread terminated!");
    check_terminate_ = true;
  }

private:
  std::shared_ptr<spdlog::logger> logger_;

private:
  YAML::Node node_;
  std::experimental::filesystem::path path_;
  double check_duration_;
  std::time_t last_write_time_, check_time_;

private:
  std::atomic_bool file_check_ = false;
  std::atomic_bool terminate_ = false;
  std::atomic_bool check_terminate_ = false;
};

#endif  // BASE_PARAMETER_MANAGER_HPP_ // NOLINT
