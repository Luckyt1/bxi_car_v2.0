#pragma once

#include "chassis/steering_config.hpp"
#include "chassis/steering_yaml_document.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <limits.h>
#include <optional>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace chassis
{

namespace detail
{

inline std::string steering_real_path(const std::string & path)
{
  char resolved[PATH_MAX]{};
  if (::realpath(path.c_str(), resolved) == nullptr) {
    throw std::runtime_error(
            "cannot resolve steering YAML path " + path + ": " + std::strerror(errno));
  }
  return resolved;
}

inline std::string steering_dirname(const std::string & path)
{
  const auto slash = path.find_last_of('/');
  if (slash == std::string::npos) {
    return ".";
  }
  if (slash == 0) {
    return "/";
  }
  return path.substr(0, slash);
}

inline std::string steering_basename(const std::string & path)
{
  const auto slash = path.find_last_of('/');
  if (slash == std::string::npos) {
    return path;
  }
  return path.substr(slash + 1);
}

inline void steering_push_optional(YAML::Node & sequence, const std::optional<int> & value)
{
  if (value) {
    sequence.push_back(*value);
  } else {
    sequence.push_back(YAML::Node(YAML::NodeType::Null));
  }
}

inline bool fill_steering_tuning_parameter(
  YAML::Node & parameters,
  const char * name,
  const std::array<std::optional<int>, 4> & values)
{
  YAML::Node node = parameters[name];
  if (node && node.IsScalar()) {
    return false;
  }

  bool changed = false;
  if (!node || node.IsNull()) {
    YAML::Node replacement(YAML::NodeType::Sequence);
    for (const auto & value : values) {
      steering_push_optional(replacement, value);
      changed = changed || static_cast<bool>(value);
    }
    if (changed) {
      parameters[name] = replacement;
    }
    return changed;
  }

  if (!node.IsSequence() || node.size() != 4) {
    throw std::runtime_error(
            std::string("invalid steering parameter ") + name +
            ": expected null, integer or four-element array");
  }

  for (std::size_t i = 0; i < 4; ++i) {
    if (node[i].IsNull() && values[i]) {
      node[i] = *values[i];
      changed = true;
    }
  }
  return changed;
}

inline bool fill_steering_tuning_parameters(
  YAML::Node & parameters,
  const SteeringConfigFile & config)
{
  bool changed = false;
  changed = fill_steering_tuning_parameter(
    parameters, "steering_velocity_loop_kp", config.steering_velocity_loop_kp) || changed;
  changed = fill_steering_tuning_parameter(
    parameters, "steering_velocity_loop_ki", config.steering_velocity_loop_ki) || changed;
  changed = fill_steering_tuning_parameter(
    parameters, "steering_velocity_feedback_filter",
    config.steering_velocity_feedback_filter) || changed;
  changed = fill_steering_tuning_parameter(
    parameters, "steering_position_loop_kp", config.steering_position_loop_kp) || changed;
  changed = fill_steering_tuning_parameter(
    parameters, "steering_position_smoothing_filter",
    config.steering_position_smoothing_filter) || changed;
  return changed;
}

inline bool fill_drive_tuning_parameters(YAML::Node & parameters, const SteeringConfigFile & config)
{
  bool changed = false;
  changed = fill_steering_tuning_parameter(
    parameters, "drive_velocity_loop_kp", config.drive_velocity_loop_kp) || changed;
  changed = fill_steering_tuning_parameter(
    parameters, "drive_velocity_loop_ki", config.drive_velocity_loop_ki) || changed;
  changed = fill_steering_tuning_parameter(
    parameters, "drive_velocity_feedback_filter",
    config.drive_velocity_feedback_filter) || changed;
  changed = fill_steering_tuning_parameter(
    parameters, "drive_position_loop_kp", config.drive_position_loop_kp) || changed;
  changed = fill_steering_tuning_parameter(
    parameters, "drive_position_smoothing_filter",
    config.drive_position_smoothing_filter) || changed;
  return changed;
}

inline void fsync_parent_directory(const std::string & path)
{
  const int fd = ::open(path.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (fd < 0) {
    throw std::runtime_error(
            "cannot open steering YAML directory " + path + ": " +
            std::strerror(errno));
  }
  if (::fsync(fd) != 0) {
    const int saved_errno = errno;
    ::close(fd);
    throw std::runtime_error(
            "cannot sync steering YAML directory " + path + ": " +
            std::strerror(saved_errno));
  }
  ::close(fd);
}

inline void write_steering_yaml_atomically(
  const std::string & target,
  const std::string & contents,
  const struct stat & metadata)
{
  const auto directory = steering_dirname(target);
  const auto basename = steering_basename(target);
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto temporary = directory + "/." + basename + ".tmp." +
    std::to_string(static_cast<long long>(::getpid())) + "." + std::to_string(stamp);

  int fd = ::open(temporary.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
  if (fd < 0) {
    throw std::runtime_error(
            "cannot create temporary steering YAML " + temporary + ": " +
            std::strerror(errno));
  }

  bool keep_temporary = true;
  try {
    const char * data = contents.data();
    std::size_t remaining = contents.size();
    while (remaining > 0) {
      const auto written = ::write(fd, data, remaining);
      if (written < 0) {
        if (errno == EINTR) {
          continue;
        }
        throw std::runtime_error(
                "cannot write temporary steering YAML " + temporary + ": " +
                std::strerror(errno));
      }
      if (written == 0) {
        throw std::runtime_error(
                "cannot write temporary steering YAML " + temporary +
                ": write returned 0");
      }
      data += written;
      remaining -= static_cast<std::size_t>(written);
    }

    if (::fchown(fd, metadata.st_uid, metadata.st_gid) != 0) {
      throw std::runtime_error(
              "cannot preserve steering YAML owner " + target + ": " +
              std::strerror(errno));
    }
    if (::fchmod(fd, metadata.st_mode & 07777) != 0) {
      throw std::runtime_error(
              "cannot preserve steering YAML permissions " + target + ": " +
              std::strerror(errno));
    }
    if (::fsync(fd) != 0) {
      throw std::runtime_error(
              "cannot sync temporary steering YAML " + temporary + ": " +
              std::strerror(errno));
    }
    const int close_result = ::close(fd);
    const int close_errno = errno;
    fd = -1;
    if (close_result != 0) {
      throw std::runtime_error(
              "cannot close temporary steering YAML " + temporary + ": " +
              std::strerror(close_errno));
    }
    if (::rename(temporary.c_str(), target.c_str()) != 0) {
      throw std::runtime_error(
              "cannot replace steering YAML " + target + ": " +
              std::strerror(errno));
    }
    keep_temporary = false;
    fsync_parent_directory(directory);
  } catch (...) {
    if (fd >= 0) {
      ::close(fd);
    }
    if (keep_temporary) {
      std::remove(temporary.c_str());
    }
    throw;
  }
}

}  // namespace detail

inline void persist_steering_tuning(const std::string & path, const SteeringConfigFile & config)
{
  if (path.empty()) {
    return;
  }

  const auto target = detail::steering_real_path(path);
  struct stat metadata {};
  if (::stat(target.c_str(), &metadata) != 0) {
    throw std::runtime_error("cannot stat steering YAML " + target + ": " + std::strerror(errno));
  }

  YAML::Node document;
  try {
    document = YAML::LoadFile(target);
  } catch (const YAML::Exception & error) {
    throw std::runtime_error("cannot load steering YAML " + target + ": " + error.what());
  }
  YAML::Node parameters = steering_parameters(document);

  bool changed = false;
  changed = detail::fill_steering_tuning_parameters(parameters, config) || changed;
  changed = detail::fill_drive_tuning_parameters(parameters, config) || changed;

  if (!changed) {
    return;
  }

  detail::write_steering_yaml_atomically(
    target, render_steering_yaml(document), metadata);
}

}  // namespace chassis
