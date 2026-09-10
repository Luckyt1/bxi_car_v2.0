#include "chassis/steering_config.hpp"

#include <sys/wait.h>
#include <unistd.h>

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
void require(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}

int run_cli(const std::vector<std::string> & args)
{
  std::vector<char *> argv;
  argv.reserve(args.size() + 1);
  for (const auto & arg : args) {
    argv.push_back(const_cast<char *>(arg.c_str()));
  }
  argv.push_back(nullptr);

  const pid_t pid = fork();
  require(pid >= 0, "fork failed");
  if (pid == 0) {
    std::freopen("/dev/null", "w", stdout);
    std::freopen("/dev/null", "w", stderr);
    execvp(argv[0], argv.data());
    _exit(127);
  }

  int status = 0;
  require(waitpid(pid, &status, 0) == pid, "waitpid failed");
  require(WIFEXITED(status), "CLI exited from a signal");
  return WEXITSTATUS(status);
}

void require_success(const std::vector<std::string> & args, const std::string & label)
{
  const int code = run_cli(args);
  require(code == 0, label + " should succeed, got exit code " + std::to_string(code));
}

void require_failure(const std::vector<std::string> & args, const std::string & label)
{
  const int code = run_cli(args);
  require(code != 0, label + " should fail");
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    require(argc == 3, "Usage: test_steering_direction_cli BINARY UNCALIBRATED_YAML");
    const std::string binary = argv[1];
    const std::string uncalibrated_yaml = argv[2];
    const auto config = chassis::load_steering_config(uncalibrated_yaml);
    require(
      !config.steering_calibrated,
      "refuse to run steering_direction_test CLI checks with a calibrated config");

    require_success({binary, "--help"}, "--help");

    require_failure({binary}, "missing arguments");
    require_failure({binary, uncalibrated_yaml, "1"}, "missing delta");
    require_failure({binary, uncalibrated_yaml, "1", "1", "extra"}, "trailing argument");

    require_failure({binary, uncalibrated_yaml, "0", "1"}, "wheel 0");
    require_failure({binary, uncalibrated_yaml, "5", "1"}, "wheel 5");

    require_failure({binary, uncalibrated_yaml, "1", "0"}, "zero delta");
    require_failure({binary, uncalibrated_yaml, "1", "4"}, "too-large delta");
    require_failure({binary, uncalibrated_yaml, "1", "nan"}, "nan delta");
    require_failure({binary, uncalibrated_yaml, "1", "inf"}, "inf delta");
    require_failure({binary, uncalibrated_yaml, "1", "1x"}, "trailing delta characters");

    require_failure(
      {binary, uncalibrated_yaml, "1", "1"},
      "valid hardware arguments with an uncalibrated config");

    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
