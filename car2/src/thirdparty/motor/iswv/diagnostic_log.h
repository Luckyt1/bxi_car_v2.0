#pragma once

#include <memory>
#include <string>

namespace iswv::canopen
{

// Low-overhead CAN diagnostic recorder.
// record() and trigger() only touch bounded in-memory queues; disk I/O runs on
// the background thread. An empty directory disables all work.
class DiagnosticLog
{
public:
  DiagnosticLog(std::string directory, std::string channel);
  ~DiagnosticLog();

  DiagnosticLog(const DiagnosticLog &) = delete;
  DiagnosticLog & operator=(const DiagnosticLog &) = delete;
  DiagnosticLog(DiagnosticLog &&) noexcept;
  DiagnosticLog & operator=(DiagnosticLog &&) noexcept;

  void record(std::string message) noexcept;
  void trigger(std::string reason) noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace iswv::canopen
