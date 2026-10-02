#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <variant>

namespace bxi
{

enum class ErrorCode : std::uint8_t
{
  none = 0,
  invalid_argument,
  transport_closed,
  transport_error,
  protocol_error
};

struct Error
{
  ErrorCode code{ErrorCode::none};
  std::string message;

  explicit operator bool() const noexcept {return code != ErrorCode::none;}
};

template<typename T>
class Result
{
public:
  static Result success(T value) {return Result(std::move(value));}
  static Result failure(Error error) {return Result(std::move(error));}
  static Result failure(ErrorCode code, std::string message)
  {
    return Result(Error{code, std::move(message)});
  }

  bool has_value() const noexcept {return std::holds_alternative<T>(storage_);}
  explicit operator bool() const noexcept {return has_value();}

  T & value() {return std::get<T>(storage_);}
  const T & value() const {return std::get<T>(storage_);}
  T && take_value() {return std::get<T>(std::move(storage_));}

  Error & error() {return std::get<Error>(storage_);}
  const Error & error() const {return std::get<Error>(storage_);}

private:
  explicit Result(T value)
  : storage_(std::move(value)) {}
  explicit Result(Error && error)
  : storage_(std::move(error)) {}

  std::variant<T, Error> storage_;
};

template<>
class Result<void>
{
public:
  static Result success() {return Result();}
  static Result failure(Error error) {return Result(std::move(error));}
  static Result failure(ErrorCode code, std::string message)
  {
    return Result(Error{code, std::move(message)});
  }

  bool has_value() const noexcept {return !error_.has_value();}
  explicit operator bool() const noexcept {return has_value();}

  Error & error() {return *error_;}
  const Error & error() const {return *error_;}

private:
  Result() = default;
  explicit Result(Error && error)
  : error_(std::move(error)) {}

  std::optional<Error> error_;
};

}  // namespace bxi
