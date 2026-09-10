#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <variant>

namespace iswv {

enum class ErrorCode : std::uint8_t {
    none = 0,
    invalid_argument,
    transport_closed,
    transport_error,
    timeout,
    protocol_error,
    sdo_abort,
    unsupported,
    illegal_state,
    would_deadlock,
    queue_overflow,
    cancelled,
    not_found,
    busy,
    internal_error
};

struct Error {
    ErrorCode code{ErrorCode::none};
    std::string message;
    std::optional<std::uint32_t> sdo_abort_code;

    explicit operator bool() const noexcept { return code != ErrorCode::none; }
};

template <typename T>
class Result {
public:
    static Result success(T value) { return Result(std::move(value)); }
    static Result failure(Error error) { return Result(std::move(error)); }
    static Result failure(ErrorCode code, std::string message)
    {
        return Result(Error{code, std::move(message), std::nullopt});
    }

    bool has_value() const noexcept { return std::holds_alternative<T>(storage_); }
    explicit operator bool() const noexcept { return has_value(); }

    T& value() { return std::get<T>(storage_); }
    const T& value() const { return std::get<T>(storage_); }
    T&& take_value() { return std::get<T>(std::move(storage_)); }

    Error& error() { return std::get<Error>(storage_); }
    const Error& error() const { return std::get<Error>(storage_); }

private:
    explicit Result(T value) : storage_(std::move(value)) {}
    explicit Result(Error error) : storage_(std::move(error)) {}

    std::variant<T, Error> storage_;
};

template <>
class Result<void> {
public:
    static Result success() { return Result(); }
    static Result failure(Error error) { return Result(std::move(error)); }
    static Result failure(ErrorCode code, std::string message)
    {
        return Result(Error{code, std::move(message), std::nullopt});
    }

    bool has_value() const noexcept { return !error_.has_value(); }
    explicit operator bool() const noexcept { return has_value(); }

    Error& error() { return *error_; }
    const Error& error() const { return *error_; }

private:
    Result() = default;
    explicit Result(Error error) : error_(std::move(error)) {}

    std::optional<Error> error_;
};

inline Error make_error(ErrorCode code, std::string message)
{
    return Error{code, std::move(message), std::nullopt};
}

inline Error make_sdo_abort(std::uint32_t abort_code, std::string message)
{
    return Error{ErrorCode::sdo_abort, std::move(message), abort_code};
}

}  // namespace iswv
