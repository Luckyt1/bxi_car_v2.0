#pragma once

#include <functional>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace iswv::test {

struct Case {
    std::string name;
    std::function<void()> function;
};

inline std::vector<Case>& registry()
{
    static std::vector<Case> cases;
    return cases;
}

class Registration {
public:
    Registration(std::string name, std::function<void()> function)
    {
        registry().push_back(Case{std::move(name), std::move(function)});
    }
};

inline void fail(const char* expression, const char* file, int line,
                 const std::string& detail = {})
{
    std::ostringstream message;
    message << file << ':' << line << ": check failed: " << expression;
    if (!detail.empty()) {
        message << " (" << detail << ')';
    }
    throw std::runtime_error(message.str());
}

}  // namespace iswv::test

#define ISWV_TEST_CONCAT_INNER(a, b) a##b
#define ISWV_TEST_CONCAT(a, b) ISWV_TEST_CONCAT_INNER(a, b)
#define TEST_CASE(name)                                                              \
    static void ISWV_TEST_CONCAT(test_function_, __LINE__)();                        \
    static ::iswv::test::Registration ISWV_TEST_CONCAT(test_registration_, __LINE__)( \
        name, ISWV_TEST_CONCAT(test_function_, __LINE__));                           \
    static void ISWV_TEST_CONCAT(test_function_, __LINE__)()

#define CHECK(expression)                                                            \
    do {                                                                             \
        if (!(expression)) {                                                         \
            ::iswv::test::fail(#expression, __FILE__, __LINE__);                     \
        }                                                                            \
    } while (false)

#define CHECK_EQ(lhs, rhs)                                                           \
    do {                                                                             \
        const auto check_lhs = (lhs);                                                \
        const auto check_rhs = (rhs);                                                \
        if (!(check_lhs == check_rhs)) {                                             \
            ::iswv::test::fail(#lhs " == " #rhs, __FILE__, __LINE__);               \
        }                                                                            \
    } while (false)

#define CHECK_NEAR(lhs, rhs, tolerance)                                              \
    do {                                                                             \
        const auto check_lhs = (lhs);                                                \
        const auto check_rhs = (rhs);                                                \
        const auto check_tolerance = (tolerance);                                    \
        if (!((check_lhs >= check_rhs - check_tolerance) &&                          \
              (check_lhs <= check_rhs + check_tolerance))) {                        \
            ::iswv::test::fail(#lhs " ~= " #rhs, __FILE__, __LINE__);               \
        }                                                                            \
    } while (false)
