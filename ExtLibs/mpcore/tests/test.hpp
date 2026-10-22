#pragma once

// A minimal test harness, so the core builds and tests with nothing but a compiler and CMake.

#include <cstdint>
#include <string>
#include <type_traits>
#include <vector>

namespace test {

struct Case {
    const char *name;
    void (*fn)();
};

std::vector<Case> &registry();
void fail(const char *file, int line, const std::string &what);

struct Register {
    Register(const char *name, void (*fn)()) { registry().push_back({name, fn}); }
};

template <typename T>
std::string show(const T &v) {
    if constexpr (std::is_same_v<T, bool>)
        return v ? "true" : "false";
    else if constexpr (std::is_enum_v<T>)
        return std::to_string(static_cast<long long>(v));
    else if constexpr (std::is_integral_v<T> && std::is_signed_v<T>)
        return std::to_string(static_cast<long long>(v));
    else if constexpr (std::is_integral_v<T>)
        return std::to_string(static_cast<unsigned long long>(v));
    else if constexpr (std::is_floating_point_v<T>)
        return std::to_string(v);
    else if constexpr (std::is_convertible_v<T, std::string>)
        return "\"" + std::string(v) + "\"";
    else
        return "?";
}

}  // namespace test

#define TEST(name)                                           \
    static void name();                                      \
    static const ::test::Register name##_register(#name, name); \
    static void name()

#define CHECK(cond)                                    \
    do {                                               \
        if (!(cond))                                   \
            ::test::fail(__FILE__, __LINE__, #cond);   \
    } while (0)

#define CHECK_EQ(a, b)                                                                     \
    do {                                                                                   \
        const auto &check_a_ = (a);                                                        \
        const auto &check_b_ = (b);                                                        \
        if (!(check_a_ == check_b_))                                                       \
            ::test::fail(__FILE__, __LINE__,                                               \
                         std::string(#a " == " #b "  (") + ::test::show(check_a_) + " vs " + \
                             ::test::show(check_b_) + ")");                                \
    } while (0)
