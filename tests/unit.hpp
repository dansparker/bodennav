#pragma once

// Minimal test framework (no dependencies).

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

namespace unit {

struct Test {
    const char* name;
    std::function<void()> body;
};

inline std::vector<Test>& registry() {
    static std::vector<Test> tests;
    return tests;
}

inline int& failures() {
    static int count = 0;
    return count;
}

struct Register {
    Register(const char* name, std::function<void()> body) { registry().push_back({name, std::move(body)}); }
};

inline void fail(const char* file, int line, const std::string& what) {
    std::printf("  FAIL %s:%d: %s\n", file, line, what.c_str());
    ++failures();
}

}  // namespace unit

#define UNIT_CAT2(a, b) a##b
#define UNIT_CAT(a, b) UNIT_CAT2(a, b)
#define TEST(name)                                                                    \
    static void name();                                                               \
    static const unit::Register UNIT_CAT(register_, name)(#name, name);               \
    static void name()

#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) unit::fail(__FILE__, __LINE__, "CHECK(" #cond ")"); \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                                          \
    do {                                                                                               \
        const double unit_a = (a);                                                                     \
        const double unit_b = (b);                                                                     \
        if (!(std::fabs(unit_a - unit_b) <= (tol))) {                                                  \
            char unit_buf[256];                                                                        \
            std::snprintf(unit_buf, sizeof(unit_buf), "%s = %.6g, expected %.6g +/- %.3g", #a, unit_a, \
                          unit_b, static_cast<double>(tol));                                           \
            unit::fail(__FILE__, __LINE__, unit_buf);                                                  \
        }                                                                                              \
    } while (0)

// Angles: difference wrapped to +/-180 deg.
#define CHECK_ANGLE(a, b, tol)                                                                           \
    do {                                                                                                 \
        const double unit_a = (a);                                                                       \
        const double unit_b = (b);                                                                       \
        double unit_d = std::fmod(unit_a - unit_b + 540.0, 360.0) - 180.0;                               \
        if (!(std::fabs(unit_d) <= (tol))) {                                                             \
            char unit_buf[256];                                                                          \
            std::snprintf(unit_buf, sizeof(unit_buf), "%s = %.3f deg, expected %.3f +/- %.2f", #a, unit_a, \
                          unit_b, static_cast<double>(tol));                                             \
            unit::fail(__FILE__, __LINE__, unit_buf);                                                    \
        }                                                                                                \
    } while (0)
