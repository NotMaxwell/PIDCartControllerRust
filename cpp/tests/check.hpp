// A ~50-line test harness, so the host tests stay dependency-free the way the
// Rust side's `cargo test --lib` does (the Rust crate has no dev-dependencies).
// Swap in Catch2/GoogleTest if you'd rather; the assertions map over directly.

#ifndef CART_TESTS_CHECK_HPP
#define CART_TESTS_CHECK_HPP

#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

namespace check {

inline int failures = 0;
inline int checks = 0;
inline std::string current_test;

inline void fail(const char* file, int line, const std::string& detail) {
    ++failures;
    std::printf("  FAIL %s\n       at %s:%d\n       %s\n", current_test.c_str(), file, line,
                detail.c_str());
}

inline void eq(double actual, double expected, const char* file, int line, const char* expr) {
    ++checks;
    // Exact-equality assertions in the Rust tests are mirrored with a tight
    // epsilon; both sides do the same f32 arithmetic, so this only guards
    // against a compiler being freer with intermediate precision than we are.
    if (!(std::fabs(actual - expected) <= 1e-6)) {
        fail(file, line,
             std::string(expr) + ": expected " + std::to_string(expected) + ", got " +
                 std::to_string(actual));
    }
}

inline void near(double actual, double expected, double tol, const char* file, int line,
                 const char* expr) {
    ++checks;
    if (!(std::fabs(actual - expected) < tol)) {
        fail(file, line,
             std::string(expr) + ": expected " + std::to_string(expected) + " +/- " +
                 std::to_string(tol) + ", got " + std::to_string(actual));
    }
}

// Registry of test functions, populated at static-init time by TEST().
using TestFn = void (*)();
inline std::vector<std::pair<const char*, TestFn>>& registry() {
    static std::vector<std::pair<const char*, TestFn>> tests;
    return tests;
}

struct Registrar {
    Registrar(const char* name, TestFn fn) { registry().emplace_back(name, fn); }
};

inline int run_all() {
    for (const auto& [name, fn] : registry()) {
        current_test = name;
        fn();
    }
    const int total = static_cast<int>(registry().size());
    std::printf("\n%d test%s, %d assertion%s, %d failure%s\n", total, total == 1 ? "" : "s", checks,
                checks == 1 ? "" : "s", failures, failures == 1 ? "" : "s");
    return failures == 0 ? 0 : 1;
}

}  // namespace check

#define TEST(name)                                                   \
    static void name();                                              \
    static const ::check::Registrar name##_registrar(#name, &name);  \
    static void name()

#define CHECK_EQ(actual, expected) ::check::eq((actual), (expected), __FILE__, __LINE__, #actual)
#define CHECK_NEAR(actual, expected, tol) \
    ::check::near((actual), (expected), (tol), __FILE__, __LINE__, #actual)

#endif  // CART_TESTS_CHECK_HPP
