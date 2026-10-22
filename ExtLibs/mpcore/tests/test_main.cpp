#include <cstdio>
#include <cstring>

#include "test.hpp"

namespace test {

static int failures_in_case = 0;

std::vector<Case> &registry() {
    static std::vector<Case> cases;
    return cases;
}

void fail(const char *file, int line, const std::string &what) {
    failures_in_case++;
    std::printf("    %s:%d: CHECK failed: %s\n", file, line, what.c_str());
}

}  // namespace test

// Usage: mpcore_tests [substring]  (runs the cases whose name contains substring)
int main(int argc, char **argv) {
    const char *filter = argc > 1 ? argv[1] : nullptr;
    int run = 0;
    int failed = 0;
    for (const auto &c : test::registry()) {
        if (filter != nullptr && std::strstr(c.name, filter) == nullptr)
            continue;
        test::failures_in_case = 0;
        c.fn();
        run++;
        if (test::failures_in_case != 0) {
            failed++;
            std::printf("FAIL %s\n", c.name);
        } else {
            std::printf("ok   %s\n", c.name);
        }
    }
    std::printf("\n%d tests, %d failed\n", run, failed);
    return failed == 0 && run > 0 ? 0 : 1;
}
