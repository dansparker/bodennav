#include "unit.hpp"

#include <cstring>

int main(int argc, char** argv) {
    const char* filter = argc > 1 ? argv[1] : nullptr;
    int run = 0;
    for (const auto& test : unit::registry()) {
        if (filter != nullptr && std::strstr(test.name, filter) == nullptr) continue;
        const int before = unit::failures();
        test.body();
        ++run;
        std::printf("%s %s\n", unit::failures() == before ? "ok  " : "FAIL", test.name);
    }
    std::printf("%d tests, %d failures\n", run, unit::failures());
    return unit::failures() == 0 ? 0 : 1;
}
