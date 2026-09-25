#include "test.h"
#include <iostream>
#include <set>

int main(int argc, char **argv)
{
    std::set<std::string> ids;
    for (const auto& test : testCases()) {
        if (!ids.insert(test.id).second) { std::cerr << "duplicate ID\n"; return 2; }
    }
    if (argc == 2 && std::string(argv[1]) == "--list") {
        for (const auto& test : testCases()) std::cout << test.id << '\t' << test.name << '\n';
        return testCases().empty() ? 2 : 0;
    }
    if (argc == 3 && std::string(argv[1]) == "--case") {
        for (const auto& test : testCases()) if (test.id == std::string(argv[2])) {
            try { test.run(); std::cout << "PASS " << test.id << '\n'; return 0; }
            catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
            catch (...) { std::cerr << "unknown exception\n"; return 1; }
        }
    }
    std::cerr << "Usage: zephcore_unit --list | --case <exact-id>\n";
    return 2;
}
