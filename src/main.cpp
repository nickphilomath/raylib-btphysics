#include <iostream>
#include <exception>

#include "Application.hpp"

int main() {
    try {
        Application app;
        app.run();
    } catch (const std::exception& e) {
        std::cerr << "Unhandled exception: " << e.what() << std::endl;
        return EXIT_FAILURE;
    } catch (...) {
        std::cerr << "Unhandled unknown exception occurred." << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}