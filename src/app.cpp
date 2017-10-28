#include "globals.h"
#include "Application.h"

int main(int argc, char** argv) {

    app::Application::start(argc, argv);

    std::cout << "Application exit!" << std::endl;
}