#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

int main() {
    try {
        franka::Robot robot("172.16.0.2");

        franka::RobotState state = robot.readOnce();

        franka::Model model = robot.loadModel();
        
        std::array<double, 7> gravity_array = model.gravity(state);

        std::cout << "Gravity Vector: " << std::endl;
        
        for (const auto &value: gravity_array) {
            std::cout << value << ' ';
        }
        std::cout << "\n" << std::endl;

        std::cout << "Coriolis Vector:" << std::endl;
        
        std::array<double, 7> coriolis_array = model.coriolis(state);

        for (const auto &value: coriolis_array) {
            std::cout << value << ' ';
        }
        std::cout << "\n" << std::endl;

        std::cout << "Mass Vector:" << std::endl;
        
        std::array<double, 49> mass_array = model.mass(state);

        for (const auto &value: mass_array) {
            std::cout << value << ' ';
        }
        std::cout << "\n" << std::endl;

        std::cout << "Joint Angles:" << std::endl;

        for (const auto &value: state.q){
            std::cout << value << " ";
        }
        std::cout << "" << std::endl;
    }
    catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }
    return 0;
}