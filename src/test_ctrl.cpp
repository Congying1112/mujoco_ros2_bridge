#include "cls_mujoco.hpp"
#include <iostream>
#include <memory>
#include <thread>

int main(int argc, char *argv[])
{
    if(argc < 2)
    {
        throw std::invalid_argument("[ERROR] Invalid number of arguments. Usage: mujoco_node path/to/scene.xml");                                  
    }
    std::string xmlPath = argv[1];
    try
    {
        auto mujoco = std::make_shared<ClsMujoco>(xmlPath);
        mujoco->start_testPositionControl(); // Start the test position control mode
        mujoco->main_loop(); // Start the main loop of the MuJoCoROS class
        return 0;
    }
    catch(const std::exception &exception)
    {
        std::cerr << "[ERROR] " << exception.what() << std::endl;                                   // Print error message
        return 1;                                                                                   // Flag error
    }
}
