#include <rclcpp/rclcpp.hpp>
#include "cls_mujoco_node.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("mujoco_node"), "Usage: mujoco_node <model_file.xml>");
        return 1;
    }
    auto node = std::make_shared<ClsMujocoNode>(argv[1]);
    RCLCPP_INFO(rclcpp::get_logger("mujoco_node"), "MuJoCo node started with model file: %s", argv[1]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}