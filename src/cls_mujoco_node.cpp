#include "cls_mujoco_node.hpp"

ClsMujocoNode::ClsMujocoNode(const std::string &model_file, const std::string &node_name)
    : rclcpp::Node(node_name), mujoco_(std::make_shared<ClsMujoco>(model_file))
{
    // Create joint state publisher and joint command subscriber
    joint_command_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("joint_commands", 1,  std::bind(&ClsMujocoNode::joint_command_callback, this, std::placeholders::_1));
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_state", 1);

    state_pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / state_pub_frequency_)),
        [this]() {
            mujoco_->get_joint_state([this](int err, sensor_msgs::msg::JointState joint_state_msg, const std::string& cb_msg) {
                if (err != 0) {
                    RCLCPP_ERROR(this->get_logger(), cb_msg.c_str());
                } else {
                    joint_state_msg.header.stamp = this->get_clock()->now();
                    joint_state_publisher_->publish(joint_state_msg);
                }
            });
        });
    viz_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / viz_frequency_)),
                                        std::bind(&ClsMujoco::update_visualization, mujoco_));

    // mujoco_->start_testPositionControl(); // Start the test position control mode
}

void ClsMujocoNode::joint_command_callback(const std_msgs::msg::Float64MultiArray& msg) {
    mujoco_->joint_command_callback(msg, [this](int err, const std::string& cb_msg) {
        if (err != 0) {
            RCLCPP_ERROR(this->get_logger(), cb_msg.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Joint command processed successfully.");
        }
    });
}
