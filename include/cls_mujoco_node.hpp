#ifndef CLS_MUJOCO_NODE_HPP
#define CLS_MUJOCO_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "cls_mujoco.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class ClsMujocoNode : public rclcpp::Node {
public:
    ClsMujocoNode(const std::string &model_file, const std::string &node_name = "mujoco_node");
    ~ClsMujocoNode() = default;

private:
    std::shared_ptr<ClsMujoco> mujoco_;
    int viz_frequency_ = 10; // Hz
    rclcpp::TimerBase::SharedPtr viz_timer_;
    int state_pub_frequency_ = 10; // Hz
    rclcpp::TimerBase::SharedPtr joint_state_pub_timer_;
    rclcpp::TimerBase::SharedPtr actuator_joint_state_pub_timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr actuator_joint_state_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_command_subscriber_;

    void actuator_command_callback(const std_msgs::msg::Float64MultiArray& msg);
};

#endif // CLS_MUJOCO_NODE_HPP
