#ifndef CLS_MUJOCO_H
#define CLS_MUJOCO_H

#include <GLFW/glfw3.h>
#include <iostream>
#include <mujoco/mujoco.h>
#include <chrono>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class ClsMujoco
{
    public:
        ClsMujoco(const std::string &model_file, const std::string &name="");
        ~ClsMujoco();
        void main_loop();
        void start_testPositionControl();
        void set_zero_pos();
        void update_visualization();
        void joint_command_callback(const std_msgs::msg::Float64MultiArray& msg, std::function<void(int, const std::string&)> cb);
        void get_joint_state(std::function<void(int, sensor_msgs::msg::JointState, const std::string&)> cb);
    private:
        mjModel *model_ = nullptr;
        mjData  *data_ = nullptr;
        mjvCamera  camera_;
        mjvOption  option_;
        mjvPerturb perturb_;
        mjvScene   scene_;
        mjrContext context_;
        double simFrequency_ = 60; // Hz
        sensor_msgs::msg::JointState joint_state_msg_;

        GLFWwindow *window_;
        // mouse interaction
        bool button_left_ = false;
        bool button_middle_ = false;
        bool button_right_ =  false;
        double lastx_ = 0;
        double lasty_ = 0;

        // interaction callback function declarations
        void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
        void mouse_button(GLFWwindow* window, int button, int act, int mods);
        void mouse_move(GLFWwindow* window, double xpos, double ypos);
        void scroll(GLFWwindow* window, double xoffset, double yoffset);
};

#endif
