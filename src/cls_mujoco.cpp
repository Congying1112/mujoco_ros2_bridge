#include "cls_mujoco.hpp"
#include <stdexcept>
#include <thread>
#include <chrono>
#include <functional>
#include <filesystem>
using namespace std;

ClsMujoco::ClsMujoco(const std::string &model_file, const std::string &name)
{
    // load and compile model
    char error[1000] = "Could not load binary model";
    filesystem::path model_path(model_file);
    if (!filesystem::exists(model_path))
    {
        mju_error("Model file does not exist: %s", model_file.c_str());
        throw std::runtime_error("Model file does not exist");
    }
    if (model_path.extension() == ".mjb")
    {
        model_ = mj_loadModel(model_path.c_str(), 0);
    }
    else if (model_path.extension() == ".xml")
    {
        model_ = mj_loadXML(model_path.c_str(), 0, error, 1000);
    }
    else
    {
        mju_error("Unsupported model file format: %s", model_path.extension().c_str());
        throw std::runtime_error("Unsupported model file format");
    }
    if (!model_)
    {
        mju_error("Load model error: %s", error);
        throw std::runtime_error("Failed to load MuJoCo model");
    }
    else
    {
        cout << "Model with " << model_->nu << " actuators has been loaded: " << endl;
    }

    // make data
    data_ = mj_makeData(model_);
    if (!data_)
        throw std::runtime_error("Failed to create MuJoCo data");

    // init GLFW
    if (!glfwInit())
    {
        mju_error("Could not initialize GLFW");
    }

    // create window, make OpenGL context current, request v-sync
    window_ = glfwCreateWindow(1200, 900, name == "" ? model_path.stem().string().c_str() : name.c_str(), NULL, NULL);
    if (!window_)
        throw std::runtime_error("Failed to create GLFW window");
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&camera_);
    mjv_defaultOption(&option_);
    mjv_defaultPerturb(&perturb_);
    mjv_defaultScene(&scene_);
    mjr_defaultContext(&context_);

    // create scene and context
    mjv_makeScene(model_, &scene_, 2000);
    mjr_makeContext(model_, &context_, mjFONTSCALE_150);

    actuator_joint_state_msg_.name.resize(model_->nu);
    actuator_joint_state_msg_.position.resize(model_->nu);
    actuator_joint_state_msg_.velocity.resize(model_->nu);
    actuator_joint_state_msg_.effort.resize(model_->nu);


    joint_state_msg_.name.resize(model_->nq);
    joint_state_msg_.position.resize(model_->nq);
    joint_state_msg_.velocity.resize(model_->nq);
    joint_state_msg_.effort.resize(model_->nq);

    // install GLFW mouse and keyboard callbacks
    glfwSetWindowUserPointer(window_, this);
    glfwSetKeyCallback(window_, [](GLFWwindow *w, int key, int sc, int act, int mods)
                       { static_cast<ClsMujoco *>(glfwGetWindowUserPointer(w))->keyboard(w, key, sc, act, mods); });
    glfwSetCursorPosCallback(window_, [](GLFWwindow *w, double xpos, double ypos)
                             { static_cast<ClsMujoco *>(glfwGetWindowUserPointer(w))->mouse_move(w, xpos, ypos); });
    glfwSetMouseButtonCallback(window_, [](GLFWwindow *w, int button, int act, int mods)
                               { static_cast<ClsMujoco *>(glfwGetWindowUserPointer(w))->mouse_button(w, button, act, mods); });
    glfwSetScrollCallback(window_, [](GLFWwindow *w, double xoffset, double yoffset)
                          { static_cast<ClsMujoco *>(glfwGetWindowUserPointer(w))->scroll(w, xoffset, yoffset); });
}

ClsMujoco::~ClsMujoco()
{
    mj_deleteData(data_);
    mj_deleteModel(model_);
    mjv_freeScene(&scene_);
    mjr_freeContext(&context_);
    glfwDestroyWindow(window_);
    glfwTerminate();
}

// 由于glfw的窗口需要在主线程中运行，故无法在后台线程中直接调用main_loop()。
// 可直接调用main_loop()阻塞运行，或者另起timer将update_visualization()作为回调函数。
void ClsMujoco::main_loop()
{
    while (!glfwWindowShouldClose(window_))
    {
        update_visualization();
    }
}

void ClsMujoco::update_visualization()
{
    mjtNum simstart = data_->time;
    while (data_->time - simstart < 1.0 / simFrequency_)
    {
        mj_step(model_, data_);
    }

    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(model_, data_, &option_, NULL, &camera_, mjCAT_ALL, &scene_);
    mjr_render(viewport, &scene_, &context_);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window_);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
}

void ClsMujoco::start_testPositionControl()
{
    cout << "[INFO] Starting test position control..." << endl;
    std::thread testThread([this]()
                           {
        double actuator_pos = 0.0;
        while (true) {
            cout << "[INFO] Setting actuators to: " << actuator_pos << endl;
            for (int i = 0; i < model_->nu; i++) {
                // data_->qpos[i] = actuator_pos;
                // data_->qvel[i] = actuator_pos;
                data_->ctrl[i] = actuator_pos;
            }
            actuator_pos += 0.03;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        } });
    testThread.detach();
}

void ClsMujoco::set_zero_pos()
{
    cout << "[INFO] Setting zero position..." << endl;
    for (int i = 0; i < model_->nu; i++)
    {
        data_->ctrl[i] = 0.0;
    }
}

void ClsMujoco::keyboard(GLFWwindow *window, int key, int scancode, int act, int mods)
{
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        mj_resetData(model_, data_);
        mj_forward(model_, data_);
    }
}

void ClsMujoco::mouse_button(GLFWwindow *window, int button, int act, int mods)
{
    button_left_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    glfwGetCursorPos(window, &lastx_, &lasty_);
}

void ClsMujoco::mouse_move(GLFWwindow *window, double xpos, double ypos)
{
    if (!button_left_ && !button_middle_ && !button_right_)
        return;
    double dx = xpos - lastx_;
    double dy = ypos - lasty_;
    lastx_ = xpos;
    lasty_ = ypos;
    int width, height;
    glfwGetWindowSize(window, &width, &height);
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
    mjtMouse action;
    if (button_right_)
    {
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    }
    else if (button_left_)
    {
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    }
    else
    {
        action = mjMOUSE_ZOOM;
    }
    mjv_moveCamera(model_, action, dx / height, dy / height, &scene_, &camera_);
}

void ClsMujoco::scroll(GLFWwindow *window, double xoffset, double yoffset)
{
    mjv_moveCamera(model_, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scene_, &camera_);
}

void ClsMujoco::actuator_command_callback(const std_msgs::msg::Float64MultiArray &msg, std::function<void(int, const std::string &)> cb)
{
    if (!model_ || !data_)
    {
        cb(1, "MuJoCo model or data is not initialized.");
        return;
    }

    if (msg.data.size() != model_->nu)
    {
        cb(2, "Received actuator command size does not match model's actuator count.");
        return;
    }

    for (size_t i = 0; i < msg.data.size(); ++i)
    {
        data_->ctrl[i] = msg.data[i];
    }
    cb(0, ""); // 默认msg为空字符串
}

void ClsMujoco::get_joint_state(std::function<void(int, sensor_msgs::msg::JointState, const std::string &)> cb)
{
    if (!model_ || !data_)
    {
        cb(1, joint_state_msg_, "MuJoCo model or data is not initialized.");
        return;
    }

    // Add joint state data to ROS2 message
    for (int i = 0; i < model_->nq; ++i)
    {
        joint_state_msg_.position[i] = data_->qpos[i];
        joint_state_msg_.velocity[i] = data_->qvel[i];
        joint_state_msg_.effort[i] = data_->qfrc_actuator[i];
    }
    cb(0, joint_state_msg_, ""); // 默认msg为空字符串
}

void ClsMujoco::get_actuator_joint_state(std::function<void(int, sensor_msgs::msg::JointState, const std::string &)> cb)
{
    if (!model_ || !data_)
    {
        cb(1, actuator_joint_state_msg_, "MuJoCo model or data is not initialized.");
        return;
    }

    // Add actuator state data to ROS2 message
    for (int i = 0; i < model_->nu; ++i)
    {
        int joint_id = mj_name2id(model_, mjOBJ_JOINT, mj_id2name(model_, mjOBJ_ACTUATOR, i));
        actuator_joint_state_msg_.position[i] = data_->qpos[joint_id];
        actuator_joint_state_msg_.velocity[i] = data_->qvel[joint_id];
        actuator_joint_state_msg_.effort[i] = data_->qfrc_actuator[joint_id];
    }
    cb(0, actuator_joint_state_msg_, ""); // 默认msg为空字符串
}
