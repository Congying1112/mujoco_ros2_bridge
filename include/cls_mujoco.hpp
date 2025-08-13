#ifndef CLS_MUJOCO_H
#define CLS_MUJOCO_H

#include <GLFW/glfw3.h>
#include <iostream>
#include <mujoco/mujoco.h>
#include <chrono>

class ClsMujoco
{
    public:
        ClsMujoco(const std::string &model_file);
        ~ClsMujoco();
        void main_loop();
        void start_testPositionControl();

    private:
        mjModel *model_ = nullptr;
        mjData  *data_ = nullptr;
        mjvCamera  camera_;
        mjvOption  option_;
        mjvPerturb perturb_;
        mjvScene   scene_;
        mjrContext context_;

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

        int sim_frequency_ = 1;
        int vis_frequency_ = 1;
        void update_visualization();
};

#endif
