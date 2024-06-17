// C++ basic libraries
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <cstdio>
#include <cstring>
#include <Eigen/Dense>
#include <list>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Mujoco
#include <mujoco/mujoco.h>

// GLFW
#include <GLFW/glfw3.h>

// ROS2
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "rclcpp/qos.hpp"

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <rmw/types.h>
#include <rclcpp/time.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include "thesis_interfaces/srv/start_single_robot.hpp"
#include "thesis_interfaces/srv/stop_single_robot.hpp"
#include "thesis_interfaces/srv/robot_exists_id.hpp"

using namespace std::chrono_literals;

class MujocoGym : public rclcpp::Node
{
public:
  MujocoGym()
      : Node("mujoco_gym")
  {

    // ******************
    // MuJoCo configuration
    // ******************

    // read mujoco model from file and check for errors
    char error[1000] = "Could not load binary model";
    // std::string filename_str = (ament_index_cpp::get_package_share_directory("thesis_robot_cpp") + "/robot/ur5e/joint_vel_scene.xml");
    // std::string filename_str = (ament_index_cpp::get_package_share_directory("thesis_robot_cpp") + "/robot/ur5e/joint_vel_scene_peg_in_hole.xml");
    std::string filename_str = (ament_index_cpp::get_package_share_directory("thesis_robot_cpp") + "/robot/ur5e/joint_vel_scene_gear_assembly.xml");
    const char *filename = filename_str.c_str();

    if (std::strlen(filename) > 4 && !std::strcmp(filename + std::strlen(filename) - 4, ".mjb"))
    {
      m = mj_loadModel(filename, 0);
    }
    else
    {
      m = mj_loadXML(filename, 0, error, 1000);
    }
    if (!m)
    {
      mju_error("Could not load model");
    }

    // make data
    d = mj_makeData(m);
    nq = m->nq;
    nu = m->nu;

    jacp = new mjtNum[3 * nq]; // pointer to position jacobian of tip, 3xnq
    jacr = new mjtNum[3 * nq]; // pointer to rotation jacobian of tip, 3xnq

    // reset key
    mj_resetDataKeyframe(m, d, 0);

    // set and get simulation timestep
    m->opt.timestep = simu_timestep;
    RCLCPP_INFO(this->get_logger(), "Simulation frequency is %d Hz", (int)round((1 / (m->opt.timestep))));

    // get site ids
    sensor_id = mj_name2id(m, mjtObj::mjOBJ_SITE, "ft_sensor");
    tip_id = mj_name2id(m, mjtObj::mjOBJ_SITE, "tip");

    // mathematics initilization
    resetMath();

    // ******************
    // ROS2 configuration
    // ******************

    this->declare_parameter("robot_exists", false);
    this->set_parameter(rclcpp::Parameter("robot_exists", false)); // so there is a flag to reset the force controller

    // configure ROS communication
    static const rmw_qos_profile_t rmw_qos_profile_reliable =
        {
            RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            10,
            RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            RMW_QOS_POLICY_DURABILITY_VOLATILE,
            RMW_QOS_DEADLINE_DEFAULT,
            RMW_QOS_LIFESPAN_DEFAULT,
            RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
            RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
            false};

    auto qos_reliable = rclcpp::QoS(
        rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_reliable),
        rmw_qos_profile_reliable);

    // subscribers
    sub_cmd_cart_vel_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "cmd_cart_vel", 10, std::bind(&MujocoGym::sub_cmd_cart_vel_callback, this, std::placeholders::_1));
    cmd_cart_vel_ = Eigen::VectorXd::Zero(6);

    // publishers and messages
    pub_sensor_ft_ = create_publisher<geometry_msgs::msg::WrenchStamped>("sensor_ft_raw", 10);
    msg_sensor_ft_ = std::make_shared<geometry_msgs::msg::WrenchStamped>();

    pub_act_cart_pos_ = create_publisher<geometry_msgs::msg::PoseStamped>("act_cart_pos", 10);
    msg_act_cart_pos_ = std::make_shared<geometry_msgs::msg::PoseStamped>();

    pub_act_cart_vel_ = create_publisher<geometry_msgs::msg::TwistStamped>("act_cart_vel", 10);
    msg_act_cart_vel_ = std::make_shared<geometry_msgs::msg::TwistStamped>();

    pub_act_joint_state_ = create_publisher<sensor_msgs::msg::JointState>("act_joint_state", 10);
    msg_act_joint_state_ = std::make_shared<sensor_msgs::msg::JointState>();
    msg_act_joint_state_->name = {"joint0", "joint1", "joint2", "joint3", "joint4", "joint5"};

    // timers
    simu_timer_ = this->create_wall_timer(
        std::chrono::nanoseconds(static_cast<int64_t>(m->opt.timestep * 1e9)),
        std::bind(&MujocoGym::simulation_manager, this));

    // services
    srv_start_single_robot_ = create_service<thesis_interfaces::srv::StartSingleRobot>("mujoco_gym/start_single_robot", std::bind(&MujocoGym::start_single_robot, this, std::placeholders::_1, std::placeholders::_2));
    srv_stop_single_robot_ = create_service<thesis_interfaces::srv::StopSingleRobot>("mujoco_gym/stop_single_robot", std::bind(&MujocoGym::stop_single_robot, this, std::placeholders::_1, std::placeholders::_2));
    srv_robot_exists_id_ = create_service<thesis_interfaces::srv::RobotExistsID>("mujoco_gym/robot_exists_id", std::bind(&MujocoGym::robot_exists_id, this, std::placeholders::_1, std::placeholders::_2));

    // buffers for rendering thread
    m_render = mj_copyModel(m_render, m);
    d_render = mj_copyData(d_render, m, d);

    RCLCPP_INFO(this->get_logger(), "MuJoCo Gym has started");
  }

  ~MujocoGym()
  {

    glfwDestroyWindow(window);
    windowExists = false;

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data
    mj_deleteData(d);
    mj_deleteModel(m);

    // deallocate pointer memory
    delete[] jacp;
    delete[] jacr;
    delete[] tip_quat;

    RCLCPP_INFO(this->get_logger(), "MuJoCo Gym has finished");
  }

private:
  void resetMath()
  {
    cmd_cart_vel_.setZero();
    jac.setZero();
    sensor_ft.setZero();
  }

  void resetAll()
  {
    std::unique_lock<std::mutex> lock(render_mutex);
    mj_resetData(m, d);
    resetMath();
    mj_forward(m, d);
    mj_resetDataKeyframe(m, d, 0);
    lock.unlock();
  }

  // GLFW window handling functions
  void keyboard_imp(int key, int act)
  {
    // RCLCPP_INFO(this->get_logger(), "keypressed code is: %d", key);
    if (act == GLFW_PRESS && key == KEYCODE)
    {

      this->set_parameter(rclcpp::Parameter("robot_exists", false));
      robotExists = false;
      resetAll();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      robotExists = true;
      this->set_parameter(rclcpp::Parameter("robot_exists", true));

      RCLCPP_INFO(this->get_logger(), "Simulation has been manually restarted");
    }
  }

  void mouse_button_imp(GLFWwindow *windowCallback)
  {
    // update button state
    button_left = (glfwGetMouseButton(windowCallback, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(windowCallback, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(windowCallback, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(windowCallback, &lastx, &lasty);
  }

  void mouse_move_imp(GLFWwindow *windowCallback, double xpos, double ypos)
  {
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
    {
      return;
    }

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(windowCallback, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(windowCallback, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(windowCallback, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
    {
      action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    }
    else if (button_left)
    {
      action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    }
    else
    {
      action = mjMOUSE_ZOOM;
    }

    // move camera
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
  }

  void scroll_imp(double yoffset)
  {
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
  }

  static void keyboard(GLFWwindow *windowCallback, int scancode, int key, int act, int mods)
  {
    MujocoGym *obj = static_cast<MujocoGym *>(glfwGetWindowUserPointer(windowCallback));
    obj->keyboard_imp(key, act);
    (void)scancode;
    (void)mods;
  }

  static void mouse_button(GLFWwindow *windowCallback, int button, int act, int mods)
  {
    MujocoGym *obj = static_cast<MujocoGym *>(glfwGetWindowUserPointer(windowCallback));
    obj->mouse_button_imp(windowCallback);
    (void)button;
    (void)act;
    (void)mods;
  }

  static void mouse_move(GLFWwindow *windowCallback, double xpos, double ypos)
  {
    MujocoGym *obj = static_cast<MujocoGym *>(glfwGetWindowUserPointer(windowCallback));
    obj->mouse_move_imp(windowCallback, xpos, ypos);
  }

  static void scroll(GLFWwindow *windowCallback, double xoffset, double yoffset)
  {
    MujocoGym *obj = static_cast<MujocoGym *>(glfwGetWindowUserPointer(windowCallback));
    obj->scroll_imp(yoffset);
    (void)xoffset;
  }

  // ROS2 topic callbacks
  void sub_cmd_cart_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    // RCLCPP_INFO(get_logger(), "Updating velocity command");
    cmd_cart_vel_(0) = static_cast<double>(msg->twist.linear.x);
    cmd_cart_vel_(1) = static_cast<double>(msg->twist.linear.y);
    cmd_cart_vel_(2) = static_cast<double>(msg->twist.linear.z);
    cmd_cart_vel_(3) = static_cast<double>(msg->twist.angular.x);
    cmd_cart_vel_(4) = static_cast<double>(msg->twist.angular.y);
    cmd_cart_vel_(5) = static_cast<double>(msg->twist.angular.z);
  }

  // Simulation
  void simulation_manager()
  {
    if (robotExists)
    {
      simu_step();
    }
  }

  void simu_step()
  {
    // RCLCPP_INFO(this->get_logger(), "Simulating");

    // Get the jacobian (nq = 6)
    // mj_jacSite(m, d, jacp, jacr, tip_id);
    // Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>> temp1(jacp);
    // Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>> temp2(jacr);
    // jac.block<3,6>(0,0) = temp1;
    // jac.block<3,6>(3,0) = temp2;

    // Get the jacobian (stored in serialized form, pointers jacp and jacr to the memory address where it is stored)
    mj_jacSite(m, d, jacp, jacr, tip_id);

    
    if (nq == 6)
    {
      // Deserialize the information and cast into a matrix with the right size
      Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>> temp1(jacp);
      Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>> temp2(jacr);
      // Stack the jacobian parts relative to the robot joints (6 first rows), this block must be inside the if becaue tempx is available only in this scope
      jac.block<3, 6>(0, 0) = temp1.block<3, 6>(0, 0);
      jac.block<3, 6>(3, 0) = temp2.block<3, 6>(0, 0);
    }
    else if (nq == 9)
    {
      // Deserialize the information and cast into a matrix with the right size
      Eigen::Map<Eigen::Matrix<double, 3, 9, Eigen::RowMajor>> temp1(jacp);
      Eigen::Map<Eigen::Matrix<double, 3, 9, Eigen::RowMajor>> temp2(jacr);
      // Stack the jacobian parts relative to the robot joints (6 first rows), this block must be inside the if becaue tempx is available only in this scope
      jac.block<3, 6>(0, 0) = temp1.block<3, 6>(0, 0);
      jac.block<3, 6>(3, 0) = temp2.block<3, 6>(0, 0);
    }

    // Compute inverse kinematics
    cmd_joint_vel_ = jac.completeOrthogonalDecomposition().pseudoInverse() * cmd_cart_vel_;

    // Update the command joint velocity
    for (int i = 0; i < nu; ++i)
    {
      d->ctrl[i] = cmd_joint_vel_(i);
    }

    // Forward the simulation one step (data is modified, hold the mutex)
    std::unique_lock<std::mutex> lock(render_mutex);
    mj_step(m, d);
    lock.unlock();

    // Get the rotation matrix of sensor site
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        sensor_mat(i, j) = d->site_xmat[sensor_id * 9 + i * 3 + j];
      }
    }

    // Rotate the signals from the ft sensor and express them in the fixed frame
    Eigen::Vector3d f_og_(d->sensordata[0], d->sensordata[1], d->sensordata[2]);
    Eigen::Vector3d t_og_(d->sensordata[3], d->sensordata[4], d->sensordata[5]);
    sensor_ft.block<3, 1>(0, 0) = sensor_mat * f_og_;
    sensor_ft.block<3, 1>(3, 0) = sensor_mat * t_og_;

    // std::cout <<  sensor_ft(2,0) << std::endl;

    // ROS2 publiser
    act_time_ = get_clock()->now();

    // Force
    msg_sensor_ft_->header.stamp = act_time_;
    msg_sensor_ft_->wrench.force.x = sensor_ft(0, 0);
    msg_sensor_ft_->wrench.force.y = sensor_ft(1, 0);
    msg_sensor_ft_->wrench.force.z = sensor_ft(2, 0);
    msg_sensor_ft_->wrench.torque.x = sensor_ft(3, 0);
    msg_sensor_ft_->wrench.torque.y = sensor_ft(4, 0);
    msg_sensor_ft_->wrench.torque.z = sensor_ft(5, 0);
    pub_sensor_ft_->publish(*msg_sensor_ft_);

    // Joint state
    msg_act_joint_state_->header.stamp = act_time_;
    msg_act_joint_state_->position = std::vector<double>(d->qpos, d->qpos + 6);
    msg_act_joint_state_->velocity = std::vector<double>(d->qvel, d->qvel + 6);
    msg_act_joint_state_->effort = std::vector<double>(d->qfrc_applied, d->qfrc_applied + 6);
    pub_act_joint_state_->publish(*msg_act_joint_state_);

    // Tip pose
    mju_mat2Quat(tip_quat, d->site_xmat + 9 * tip_id); // pointers to the beginning of the relevant information
    msg_act_cart_pos_->header.stamp = act_time_;
    msg_act_cart_pos_->pose.position.x = (d->site_xpos[3 * tip_id + 0]);
    msg_act_cart_pos_->pose.position.y = (d->site_xpos[3 * tip_id + 1]);
    msg_act_cart_pos_->pose.position.z = (d->site_xpos[3 * tip_id + 2]);
    msg_act_cart_pos_->pose.orientation.w = tip_quat[0];
    msg_act_cart_pos_->pose.orientation.x = tip_quat[1];
    msg_act_cart_pos_->pose.orientation.y = tip_quat[2];
    msg_act_cart_pos_->pose.orientation.z = tip_quat[3];
    pub_act_cart_pos_->publish(*msg_act_cart_pos_);

    // Tip velocity
    Eigen::Map<Eigen::VectorXd> temp3(d->qvel, 6);
    act_cart_vel_ = jac * temp3;
    msg_act_cart_vel_->header.stamp = act_time_;
    msg_act_cart_vel_->twist.linear.x = act_cart_vel_[0];
    msg_act_cart_vel_->twist.linear.y = act_cart_vel_[1];
    msg_act_cart_vel_->twist.linear.z = act_cart_vel_[2];
    msg_act_cart_vel_->twist.angular.x = act_cart_vel_[3];
    msg_act_cart_vel_->twist.angular.y = act_cart_vel_[4];
    msg_act_cart_vel_->twist.angular.z = act_cart_vel_[5];
    pub_act_cart_vel_->publish(*msg_act_cart_vel_);
  }

  // Rendering
  void start_render_thread()
  {
    render_thread_ = std::thread([this]()
                                 {

        // ******************
        // GLFW configuration
        // ******************

        RCLCPP_INFO(this->get_logger(), "Rendering frequency is %d Hz", (int)round((1000/(render_timestep))));

        // init GLFW
        if (!glfwInit()) {
          mju_error("Could not initialize GLFW");
        }

        // create window, make OpenGL context current, request v-sync
        // GLFWwindow* 
        window = glfwCreateWindow(1200, 900, "MuJoCo Gym", NULL, NULL);
        windowExists = true;
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);

        // initialize visualization data structures
        mjv_defaultCamera(&cam);
        mjv_defaultOption(&opt);
        mjv_defaultScene(&scn);
        mjr_defaultContext(&con);

        // create scene and context
        mjv_makeScene(m, &scn, 2000);
        mjr_makeContext(m, &con, mjFONTSCALE_150);

        // Orientate camera
        cam.distance = 1.2;
        cam.azimuth = 130;
        cam.elevation = -20;
        cam.lookat[0] = 0.7;
        cam.lookat[1] = -0.4;
        cam.lookat[2] = 0.5;

        // install GLFW mouse and keyboard callbacks
        glfwSetWindowUserPointer(window, this);
        glfwSetKeyCallback(window, keyboard);
        glfwSetCursorPosCallback(window, mouse_move);
        glfwSetMouseButtonCallback(window, mouse_button);
        glfwSetScrollCallback(window, scroll);

        while (windowExists) {
          if(!glfwWindowShouldClose(window)){
            this->render_step();
            std::this_thread::sleep_for(std::chrono::milliseconds(render_timestep));
          }
          else{
            glfwDestroyWindow(window);
            RCLCPP_INFO(this->get_logger(), "Rendering window closed, but simulation still continues in the background");
            windowExists = false;
          }    
        }
        RCLCPP_INFO(this->get_logger(), "Rendering thread is exited"); });

    render_thread_.detach();
  }

  void render_step()
  {
    // RCLCPP_INFO(this->get_logger(), "Rendering");

    // copy data from sim to render buffer
    std::unique_lock<std::mutex> lock(render_mutex);
    d_render = mj_copyData(d_render, m, d);
    lock.unlock();

    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m_render, d_render, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
  }

  // ROS2 Services
  void start_single_robot(const std::shared_ptr<thesis_interfaces::srv::StartSingleRobot::Request> request, std::shared_ptr<thesis_interfaces::srv::StartSingleRobot::Response> response)
  {
    if (robotExists)
    {
      RCLCPP_WARN(this->get_logger(), "Robot with id %i already exists, please terminate it before starting a new one", robotID);
      response->spawned = false;
    }
    else
    {
      resetAll();
      robotExists = true;
      this->set_parameter(rclcpp::Parameter("robot_exists", true));
      robotID = request->robot_id;
      addID(request->robot_id);
      if (request->enable_visu)
      {
        this->start_render_thread();
      }
      response->spawned = true;
    }
  }

  void stop_single_robot(const std::shared_ptr<thesis_interfaces::srv::StopSingleRobot::Request> request, std::shared_ptr<thesis_interfaces::srv::StopSingleRobot::Response> response)
  {
    if (robotExists)
    {
      if (robotID == request->robot_id)
      {
        if (windowExists)
        {
          glfwDestroyWindow(window);
          windowExists = false;
        }
        this->set_parameter(rclcpp::Parameter("robot_exists", false));
        robotExists = false;
        response->killed = true;
        removeID(request->robot_id);
      }
      else
      {
        response->killed = false;
        RCLCPP_WARN(this->get_logger(), "Trying to kill a robot with id %i, but current alive robot has id %i", request->robot_id, robotID);
      }
    }
    else
    {
      response->killed = false;
      RCLCPP_WARN(this->get_logger(), "No robot is alive");
    }
  }

  void robot_exists_id(const std::shared_ptr<thesis_interfaces::srv::RobotExistsID::Request> request, std::shared_ptr<thesis_interfaces::srv::RobotExistsID::Response> response)
  {
    response->exists = robotExistsID(request->robot_id);
  }

  void addID(int ID)
  {
    if (robotExistsID(ID))
    {
      RCLCPP_INFO(this->get_logger(), "Robot with id %i already exists", ID);
    }
    robotsAlive.push_back(ID);
    RCLCPP_INFO(this->get_logger(), "Spawning a robot with id %i", ID);
  }

  void removeID(int ID)
  {
    if (robotExistsID(ID))
    {
      for (auto i = robotsAlive.begin(); i != robotsAlive.end(); ++i)
      {
        if (*i == ID)
        {
          robotsAlive.erase(i);
          RCLCPP_INFO(this->get_logger(), "Killing the robot with id %i", ID);
          return; // stop first ocurrence
        }
      }
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Robot with id %i does no exist", ID);
      return;
    }
  }

  bool robotExistsID(int ID)
  {
    RCLCPP_INFO(this->get_logger(), "Checking if robot with id %i exists ...", ID);

    for (int i : robotsAlive)
    {
      if (i == ID)
      {
        RCLCPP_INFO(this->get_logger(), "Robot with id %i is already alive", ID);
        return true;
      }
    }
    RCLCPP_INFO(this->get_logger(), "Robot with id %i is not alive yet", ID);
    return false;
  }

  void printIDs()
  {
    std::stringstream ss;
    for (int num : robotsAlive)
    {
      ss << num << " ";
    }
    RCLCPP_INFO(rclcpp::get_logger("print_list"), "The IDs of the currently alive robots are: %s", ss.str().c_str());
  }

  // ******************
  // MuJoCo variables
  // ******************
  float simu_timestep = 0.002; // in seconds, 500 Hz
  bool robotExists = false;    // initialzed in constructor
  int robotID = -1;

  mjModel *m = NULL;        // MuJoCo model for simulation
  mjData *d = NULL;         // MuJoCo data for simulation
  mjModel *m_render = NULL; // MuJoCo model for rendering
  mjData *d_render = NULL;  // MuJoCo data for rendering
  mjvCamera cam;            // abstract camera, main
  mjvOption opt;            // visualization options
  mjvScene scn;             // abstract scene
  mjrContext con;           // custom GPU context
  int nq;                   // number of joints
  int nu;                   // number of actuators

  // mathematics
  int sensor_id; // ft sensor site id
  int tip_id;    // tip site id

  Eigen::Matrix<mjtNum, 6, 6> jac; // stacked jacobian of tip, 6x6
  mjtNum *jacp;                    // pointer to position jacobian of tip, 3xnq (initialized in constructor)
  mjtNum *jacr;                    // pointer to rotation jacobian of tip, 3xnq (initialized in constructor)

  Eigen::VectorXd cmd_cart_vel_;  // command cartesian velocity, 3x1 position, 3x1 rotation, 6x total
  Eigen::VectorXd cmd_joint_vel_; // commanded joint velocity, 6x1
  Eigen::VectorXd act_cart_vel_;  // actual cartesian velocity, 3x1 position, 3x1 rotation, 6x1 total

  Eigen::Matrix<mjtNum, 3, 3> sensor_mat; // rotation matrix of the sensor site
  Eigen::Matrix<mjtNum, 6, 1> sensor_ft;  // force torque sensor signal in fixed frame

  mjtNum *tip_quat = new mjtNum[4]; // pointer to the quaternion of the tip frame

  // gym status
  std::list<int> robotsAlive; // list containing the id of the robos that are currently alive

  // ******************
  // GLFW variables
  // ******************
  int render_timestep = 40; // in miliseconds, 25 Hz

  GLFWwindow *window;         // initialized in the constructor
  int KEYCODE = 22;           // to restart the simulation
  bool windowExists = false;  // flag that indicates that a glfw window exists
  bool button_left = false;   // flag capturing mouse left key
  bool button_middle = false; // flag capturing mouse middle key
  bool button_right = false;  // flag capturing mouse right key
  double lastx = 0;           // mouse x position
  double lasty = 0;           // mouse y position

  std::mutex render_mutex;    // mutex for accesing model and data variables in write/read operations
  std::thread render_thread_; // thread for rendering

  // ******************
  // ROS2 variables
  // ******************

  // subscribers
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cmd_cart_vel_;

  // publishers and messages
  rclcpp::Time act_time_;

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_sensor_ft_;
  std::shared_ptr<geometry_msgs::msg::WrenchStamped> msg_sensor_ft_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_act_cart_pos_;
  std::shared_ptr<geometry_msgs::msg::PoseStamped> msg_act_cart_pos_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_act_cart_vel_;
  std::shared_ptr<geometry_msgs::msg::TwistStamped> msg_act_cart_vel_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_act_joint_state_;
  std::shared_ptr<sensor_msgs::msg::JointState> msg_act_joint_state_;

  // timers
  rclcpp::TimerBase::SharedPtr simu_timer_;

  // services
  rclcpp::Service<thesis_interfaces::srv::StartSingleRobot>::SharedPtr srv_start_single_robot_;
  rclcpp::Service<thesis_interfaces::srv::StopSingleRobot>::SharedPtr srv_stop_single_robot_;
  rclcpp::Service<thesis_interfaces::srv::RobotExistsID>::SharedPtr srv_robot_exists_id_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<MujocoGym> node = std::make_shared<MujocoGym>();
  // rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}