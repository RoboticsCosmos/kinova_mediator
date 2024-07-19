/*
Author(s): Djordje Vukcevic, Sven Schneider
Institute: Hochschule Bonn-Rhein-Sieg
Description: Mediator component for enabling conversion of data types.

Copyright (c) [2020]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef KINOVA_MEDIATOR_HPP
#define KINOVA_MEDIATOR_HPP
#include <ActuatorConfigClientRpc.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <KDetailedException.h>
#include <RouterClient.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>
#include <google/protobuf/util/json_util.h>
#include <math.h>
#include <stdlib.h> /* abs */
#include <time.h>
#include <unistd.h>

#include <Eigen/Dense>  // Eigen
#include <boost/assign/list_of.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>  // std::this_thread::sleep_for
#include <utility>
#include <vector>

#include "kdl/jntarray.hpp"

// define constants
#define PI 3.14159265358979323846

enum kinova_model
{
  URDF = 0,
  CUSTOM = 1
};

enum kinova_environment
{
  REAL = 0,
  SIMULATION = 1
};

enum control_mode
{
  STOP_MOTION = -1,
  POSITION = 0,
  VELOCITY = 1,
  TORQUE = 2
};

namespace kinova_constants
{
int NUMBER_OF_JOINTS = 7;
std::vector<double> joint_position_limits_max;
std::vector<double> joint_position_limits_min;
std::vector<double> joint_position_thresholds;
std::vector<double> joint_velocity_limits;
std::vector<double> joint_acceleration_limits;
std::vector<double> joint_torque_limits;
std::vector<double> joint_inertia;
std::vector<double> joint_stopping_torque_limits;
std::vector<double> joint_offsets;
std::vector<double> joint_sim_inertia;
std::vector<double> root_acceleration_1;
std::vector<double> root_acceleration_2;

};  // namespace kinova_constants

enum robot_id
{
  KINOVA_GEN3_1 = 0,
  KINOVA_GEN3_2 = 1
};

class kinova_mediator
{
public:
  kinova_mediator();
  ~kinova_mediator();

  // Initializes variables and calibrates the manipulator
  virtual void initialize(const int robot_environment, const int id, const double DT_SEC);

  virtual bool is_initialized();

  // De-initializes variables and closes the sessions
  void deinitialize();

  // Refreshes feedback from the robot
  void refresh_feedback();

  // Update joint space state: measured positions, velocities and torques
  virtual void get_joint_state(KDL::JntArray &joint_positions, KDL::JntArray &joint_velocities,
                               KDL::JntArray &joint_torques);

  // Update robot state: measured positions, velocities, torques and measured / estimated
  // external forces on end-effector
  virtual void get_robot_state(KDL::JntArray &joint_positions, KDL::JntArray &joint_velocities,
                               KDL::JntArray &joint_torques, KDL::Wrench &end_effector_wrench);

  // Set desired joint commands to move robot and save them for sake of simulation
  virtual int set_joint_command(const KDL::JntArray &joint_positions,
                                const KDL::JntArray &joint_velocities,
                                const KDL::JntArray &joint_torques,
                                const int desired_control_mode);

  // Set joint position command
  virtual int set_joint_positions(const KDL::JntArray &joint_positions);
  // Set joint velocity command
  virtual int set_joint_velocities(const KDL::JntArray &joint_velocities);
  // Set joint torque command
  virtual int set_joint_torques(const KDL::JntArray &joint_torques);
  // Set Zero Joint Velocities and wait until robot has stopped completely
  virtual int stop_robot_motion();
  // Set desired control mode for robot actuators (position/velocity/torque)
  int set_control_mode(const int desired_control_mode, double *joint_torques_sp);

  virtual std::vector<double> get_maximum_joint_pos_limits();
  virtual std::vector<double> get_minimum_joint_pos_limits();
  virtual std::vector<double> get_joint_position_thresholds();
  virtual std::vector<double> get_joint_velocity_limits();
  virtual std::vector<double> get_joint_acceleration_limits();
  virtual std::vector<double> get_joint_torque_limits();
  virtual std::vector<double> get_joint_stopping_torque_limits();
  virtual std::vector<double> get_joint_inertia();
  virtual std::vector<double> get_joint_offsets();
  virtual int get_robot_ID();
  virtual int get_robot_environment();

  virtual KDL::Twist get_root_acceleration();

  float DEG_TO_RAD(float deg);
  float RAD_TO_DEG(float deg);

public:
  bool is_initialized_;
  int kinova_id;
  int kinova_environment_;
  int control_mode_;
  bool add_offsets_;
  bool connection_established_;
  double DT_SEC_;

  KDL::JntArray joint_inertia_sim_;

  // Handles for the kinova manipulator and kdl urdf parsel
  // Create API objects
  std::shared_ptr<Kinova::Api::TransportClientTcp> transport_;
  std::shared_ptr<Kinova::Api::TransportClientUdp> transport_real_time_;
  std::shared_ptr<Kinova::Api::RouterClient> router_;
  std::shared_ptr<Kinova::Api::RouterClient> router_real_time_;
  std::shared_ptr<Kinova::Api::SessionManager> session_manager_;
  std::shared_ptr<Kinova::Api::SessionManager> session_manager_real_time_;
  std::shared_ptr<Kinova::Api::Base::BaseClient> base_;
  std::shared_ptr<Kinova::Api::BaseCyclic::BaseCyclicClient> base_cyclic_;
  std::shared_ptr<Kinova::Api::ActuatorConfig::ActuatorConfigClient> actuator_config_;

  // Joint Measured State Variables
  Kinova::Api::BaseCyclic::Feedback base_feedback_;

  // Joint Setpoint Variables
  Kinova::Api::BaseCyclic::Command base_command_;

  // Variable for alternating servoing mode (High or Low level control)
  Kinova::Api::Base::ServoingModeInformation servoing_mode_;
  Kinova::Api::ActuatorConfig::ControlModeInformation control_mode_message_;

  // Get current joint positions
  virtual void get_joint_positions(KDL::JntArray &joint_positions);
  // Get current joint velocities
  virtual void get_joint_velocities(KDL::JntArray &joint_velocities);
  // Get current joint torques
  virtual void get_joint_torques(KDL::JntArray &joint_torques);
  // Get measured / estimated external forces acting on the end-effector
  virtual void get_end_effector_wrench(KDL::Wrench &end_effector_wrench);

  // get arm voltage
  virtual void get_arm_voltage(double &voltage);
  // get arm current
  virtual void get_arm_current(double &current);
  

  // Increses index of the command's frame id (buffer)
  void increment_command_id();

  // void error_callback_(Kinova::Api::KError err);

  bool get_bit(unsigned int flag, const int position);
  bool robot_stopped();
};
#endif /* KINOVA_MEDIATOR_HPP */
