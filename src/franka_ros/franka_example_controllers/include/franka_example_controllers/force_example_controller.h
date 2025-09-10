// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Core>


#include <std_msgs/Float32MultiArray.h>


#include <franka_example_controllers/desired_mass_paramConfig.h>

namespace franka_example_controllers {

class ForceExampleController : public controller_interface::MultiInterfaceController<
                                   franka_hw::FrankaModelInterface,
                                   hardware_interface::EffortJointInterface,
                                   franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

    // publisher nodes
    ros::Publisher desiredTrajPub;
    ros::Publisher tauExtHatFiltered;
    ros::Publisher traFrcRef;


 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  // Low-pass hyperparameter
  double alpha_ddq = 0.99;
  double alpha_Q_lower = 0;
  double alpha_Q_upper = 0;
  double f_Z = 0;
  double f_Z_ = 0;

   Eigen::Matrix<double, 7, 1> alpha_Q;


  // Model reference
  Eigen::Matrix<double, 7, 1> motors_inertia;

  // MR-FOB feedback controller
  Eigen::Matrix<double, 7, 1> Q;

  // MR-FOB friction shaper
  Eigen::Matrix<double, 7, 1> f_r;

  Eigen::Matrix<double, 7, 1> tau_frc_hat_prev;
  Eigen::Matrix<double, 7, 1> dq_prev;
  Eigen::Matrix<double, 7, 1> ddq_prev;
  Eigen::Matrix<double, 7, 1> tau_cmd_prev;

  bool joint_zero_torque{false};
    
  double desired_mass_{0.0};
  double target_mass_{0.0};
  double k_p_{0.0};
  double k_i_{0.0};
  double target_k_p_{0.0};
  double target_k_i_{0.0};
  double filter_gain_;
  Eigen::Matrix<double, 7, 1> tau_ext_initial_;
  Eigen::Matrix<double, 7, 1> tau_error_;
  static constexpr double kDeltaTauMax{1.0};

  // Dynamic reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::desired_mass_paramConfig>>
      dynamic_server_desired_mass_param_;
  ros::NodeHandle dynamic_reconfigure_desired_mass_param_node_;
  void desiredMassParamCallback(franka_example_controllers::desired_mass_paramConfig& config,
                                uint32_t level);
};

}  // namespace franka_example_controllers
