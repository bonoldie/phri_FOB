// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/force_example_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace franka_example_controllers {

  // Exponential smoothing version (often preferred)
inline double computeAlpha_Exp(double fc, double fs) {
    return 1.0 - std::exp(-2.0 * M_PI * fc / fs);
}

bool ForceExampleController::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  desiredTrajPub = node_handle.advertise<std_msgs::Float32MultiArray>("desired_trajectory", 1);
  tauExtHatFiltered = node_handle.advertise<std_msgs::Float32MultiArray>("tau_ext_hat_filtered", 1);
  traFrcRef = node_handle.advertise<std_msgs::Float32MultiArray>("tau_frc_ref", 1);

  std::vector<std::string> joint_names;
  std::string arm_id;
  ROS_WARN(
      "ForceExampleController: Make sure your robot's endeffector is in contact "
      "with a horizontal surface before starting the controller!");
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("ForceExampleController: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "ForceExampleController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("ForceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

   // auto *position_joint_interface = robot_hw->get<hardware_interface::PositionJointInterface>();
   //  if (position_joint_interface == nullptr)
   //  {
   //    ROS_ERROR_STREAM(
   //        "FOB_controller: Error getting position joint interface from hardware");
   //    return false;
   //  }
   //  for (size_t i = 0; i < 7; ++i)
   //  {
   //    try
   //    {
   //      position_joint_handles_.push_back(position_joint_interface->getHandle(joint_names[i]));
   //    }
   //    catch (const hardware_interface::HardwareInterfaceException &ex)
   //    {
   //      ROS_ERROR_STREAM(
   //          "FOB_controller: Exception getting joint handles: " << ex.what());
   //      return false;
   //    }
   //  }
    

  dynamic_reconfigure_desired_mass_param_node_ =
      ros::NodeHandle("dynamic_reconfigure_desired_mass_param_node");
  dynamic_server_desired_mass_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_example_controllers::desired_mass_paramConfig>>(

      dynamic_reconfigure_desired_mass_param_node_);
  dynamic_server_desired_mass_param_->setCallback(
      boost::bind(&ForceExampleController::desiredMassParamCallback, this, _1, _2));

  return true;
}

void ForceExampleController::starting(const ros::Time& /*time*/) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  // Bias correction for the current external torque
  tau_ext_initial_ = tau_measured - gravity;
  tau_error_.setZero();

  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial_(robot_state.q.data());
  q_initial = q_initial_;

  motors_inertia << 
        20*0.075,// 100*0.0013, // 7.5e-5,               // J1 (87 Nm)
        20*0.075,// 100*0.0013, // 7.5e-5,               // J2 (87 Nm)
        20*0.075,// 100*0.0013, // 7.5e-5,               // J3 (87 Nm)
        20*0.075,// 100*0.0013, // 7.5e-5,               // J4 (87 Nm)
        3*0.075,// 10*0.0013,// 4.5e-6,               // J5 (12 Nm)
        3*0.075,// 10*0.0013,// 4.5e-6,               // J6 (12 Nm)
        3*0.075;// 10*0.0013;// 4.5e-6;               // J7 (12 Nm)

  Q.setConstant(0);
  // Q(0) = 0;
  // Q(1) = 0;
  // Q(2) = 0;
  // Q(3) = 0;
  // Q(4) = 0;
  // Q(5) = 0;
  // Q(6) = 0;

  f_r.setZero();

  q_error_.setZero();
  tau_frc_hat_prev.setZero();
  ddq_prev.setZero();
  tau_cmd_prev.setZero();
  tau_ext_hat_filtered_prev.setZero();

  alpha_ddq = computeAlpha_Exp(70, 1000);
  alpha_Q_lower = computeAlpha_Exp(0, 1000);
  alpha_Q_upper = computeAlpha_Exp(0, 1000);

  filter_gain_ = computeAlpha_Exp(0.5, 1000);

  alpha_Q(0) = alpha_Q_lower;
  alpha_Q(1) = alpha_Q_lower;
  alpha_Q(2) = alpha_Q_lower;
  alpha_Q(3) = alpha_Q_lower;
  alpha_Q(4) = alpha_Q_upper;
  alpha_Q(5) = alpha_Q_upper;
  alpha_Q(6) = alpha_Q_upper;

  

}

void ForceExampleController::update(const ros::Time &time, const ros::Duration& period) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_ext_hat_filtered(robot_state.tau_ext_hat_filtered.data());

  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());

  Eigen::Matrix<double, 7, 1> tau_d, tau_cmd, tau_ext;
  Eigen::Matrix<double, 6, 1> desired_force_torque;
  
  if (joint_zero_torque) {
    desired_force_torque.setZero();
  } else {
    desired_force_torque(2) = f_Z; //(1 + std::sin(time.now().toNSec() * 1e-9)) * -5;// desired_mass_ * -9.81 ;
  }
  // std::cout << std::sin(time.now().toNSec() * 1e-9) << std::endl;
  tau_ext = tau_measured - gravity - tau_ext_initial_;
  tau_d = jacobian.transpose() * desired_force_torque;

  q_error_ = q_error_ + period.toSec() * (q_initial - q); 
  tau_error_ = tau_error_ + period.toSec() * (tau_d - tau_ext);
  // FF + PI control (PI gains are initially all 0)
  tau_cmd = tau_d + k_p_ * (tau_d - tau_ext) + k_i_ * tau_error_;
  // tau_cmd.setZero();
  // tau_cmd = saturateTorqueRate(tau_cmd, tau_J_d);

  // Euler approx. for acceleration calculation
  // period is fixed at 1ms (1kHz controller loop)
  Eigen::Matrix<double, 7, 1> ddq = (dq - dq_prev) / 0.001f;
  // Removed because we filter with the Q filter 
  ddq = alpha_ddq * ddq + (1 - alpha_ddq) * ddq_prev;
  
  auto tau_m_ref = ddq.cwiseProduct(motors_inertia) + dq.cwiseProduct(f_r);

  // Torque estimation due to friction (low-passed in the next line)
  Eigen::Matrix<double, 7, 1> tau_frc_hat = tau_m_ref - (tau_cmd_prev - tau_ext_hat_filtered_prev);
  tau_frc_hat =  alpha_Q.cwiseProduct(tau_frc_hat) + (Eigen::Matrix<double, 7, 1>::Ones() - alpha_Q).cwiseProduct(tau_frc_hat_prev);

  for (size_t i = 0; i < 7; ++i) {

    //if(i < 6) {
    //  joint_handles_[i].setCommand(300 * (q_initial(i) - q(i)) + 40 * q_error_(i) - 40 * dq(i));
      // position_joint_handles_[i].setCommand(q_initial(i));
    //} else {
      joint_handles_[i].setCommand(tau_cmd(i) - (Q(i) * tau_frc_hat(i)));
    // }
  }

  std_msgs::Float32MultiArray float32MultiArrayMsg;
  float32MultiArrayMsg.data.resize(6);

  for (size_t i = 0; i < 6; ++i)
  {
    float32MultiArrayMsg.data[i] = desired_force_torque(i);
  }
  desiredTrajPub.publish(float32MultiArrayMsg);

  std::copy(tau_frc_hat.data(), tau_frc_hat.data() + 7, float32MultiArrayMsg.data.begin());
  traFrcRef.publish(float32MultiArrayMsg);

  // Update signals changed online through dynamic reconfigure
  desired_mass_ = filter_gain_ * target_mass_ + (1 - filter_gain_) * desired_mass_;
  k_p_ = filter_gain_ * target_k_p_ + (1 - filter_gain_) * k_p_;
  k_i_ = filter_gain_ * target_k_i_ + (1 - filter_gain_) * k_i_;
  f_Z = filter_gain_ * f_Z_ + (1 - filter_gain_) * f_Z;

  tau_frc_hat_prev = tau_frc_hat;
  dq_prev = dq;
  ddq_prev = ddq;
  tau_cmd_prev = tau_cmd;
  tau_ext_hat_filtered_prev = tau_ext_hat_filtered;
}

void ForceExampleController::desiredMassParamCallback(
    franka_example_controllers::desired_mass_paramConfig& config,
    uint32_t /*level*/) {
  target_mass_ = config.desired_mass;
  target_k_p_ = config.k_p;
  target_k_i_ = config.k_i;
  
  Q(0) = config.q_lower;
  Q(1) = config.q_lower;
  Q(2) = config.q_lower;
  Q(3) = config.q_lower;
  Q(4) = config.q_upper;
  Q(5) = config.q_upper;
  Q(6) = config.q_upper;

  alpha_Q(0) = computeAlpha_Exp(config.f_c_lower, 1000);
  alpha_Q(1) = computeAlpha_Exp(config.f_c_lower, 1000);
  alpha_Q(2) = computeAlpha_Exp(config.f_c_lower, 1000);
  alpha_Q(3) = computeAlpha_Exp(config.f_c_lower, 1000);
  alpha_Q(4) = computeAlpha_Exp(config.f_c_upper, 1000);
  alpha_Q(5) = computeAlpha_Exp(config.f_c_upper, 1000);
  alpha_Q(6) = computeAlpha_Exp(config.f_c_upper, 1000);
  
  joint_zero_torque = config.joint_zero_torque;

  f_Z_ = - config.f_z;

  std::cout << "Q: "<< f_Z << '\n';
  std::cout << "Q: "<< Q << '\n';
  std::cout << "alpha_Q :"<< alpha_Q << '\n';
}

Eigen::Matrix<double, 7, 1> ForceExampleController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ForceExampleController,
                       controller_interface::ControllerBase)
