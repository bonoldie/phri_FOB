// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <phri_fob/identification_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <phri_fob/pseudo_inversion.h>

namespace phri_fob
{

  bool Identification_controller::init(hardware_interface::RobotHW *robot_hw,
                            ros::NodeHandle &node_handle)
  {
    std::vector<double> cartesian_stiffness_vector;
    std::vector<double> cartesian_damping_vector;


    desiredTrajPub = node_handle.advertise<std_msgs::Float32MultiArray>("desired_trajectory", 1);

    // sub_equilibrium_pose_ = node_handle.subscribe(
    //     "equilibrium_pose", 20, &Identification_controller::equilibriumPoseCallback, this,
    //     ros::TransportHints().reliable().tcpNoDelay());

    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id))
    {
      ROS_ERROR_STREAM("Identification_controller: Could not read parameter arm_id");
      return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7)
    {
      ROS_ERROR(
          "Identification_controller: Invalid or no joint_names parameters provided, "
          "aborting controller init!");
      return false;
    }

    auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr)
    {
      ROS_ERROR_STREAM(
          "Identification_controller: Error getting model interface from hardware");
      return false;
    }
    try
    {
      model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
          model_interface->getHandle(arm_id + "_model"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM(
          "Identification_controller: Exception getting model handle from interface: "
          << ex.what());
      return false;
    }

    auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR_STREAM(
          "Identification_controller: Error getting state interface from hardware");
      return false;
    }
    try
    {
      state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
          state_interface->getHandle(arm_id + "_robot"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM(
          "Identification_controller: Exception getting state handle from interface: "
          << ex.what());
      return false;
    }

    auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr)
    {
      ROS_ERROR_STREAM(
          "Identification_controller: Error getting effort joint interface from hardware");
      return false;
    }
    for (size_t i = 0; i < 7; ++i)
    {
      try
      {
        joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
      }
      catch (const hardware_interface::HardwareInterfaceException &ex)
      {
        ROS_ERROR_STREAM(
            "Identification_controller: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    auto *position_joint_interface = robot_hw->get<hardware_interface::PositionJointInterface>();
    if (position_joint_interface == nullptr)
    {
      ROS_ERROR_STREAM(
          "Identification_controller: Error getting position joint interface from hardware");
      return false;
    }
    for (size_t i = 0; i < 7; ++i)
    {
      try
      {
        position_joint_handles_.push_back(position_joint_interface->getHandle(joint_names[i]));
      }
      catch (const hardware_interface::HardwareInterfaceException &ex)
      {
        ROS_ERROR_STREAM(
            "Identification_controller: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    // dynamic_reconfigure_compliance_param_node_ = ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");

    // dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<phri_fob::compliance_paramConfig>>(dynamic_reconfigure_compliance_param_node_);
    // dynamic_server_compliance_param_->setCallback(boost::bind(&Identification_controller::complianceParamCallback, this, _1, _2));

    position_d_.setZero();
    orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
    position_d_target_.setZero();
    orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

    cartesian_stiffness_.setZero();
    cartesian_damping_.setZero();

    return true;
  }

  void Identification_controller::starting(const ros::Time & /*time*/)
  {
    KP.setZero();
    KP(6) = 2500.0;
    // KP(5) = 2500.0;
    KP(4) = 2500.0; 
    KP(2) = 2500.0; 

    // KP(5) = 10.0;
    // KP(4) = 10.0;

    KD.setZero();
    KP(6) = 15.0;
    // KP(5) = 15.0;
    KP(4) = 15.0;
    KP(2) = 15.0;

    // 3–6 × 10⁻⁶ kg·m² for the smallest motors
    // 5–10 × 10⁻⁵ kg·m² for the largest motors
    motors_inertia << 
        7.5e-5,  // J1 (87 Nm)
        7.5e-5,  // J2 (87 Nm)
        7.5e-5,  // J3 (87 Nm)
        7.5e-5,  // J4 (87 Nm)
        4.5e-6,  // J5 (12 Nm)
        4.5e-6,  // J6 (12 Nm)
        4.5e-6;  // J7 (12 Nm)
    

    // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
    // to initial configuration
    auto initial_state = state_handle_->getRobotState();
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> prev_joints_dq(initial_state.dq.data());
  
    prev_torque_frc_estimated.setZero();
    prev_joints_ddq.setZero();

    // get jacobian
    // std::array<double, 42> jacobian_array =
    //     model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    // // convert to eigen
    // Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
    // Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    //
    // // set equilibrium point to current state
    // position_d_ = initial_transform.translation();
    // orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
    // position_d_target_ = initial_transform.translation();
    // orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());
    //
    // // set nullspace equilibrium configuration to initial q
    // q_d_nullspace_ = q_initial;
  }


  void Identification_controller::update(const ros::Time &time,
                              const ros::Duration & /*period*/)
  {
    // Joints trajectory setup
    uint64_t nsec_now =  time.now().toNSec();
    auto time_in_sec = static_cast<double>(nsec_now) * 1e-9;
    auto ref_offset = std::sin(time_in_sec)/(2.0 * M_PI); 
    Eigen::Matrix<double, 7, 1> q_ref = q_initial.array() + ref_offset;

    std_msgs::Float32MultiArray msg;
    msg.data.resize(7);

    for (size_t i = 0; i < 7; ++i)
    {
      msg.data[i] = q_ref[i];
    }

    desiredTrajPub.publish(msg);

    // get state variables and gravity vector
    franka::RobotState robot_state = state_handle_->getRobotState();
    Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity_vec(model_handle_->getGravity().data());

    Eigen::Map<Eigen::Matrix<double, 7, 1>> joints_effort(robot_state.tau_J.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> joints_dq(robot_state.dq.data());
    Eigen::Matrix<double, 7, 1> joints_ddq = (joints_dq - prev_joints_dq) / 0.001f;
    joints_ddq = alpha * joints_ddq + (1 - alpha) * prev_joints_ddq;

    auto torque_without_g = joints_effort - gravity_vec;

    auto effective_torque = joints_ddq.cwiseProduct(motors_inertia); 
    
    // // convert to Eigen
    // Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    // Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());


    // Position PD outputs torque commands
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    
    Eigen::Matrix<double, 7, 1> q_error = q_ref - q;
    Eigen::Matrix<double, 7, 1> dq_error =  - dq; 

    // tau_m
    auto torque_command = KP.cwiseProduct(q_error) + KD.cwiseProduct(dq_error);  
    
    auto torque_frc_estimated = alpha * (effective_torque - torque_command) + (1 - alpha) * prev_torque_frc_estimated; 
    
    printf("\33[H\33[2J");
    std::cout <<  (torque_command - torque_frc_estimated) << std::endl;

    for (size_t i = 0; i < 7; ++i) {
      joint_handles_[i].setCommand(torque_command(i) - (torque_frc_estimated(i) * 6));//torque_command(i)); //   
      // joint_handles_[i].setCommand(0.0);
    }

    prev_torque_frc_estimated = torque_frc_estimated;
    prev_joints_dq = joints_dq;
    prev_joints_ddq = joints_ddq;
  }
} // namespace phri_fob

PLUGINLIB_EXPORT_CLASS(phri_fob::Identification_controller, controller_interface::ControllerBase)
