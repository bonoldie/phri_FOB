// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <phri_fob/fob_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <phri_fob/pseudo_inversion.h>

namespace phri_fob
{

  bool FOB_controller::init(hardware_interface::RobotHW *robot_hw,
                            ros::NodeHandle &node_handle)
  {
    // std::vector<double> cartesian_stiffness_vector;
    // std::vector<double> cartesian_damping_vector;

    // register publishers
    
    desiredTrajPub = node_handle.advertise<std_msgs::Float32MultiArray>("desired_trajectory", 1);
    tauExtHatFiltered = node_handle.advertise<std_msgs::Float32MultiArray>("tau_ext_hat_filtered", 1);
    tauFrcHat = node_handle.advertise<std_msgs::Float32MultiArray>("tau_frc_ref", 1);

    // sub_equilibrium_pose_ = node_handle.subscribe(
    //     "equilibrium_pose", 20, &FOB_controller::equilibriumPoseCallback, this,
    //     ros::TransportHints().reliable().tcpNoDelay());

    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id))
    {
      ROS_ERROR_STREAM("FOB_controller: Could not read parameter arm_id");
      return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7)
    {
      ROS_ERROR(
          "FOB_controller: Invalid or no joint_names parameters provided, "
          "aborting controller init!");
      return false;
    }

    auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr)
    {
      ROS_ERROR_STREAM(
          "FOB_controller: Error getting model interface from hardware");
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
          "FOB_controller: Exception getting model handle from interface: "
          << ex.what());
      return false;
    }

    auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR_STREAM(
          "FOB_controller: Error getting state interface from hardware");
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
          "FOB_controller: Exception getting state handle from interface: "
          << ex.what());
      return false;
    }

    auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr)
    {
      ROS_ERROR_STREAM(
          "FOB_controller: Error getting effort joint interface from hardware");
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
            "FOB_controller: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    auto *position_joint_interface = robot_hw->get<hardware_interface::PositionJointInterface>();
    if (position_joint_interface == nullptr)
    {
      ROS_ERROR_STREAM(
          "FOB_controller: Error getting position joint interface from hardware");
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
            "FOB_controller: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    // Dynamic config server
    // dynamic_reconfigure_compliance_param_node_ = ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");
    // dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<phri_fob::compliance_paramConfig>>(dynamic_reconfigure_compliance_param_node_);
    // dynamic_server_compliance_param_->setCallback(boost::bind(&FOB_controller::complianceParamCallback, this, _1, _2));

    // Initialization of class variables
    position_d_.setZero();
    orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
    position_d_target_.setZero();
    orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

    cartesian_stiffness_.setZero();
    cartesian_damping_.setZero();

    return true;
  }

  void FOB_controller::starting(const ros::Time & /*time*/)
  {
    // Position PD control parameters

    KP.setConstant(20.0);
    // KP(6) = 10.0;
    // KP(5) = 10.0;
    // KP(2) = 10.0;

    KD.setConstant(3.0);
    // KD(6) = 1.0;
    // KD(4) = 1.0;
    // KD(2) = 1.0;

    // Here we set the estimated motors MoI

    // 3–6 × 10⁻⁶ kg·m² for the smallest motors (joint 5,6,7)
    // 5–10 × 10⁻⁵ kg·m² for the largest motors (joint 1,2,3,4)
    motors_inertia << 
        7.5e-5,               // J1 (87 Nm)
        7.5e-5,               // J2 (87 Nm)
        7.5e-5,               // J3 (87 Nm)
        7.5e-5,               // J4 (87 Nm)
        4.5e-6,               // J5 (12 Nm)
        4.5e-6,               // J6 (12 Nm)
        4.5e-6;               // J7 (12 Nm)

    
    // motors_inertia =  motors_inertia.array();

     
    // Initial configuration
    auto initial_state = state_handle_->getRobotState();
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_prev(initial_state.dq.data());

    // Previous values set to zero 
    tau_frc_hat_prev.setZero();
    ddq_prev.setZero();

    // MR-FOB setup
    Q.setConstant(0.0);
    f_r.setZero();

    // MR-FOB controller
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

  void FOB_controller::update(const ros::Time &time,
                              const ros::Duration & /*period*/)
  {

    // Joints trajectory setup
    uint64_t nsec_now = time.now().toNSec();
    auto time_in_sec = static_cast<double>(nsec_now) * 1e-9;
    auto ref_offset = std::sin(time_in_sec * trajectory_freq) * trajectory_scale;
    
    // position reference to track at joint-space level
    // we want to track a sinusoidal trajectory centered in the initial joints configuration
    Eigen::Matrix<double, 7, 1> q_ref = q_initial.array() + ref_offset;
    
    // velocity reference to track at joint-space level
    Eigen::Matrix<double, 7, 1> dq_ref;
    dq_ref.setZero();
    dq_ref = dq_ref.array() + trajectory_freq * std::cos(time_in_sec * trajectory_freq) * trajectory_scale;

    // We do position tracking of the sinusoid on joints 7,5 and 3 only
    // q_ref(0) = q_initial(0);
    // q_ref(1) = q_initial(1);
    // q_ref(3) = q_initial(3);
    // q_ref(5) = q_initial(5);

    // dq_ref(0) = 0.0;
    // dq_ref(1) = 0.0;
    // dq_ref(3) = 0.0;
    // dq_ref(5) = 0.0;
    

    // get state variables and gravity vector
    franka::RobotState robot_state = state_handle_->getRobotState();
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_gravity(model_handle_->getGravity().data());

    // We know from the frankarobotics docs that the manipulator adds the friction & gravity compensation at driver level 
    // tau_sensored = tau_command + tau_friction + tau_gravity + tau_environment 
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_ext_hat_filtered(robot_state.tau_ext_hat_filtered.data());
    
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.dq.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

    // Euler approx. for acceleration calculation
    // period is fixed at 1ms (1kHz controller loop)
    Eigen::Matrix<double, 7, 1> ddq = (dq - dq_prev) / 0.001f;
    ddq = alpha * ddq + (1 - alpha) * ddq_prev;

    Eigen::Matrix<double, 7, 1> q_error = q_ref - q;
    Eigen::Matrix<double, 7, 1> dq_error = dq_ref - dq;

    // tau_command (i.e. tau_m)
    Eigen::Matrix<double, 7, 1> tau_m = KP.cwiseProduct(q_error) + KD.cwiseProduct(dq_error);

    // Here we compute the torque of the motors following the model reference
    // then we add it to the friction shaper term
    auto tau_m_ref = ddq.cwiseProduct(motors_inertia) + dq.cwiseProduct(f_r);

    // Torque estimation due to friction (low-passed in the next line)
    Eigen::Matrix<double, 7, 1> tau_frc_hat = tau_m_ref - (tau_m - tau_ext_hat_filtered);
    tau_frc_hat =  alpha * (tau_frc_hat) + (1 - alpha) * tau_frc_hat_prev;

    printf("\33[H\33[2J");
    std::cout << q_error << std::endl;

    for (size_t i = 0; i < 7; ++i)
    {
      joint_handles_[i].setCommand(tau_m(i) - (Q(i) * tau_frc_hat(i)));
      // joint_handles_[i].setCommand(0.0);
    }

    // Publish desired trajectory
    // message is reused for each topic
    std_msgs::Float32MultiArray float32MultiArrayMsg;
    float32MultiArrayMsg.data.resize(7);

    for (size_t i = 0; i < 7; ++i)
    {
      float32MultiArrayMsg.data[i] = q_ref(i);
    }
    desiredTrajPub.publish(float32MultiArrayMsg);

    std::copy(tau_frc_hat.data(), tau_frc_hat.data() + 7, float32MultiArrayMsg.data.begin());
    tauFrcHat.publish(float32MultiArrayMsg);

    std::copy(tau_ext_hat_filtered.data(), tau_ext_hat_filtered.data() + 7, float32MultiArrayMsg.data.begin());
    tauExtHatFiltered.publish(float32MultiArrayMsg);

    tau_frc_hat_prev = tau_frc_hat;
    dq_prev = dq;
    ddq_prev = ddq;

    // // convert to Eigen
    // Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    // Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

    // Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d( // NOLINT (readability-identifier-naming)
    //     robot_state.tau_J_d.data());
    // Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    // Eigen::Vector3d position(transform.translation());
    // Eigen::Quaterniond orientation(transform.linear());

    // // compute error to desired pose
    // // position error
    // Eigen::Matrix<double, 6, 1> error;
    // error.head(3) << position - position_d_;

    // // orientation error
    // if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0)
    // {
    //   orientation.coeffs() << -orientation.coeffs();
    // }
    // // "difference" quaternion
    // Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
    // error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    // // Transform to base frame
    // error.tail(3) << -transform.linear() * error.tail(3);

    // // compute control
    // // allocate variables
    // Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

    // // pseudoinverse for nullspace handling
    // // kinematic pseuoinverse
    // Eigen::MatrixXd jacobian_transpose_pinv;
    // pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

    // // Cartesian PD control with damping ratio = 1
    // tau_task << jacobian.transpose() *
    //                 (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
    // // nullspace PD control with damping ratio = 1
    // tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
    //                   jacobian.transpose() * jacobian_transpose_pinv) *
    //                      (nullspace_stiffness_ * (q_d_nullspace_ - q) -
    //                       (2.0 * sqrt(nullspace_stiffness_)) * dq);
    // // Desired torque
    // tau_d << tau_task + tau_nullspace + coriolis;
    // // Saturate torque rate to avoid discontinuities
    // tau_d << saturateTorqueRate(tau_d, tau_J_d);
    // for (size_t i = 0; i < 7; ++i)
    // {
    //   joint_handles_[i].setCommand(tau_d(i));
    // }

    // // update parameters changed online either through dynamic reconfigure or through the interactive
    // // target by filtering
    // cartesian_stiffness_ =
    //     filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
    // cartesian_damping_ =
    //     filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
    // nullspace_stiffness_ =
    //     filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
    // std::lock_guard<std::mutex> position_d_target_mutex_lock(
    //     position_and_orientation_d_target_mutex_);
    // position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
    // orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
  }

  // Eigen::Matrix<double, 7, 1> FOB_controller::saturateTorqueRate(
  //     const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
  //     const Eigen::Matrix<double, 7, 1> &tau_J_d)
  // { // NOLINT (readability-identifier-naming)
  //   Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  //   for (size_t i = 0; i < 7; i++)
  //   {
  //     double difference = tau_d_calculated[i] - tau_J_d[i];
  //     tau_d_saturated[i] =
  //         tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  //   }
  //   return tau_d_saturated;
  // }

  // void FOB_controller::complianceParamCallback(
  //     phri_fob::compliance_paramConfig &config,
  //     uint32_t /*level*/)
  // {
  //   cartesian_stiffness_target_.setIdentity();
  //   cartesian_stiffness_target_.topLeftCorner(3, 3)
  //       << config.translational_stiffness * Eigen::Matrix3d::Identity();
  //   cartesian_stiffness_target_.bottomRightCorner(3, 3)
  //       << config.rotational_stiffness * Eigen::Matrix3d::Identity();
  //   cartesian_damping_target_.setIdentity();
  //   // Damping ratio = 1
  //   cartesian_damping_target_.topLeftCorner(3, 3)
  //       << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
  //   cartesian_damping_target_.bottomRightCorner(3, 3)
  //       << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
  //   nullspace_stiffness_target_ = config.nullspace_stiffness;
  // }

  // void FOB_controller::equilibriumPoseCallback(
  //     const geometry_msgs::PoseStampedConstPtr &msg)
  // {
  //   std::lock_guard<std::mutex> position_d_target_mutex_lock(
  //       position_and_orientation_d_target_mutex_);
  //   position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  //   Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  //   orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
  //       msg->pose.orientation.z, msg->pose.orientation.w;
  //   if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0)
  //   {
  //     orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  //   }
  // }

} // namespace phri_fob

PLUGINLIB_EXPORT_CLASS(phri_fob::FOB_controller, controller_interface::ControllerBase)
