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

    boost::recursive_mutex fob_mutex_;

    bool FOB_controller::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
    {
        // Topic setup
        // desiredTrajPub = node_handle.advertise<std_msgs::Float32MultiArray>("desired_trajectory", 1);
        // tauExtHatFiltered = node_handle.advertise<std_msgs::Float32MultiArray>("tau_ext_hat_filtered", 1);
        traFrcHatNode = node_handle.advertise<std_msgs::Float32MultiArray>("tau_frc_hat", 1);

        std::vector<std::string> joint_names;
        std::string arm_id;
        ROS_WARN(
            "FOB_controller: Make sure your robot's endeffector is in contact "
            "with a horizontal surface before starting the controller!");
        if (!node_handle.getParam("arm_id", arm_id))
        {
            ROS_ERROR("FOB_controller: Could not read parameter arm_id");
            return false;
        }
        if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7)
        {
            ROS_ERROR(
                "FOB_controller: Invalid or no joint_names parameters provided, aborting "
                "controller init!");
            return false;
        }

        auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
        if (model_interface == nullptr)
        {
            ROS_ERROR_STREAM("FOB_controller: Error getting model interface from hardware");
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
                "FOB_controller: Exception getting model handle from interface: " << ex.what());
            return false;
        }

        auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
        if (state_interface == nullptr)
        {
            ROS_ERROR_STREAM("FOB_controller: Error getting state interface from hardware");
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
                "FOB_controller: Exception getting state handle from interface: " << ex.what());
            return false;
        }

        auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (effort_joint_interface == nullptr)
        {
            ROS_ERROR_STREAM("FOB_controller: Error getting effort joint interface from hardware");
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
                ROS_ERROR_STREAM("FOB_controller: Exception getting joint handles: " << ex.what());
                return false;
            }
        }

        // Param server setup
        FOB_param_node_ = ros::NodeHandle("dynamic_reconfigure_FOB_param_node");
        FOB_param_ = std::make_unique<dynamic_reconfigure::Server<phri_fob::FOB_paramConfig>>(fob_mutex_, FOB_param_node_);
        FOB_param_->setCallback(boost::bind(&FOB_controller::FOBParamCallback, this, _1, _2));

        // This triggers the callback to update all the parameters
        phri_fob::FOB_paramConfig cfg_default;
        FOB_param_->getConfigDefault(cfg_default);  
        FOB_param_->updateConfig(cfg_default);    

        return true;
    }

    void FOB_controller::starting(const ros::Time & /*time*/)
    {
        
        franka::RobotState robot_state = state_handle_->getRobotState();
        std::array<double, 7> gravity_array = model_handle_->getGravity();

        Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> current_q(robot_state.q.data());

        // Bias correction for the current external torque
        tau_ext_initial = tau_measured - gravity;

        // Bias correction for the current external position
        q_initial = current_q;

        // Motor model
        // motors_inertia << 
        //     20 * 0.075, // 7.5e-5,  // J1 (87 Nm)
        //     20 * 0.075, // 7.5e-5,  // J2 (87 Nm)
        //     20 * 0.075, // 7.5e-5,  // J3 (87 Nm)
        //     20 * 0.075, // 7.5e-5,  // J4 (87 Nm)
        //     3 * 0.075,  // 4.5e-6,  // J5 (12 Nm)
        //     3 * 0.075,  // 4.5e-6,  // J6 (12 Nm)
        //     3 * 0.075;  // 4.5e-6;  // J7 (12 Nm)
            
        desired_cartesian_force_torque.setZero();
// 
        // FOB_fr.setZero();
        // tau_error.setZero();
// 
        // q_error.setZero();
        // tau_frc_hat_prev.setZero();
        // ddq_prev.setZero();
        // tau_cmd_prev.setZero();
        // tau_ext_hat_filtered_prev.setZero();
        // tau_ext_prev = tau_ext_initial;
// 
        // FOB_active.setConstant(0);
        // FOB_alpha_Q.setConstant(computeAlphaExp(0, 1000));

       
    }

    void FOB_controller::update(const ros::Time &time, const ros::Duration &period)
    {
        franka::RobotState robot_state = state_handle_->getRobotState();
        std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
        std::array<double, 7> gravity_array = model_handle_->getGravity();

        // Getting whole robot state 
        Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
        Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
        
        Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_ext_hat_filtered(robot_state.tau_ext_hat_filtered.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());

        Eigen::Matrix<double, 7, 1> tau_d, tau_cmd, tau_ext;

        // Low passed desired wrench (to avoid huge commands)
        desired_cartesian_force_torque(2) = -Fz * alpha_lp + desired_cartesian_force_torque(2) * (alpha_lp - 1);
        // ... moving to joint space
        tau_d = jacobian.transpose() * desired_cartesian_force_torque;

        // External torque of each joint
        tau_ext = tau_measured - gravity - tau_ext_initial;

        // Compute errors for controllers
        // q_error_ = q_error_ + period.toSec() * (q_initial - q);
        tau_error = tau_error + period.toSec() * (tau_d - tau_ext);

        switch (_mode)
        {
        case 0:
            // FORCE_CONTROL
            // FF + PI control (PI gains are initially all 0)
            tau_cmd = tau_d + _fc_kp * (tau_d - tau_ext) + _fc_ki * tau_error;
            break;
        case 1:
            // TRACKING CONTROLLER
            // TODO: implement this
        default:
            tau_cmd.setConstant(0.0);
            break;
        }
    
        // Euler approx. for acceleration calculation
        // period is fixed at 1ms (1kHz controller loop)
        Eigen::Matrix<double, 7, 1> ddq = (dq - dq_prev) / 0.001f;
        // Removed because we filter with the Q filter
        ddq = alpha_lp * ddq + (1 - alpha_lp) * ddq_prev;

        // Model-based computed motor torque 
        auto tau_m_model = ddq.cwiseProduct(motors_inertia) + dq.cwiseProduct(FOB_fr);

        // Torque estimation due to friction
        Eigen::Matrix<double, 7, 1> tau_frc_hat = tau_m_model - (tau_cmd_prev - tau_ext_prev);
        // ... low-passed
        tau_frc_hat = FOB_alpha_Q.cwiseProduct(tau_frc_hat) + (Eigen::Matrix<double, 7, 1>::Ones() - FOB_alpha_Q).cwiseProduct(tau_frc_hat_prev);

        // Apply command to the robot torque joint handles
        for (size_t i = 0; i < 7; ++i)
        {         
            joint_handles_[i].setCommand(_reset_and_restart_btn ? 0 : tau_cmd(i) - (FOB_active(i) * tau_frc_hat(i)));
        }

        
        // Publish data 
        
        std_msgs::Float32MultiArray float32MultiArrayMsg;
        float32MultiArrayMsg.data.resize(7);
        
        // for (size_t i = 0; i < 7; ++i)
        // {
        //   float32MultiArrayMsg.data[i] = tau_frc_hat(i);
        // }

        // traFrcHatNode.publish(float32MultiArrayMsg);
        
        std::copy(tau_frc_hat.data(), tau_frc_hat.data() + 7, float32MultiArrayMsg.data.begin());
        traFrcHatNode.publish(float32MultiArrayMsg);

        // Update signals changed online through dynamic reconfigure
        // desired_mass_ = filter_gain_ * target_mass_ + (1 - filter_gain_) * desired_mass_;
        // k_p_ = filter_gain_ * target_k_p_ + (1 - filter_gain_) * k_p_;
        // k_i_ = filter_gain_ * target_k_i_ + (1 - filter_gain_) * k_i_;
        // f_Z = filter_gain_ * f_Z_ + (1 - filter_gain_) * f_Z;

        tau_frc_hat_prev = tau_frc_hat;
        dq_prev = dq;
        ddq_prev = ddq;
        tau_cmd_prev = tau_cmd;
        tau_ext_prev = tau_ext;
        tau_ext_hat_filtered_prev = tau_ext_hat_filtered;
    }

    void FOB_controller::FOBParamCallback(phri_fob::FOB_paramConfig &config, uint32_t /*level*/)
    {
        // Saving all the reconfigure server params
        _pc_kp = config.pc_kp;
        _pc_kd = config.pc_kd;
        _pc_ki = config.pc_ki;
        _ref_A = config.ref_A;
        _ref_freq = config.ref_freq;
        _fc_kp = config.fc_kp;
        _fc_ki = config.fc_ki;
        _fc_fz = config.fc_fz;
        _mode = config.mode;
        _FOB_lower = config.FOB_lower;
        _FOB_upper = config.FOB_upper;
        _q_lower = config.q_lower;
        _q_upper = config.q_upper;
        _fr_lower = config.fr_lower;
        _fr_upper = config.fr_upper;
        _motor_inertia = config.motor_inertia;
        _lower_motors_multiplier = config.lower_motors_multiplier;
        _upper_motors_multiplier = config.upper_motors_multiplier;
        _reset_and_restart_btn = config.reset_and_restart_btn;

        // Applying all the parameters to the current setup

        FOB_alpha_Q(0) = computeAlphaExp(_q_lower, 1000);
        FOB_alpha_Q(1) = computeAlphaExp(_q_lower, 1000);
        FOB_alpha_Q(2) = computeAlphaExp(_q_lower, 1000);
        FOB_alpha_Q(3) = computeAlphaExp(_q_lower, 1000);
        FOB_alpha_Q(4) = computeAlphaExp(_q_upper, 1000);
        FOB_alpha_Q(5) = computeAlphaExp(_q_upper, 1000);
        FOB_alpha_Q(6) = computeAlphaExp(_q_upper, 1000);

        motors_inertia(0) = _motor_inertia * _lower_motors_multiplier;
        motors_inertia(1) = _motor_inertia * _lower_motors_multiplier;
        motors_inertia(2) = _motor_inertia * _lower_motors_multiplier;
        motors_inertia(3) = _motor_inertia * _lower_motors_multiplier;
        motors_inertia(4) = _motor_inertia * _upper_motors_multiplier;
        motors_inertia(5) = _motor_inertia * _upper_motors_multiplier;
        motors_inertia(6) = _motor_inertia * _upper_motors_multiplier;

        FOB_fr(0) = _fr_lower;
        FOB_fr(1) = _fr_lower;
        FOB_fr(2) = _fr_lower;
        FOB_fr(3) = _fr_lower;
        FOB_fr(4) = _fr_upper;
        FOB_fr(5) = _fr_upper;
        FOB_fr(6) = _fr_upper;

        Fz = _fc_fz;

        FOB_active(0) = _FOB_lower;
        FOB_active(1) = _FOB_lower;
        FOB_active(2) = _FOB_lower;
        FOB_active(3) = _FOB_lower;
        FOB_active(4) = _FOB_upper;
        FOB_active(5) = _FOB_upper;
        FOB_active(6) = _FOB_upper;

        if (_reset_and_restart_btn) {
            franka::RobotState robot_state = state_handle_->getRobotState();
            std::array<double, 7> gravity_array = model_handle_->getGravity();

            Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
            Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
            Eigen::Map<Eigen::Matrix<double, 7, 1>> current_q(robot_state.q.data());

            // Bias correction for the current external torque
            tau_ext_initial = tau_measured - gravity;
            tau_error.setConstant(0);

            // Bias correction for the current external position
            q_initial = current_q;
        }
    }
}

PLUGINLIB_EXPORT_CLASS(phri_fob::FOB_controller, controller_interface::ControllerBase)
