// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>


#include <std_msgs/Float32MultiArray.h>

#include <phri_fob/compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace phri_fob
{
    class Identification_controller : public controller_interface::MultiInterfaceController<
                               franka_hw::FrankaModelInterface,
                               hardware_interface::EffortJointInterface,
                               hardware_interface::PositionJointInterface,
                               franka_hw::FrankaStateInterface>
    {
    public:
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
        void starting(const ros::Time &) override;
        void update(const ros::Time &, const ros::Duration &period) override;
        
        Eigen::Matrix<double, 7, 1> KP;
        Eigen::Matrix<double, 7, 1> KD;

        Eigen::Matrix<double, 7, 1> motors_inertia;

    private:
        ros::Publisher desiredTrajPub;

        double alpha = 0.95;
        Eigen::Matrix<double, 7, 1> q_initial;

        Eigen::Matrix<double, 7, 1> prev_torque_frc_estimated;

        Eigen::Matrix<double, 7, 1> prev_joints_dq;
        Eigen::Matrix<double, 7, 1> prev_joints_ddq;


        // Saturation
        Eigen::Matrix<double, 7, 1>
        saturateTorqueRate(
            const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
            const Eigen::Matrix<double, 7, 1> &tau_J_d); // NOLINT (readability-identifier-naming)

        std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
        std::vector<hardware_interface::JointHandle> joint_handles_;
        std::vector<hardware_interface::JointHandle> position_joint_handles_;

        double filter_params_{0.005};
        double nullspace_stiffness_{20.0};
        double nullspace_stiffness_target_{20.0};
        const double delta_tau_max_{1.0};
        Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
        Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
        Eigen::Matrix<double, 6, 6> cartesian_damping_;
        Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
        Eigen::Matrix<double, 7, 1> q_d_nullspace_;
        Eigen::Vector3d position_d_;
        Eigen::Quaterniond orientation_d_;
        std::mutex position_and_orientation_d_target_mutex_;
        Eigen::Vector3d position_d_target_;
        Eigen::Quaterniond orientation_d_target_;

        // Dynamic reconfigure
        // std::unique_ptr<dynamic_reconfigure::Server<phri_fob::compliance_paramConfig>>
        //     dynamic_server_compliance_param_;
        // ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
        // void complianceParamCallback(phri_fob::compliance_paramConfig &config,
        //                              uint32_t level);

        // Equilibrium pose subscriber
        // ros::Subscriber sub_equilibrium_pose_;
        // void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    };

} // namespace phri_fob
