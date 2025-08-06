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

    class FOB_controller : public controller_interface::MultiInterfaceController<
                               franka_hw::FrankaModelInterface,
                               hardware_interface::EffortJointInterface,
                               hardware_interface::PositionJointInterface,
                               franka_hw::FrankaStateInterface>
    {
    public:
        // ROS node lifecycle callbacks
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
        void starting(const ros::Time &) override;
        void update(const ros::Time &, const ros::Duration &period) override;

        // Tracking controller params
        Eigen::Matrix<double, 7, 1> KP;
        Eigen::Matrix<double, 7, 1> KD;

        // Trajectory params
        double trajectory_freq = 8;
        double trajectory_scale = 1 / (2.0 * M_PI);

        // MOdel reference
        Eigen::Matrix<double, 7, 1> motors_inertia;

        // MR-FOB feedback controller
        Eigen::Matrix<double, 7, 1> Q;

        // MR-FOB friction shaper
        Eigen::Matrix<double, 7, 1> f_r;
        

    private:
        // publisher nodes
        ros::Publisher desiredTrajPub;
        ros::Publisher torqueWOGravity;
        ros::Publisher tauFrcHat;

        //  Low-pass hyperparameter
        double alpha = 0.95;

        // Initial configuration
        Eigen::Matrix<double, 7, 1> q_initial;

        Eigen::Matrix<double, 7, 1> tau_frc_hat_prev;
        Eigen::Matrix<double, 7, 1> dq_prev;
        Eigen::Matrix<double, 7, 1> ddq_prev;

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
