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

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <phri_fob/FOB_paramConfig.h>



namespace phri_fob
{


    // Utils

    /**
     * Exponential smoothing (1st order low-pass)
     * 
     * Given a cut-off frequency ``Fc`` and a sampling frequency ``Fs`` it returns the alpha value to plug into 
     * `` 
     * low_passed_val = current_val * alpha + previous_val * (1 - alpha) 
     * ``
     */
    double computeAlphaExp(double Fc, double Fs) {
        return 1.0 - std::exp(-2.0 * M_PI * Fc / Fs);
    }

    
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

        // By designs the controller runs at 1k
        double controller_freq = 1000;

        // Tracking controller trajectory
        Eigen::Matrix<double, 7, 1> q_d;
        Eigen::Matrix<double, 7, 1> dq_d;

        // Tracking controller params
        Eigen::Matrix<double, 7, 1> tracking_Kp;
        Eigen::Matrix<double, 7, 1> tracking_Kd;
        Eigen::Matrix<double, 7, 1> tracking_Ki;

        /**
         * Trajectory frequency in Hz
         */
        double traj_freq = 0;
        
        /**
         * Trajectory amplitude
         */
        double traj_amp = 0;

        // Force controller params
        Eigen::Matrix<double, 7, 1> force_Kp;

        double Fz = 0;
        Eigen::Matrix<double, 6, 1> desired_cartesian_force_torque;

        /**
         * Force target in the -Z direction (positive is down)
         */

         
        // Model reference
        Eigen::Matrix<double, 7, 1> motors_inertia;

        /** 
         * FOB feedback controller
         * 
         * Alpha parameter (for the exponential smoothing filter) computed based on cutoff frequency for lower and upper motors
         */
        Eigen::Matrix<double, 7, 1> FOB_alpha_Q;

        /**
         * FOB friction shaper
         * 
         * Friction shaper term for lower and upper motors
         */
        Eigen::Matrix<double, 7, 1> FOB_fr;


        /**
         * FOB activation flag
         */
        Eigen::Matrix<double, 7, 1> FOB_active;
        
    private:
        // whenever the controller started
        uint64_t nsec_init = 0;


        // Params from the reconfigure server
        // These raw params will be applied gradually to avoid huge commands
        double _pc_kp;
        double _pc_kd;
        double _pc_ki;
        double _ref_A;
        double _ref_freq;
        double _fc_kp;
        double _fc_ki;
        double _fc_fz;
        double _fc_A;
        double _fc_freq;
        bool _fc_sinusoidal;
        int _mode;
        bool _FOB_lower;
        bool _FOB_upper;
        double _q_lower;
        double _q_upper;
        double _fr_lower;
        double _fr_upper;
        double _motor_inertia;
        double _lower_motors_multiplier;
        double _upper_motors_multiplier;
        bool _reset_and_restart_btn;

        // publisher nodes
        ros::Publisher desired_trajectory_node;
        // ros::Publisher tauExtHatFiltered;
        ros::Publisher tau_frc_hat_node;

        //  Low-pass hyperparameter
        double alpha_lp = computeAlphaExp(30, controller_freq);

        
        // Initial configuration
        Eigen::Matrix<double, 7, 1> q_initial;
        Eigen::Matrix<double, 7, 1> q_initial_;

        // Previous values for later low-passing
        Eigen::Matrix<double, 7, 1> tau_frc_hat_prev;
        Eigen::Matrix<double, 7, 1> tau_ext_prev;
        Eigen::Matrix<double, 7, 1> tau_cmd_prev;
        Eigen::Matrix<double, 7, 1> tau_ext_hat_filtered_prev;
        Eigen::Matrix<double, 7, 1> dq_prev;
        Eigen::Matrix<double, 7, 1> ddq_prev;

        Eigen::Matrix<double, 7, 1> tau_ext_initial;

        Eigen::Matrix<double, 7, 1> q_error_integral;
        Eigen::Matrix<double, 7, 1> tau_error_integral;

        // Robot handles
        std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
        std::vector<hardware_interface::JointHandle> joint_handles_;
        std::vector<hardware_interface::JointHandle> position_joint_handles_;

        // Dynamic reconfigure
        std::unique_ptr<dynamic_reconfigure::Server<phri_fob::FOB_paramConfig>> FOB_param_;
        ros::NodeHandle FOB_param_node_;
        void FOBParamCallback(phri_fob::FOB_paramConfig& config, uint32_t level);
    };



} // namespace phri_fob
