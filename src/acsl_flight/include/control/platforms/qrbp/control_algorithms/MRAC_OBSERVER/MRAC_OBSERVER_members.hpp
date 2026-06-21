/***********************************************************************************************************************
 * Copyright (c) 2024 Giri M. Kumar, Mattia Gramuglia, Andrea L'Afflitto. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 *    disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * File:        MRAC_OBSERVER_members.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        December 21, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: MRAC with angular velocities and state observer controller class members.
 *              This file contains the members for the MRAO, 2L MRAO, MRAD, 2L MRAD.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

/*
________/\\\__________/\\\\\\\\\______/\\\\\\\\\\\\\____/\\\\\\\\\\\\\___        
 _____/\\\\/\\\\_____/\\\///////\\\___\/\\\/////////\\\_\/\\\/////////\\\_       
  ___/\\\//\////\\\__\/\\\_____\/\\\___\/\\\_______\/\\\_\/\\\_______\/\\\_      
   __/\\\______\//\\\_\/\\\\\\\\\\\/____\/\\\\\\\\\\\\\\__\/\\\\\\\\\\\\\/__     
    _\//\\\______/\\\__\/\\\//////\\\____\/\\\/////////\\\_\/\\\/////////____    
     __\///\\\\/\\\\/___\/\\\____\//\\\___\/\\\_______\/\\\_\/\\\_____________   
      ____\////\\\//_____\/\\\_____\//\\\__\/\\\_______\/\\\_\/\\\_____________  
       _______\///\\\\\\__\/\\\______\//\\\_\/\\\\\\\\\\\\\/__\/\\\_____________ 
        _________\//////___\///________\///__\/////////////____\///______________
*/

#ifndef CONTROLLERS_MRAC_OBSERVER_MEMBERS_HPP_
#define CONTROLLERS_MRAC_OBSERVER_MEMBERS_HPP_

#include "Eigen/Dense"
#include <chrono>
#include "qrbp.hpp"                         // Header file for vehicle specific information and some other functions
#include "aero_qrbp.hpp"                    // Header file for aerodynamic specific information and some

using namespace Eigen;
namespace _qrbp_{
namespace _mrac_observer_{

// Structure for all the aerodynamic members of the controller
struct controller_internal_members_aero
{
    AeroStates states;
    AeroCoeff coeff;
    AeroDynamicMembers dyn;
};

// Structure for all parameter members of the controller
struct controller_internal_paramters
{

    // Outerloop parameters ----------------------------------------------------------------------------------------------
    /// Baseline Paramters
    // Trnaslational gains to let the reference model follow the user-defined trajectory
    Matrix<double, 3, 3> Kp_refmod_tran;
    Matrix<double, 3, 3> Ki_refmod_tran;
    Matrix<double, 3, 3> Kd_refmod_tran;

    // Translational gains for the Baseline controller to follow the reference model
    Matrix<double, 3, 3> Kp_tran;
    Matrix<double, 3, 3> Ki_tran;
    Matrix<double, 3, 3> Kd_tran;

    /// Adaptive parameters - Translational
    // Translational Adaptive gains
    Matrix<double, 6, 6> Gamma_x_tran;
    Matrix<double, 3, 3> Gamma_r_tran;
    Matrix<double, 30, 30> Gamma_Theta_tran;

    // Translational parameters Lyapunov equation
    Matrix<double, 6, 6> Q_tran;
    Matrix<double, 6, 6> P_tran;

    // Translational dynamics plant parameters
    Matrix<double, 6, 6> A_tran;
    Matrix<double, 6, 3> B_tran;

    // Translational reference model parameters
    Matrix<double, 6, 6> A_ref_tran;
    Matrix<double, 6, 3> B_ref_tran;
    
    // Innerloop parameters ----------------------------------------------------------------------------------------------
    /// Baseline Parameters
    // Rotational gains for the reference model and command to follow the user-defined trajectory
    Matrix<double, 3, 3> Kp_omega_ref;
    Matrix<double, 3, 3> Ki_omega_ref;

    // Rotational gains for the baseline controller to follow the reference model
    Matrix<double, 3, 3> Kp_rot;
    Matrix<double, 3, 3> Ki_rot;
    Matrix<double, 3, 3> Kd_rot;

    /// Adaptive parameters - Translational
    // Rotational Adaptive gains
    Matrix<double, 3, 3> Gamma_x_rot;
    Matrix<double, 3, 3> Gamma_r_rot;
    Matrix<double, 12, 12> Gamma_Theta_rot;

    // Rotationtational parameters Lyapunov equation
    Matrix<double, 3, 3> Q_rot;
    Matrix<double, 3, 3> P_rot;

    // Rotaional dynamics plant parameters
    Matrix<double, 3, 3> A_rot;
    Matrix<double, 3, 3> B_rot;
    
    // Rotational reference model parameters
    Matrix<double, 3, 3> A_ref_rot;
    Matrix<double, 3, 3> B_ref_rot;

    // Robustification parameters ----------------------------------------------------------------------------------------
    // Deadzone operator
    double dead_zone_delta_translational;
    double dead_zone_e0_translational;
    double dead_zone_delta_rotational;
    double dead_zone_e0_rotational;

    // E-modification 
    double sigma_x_translational;
    double sigma_r_translational;
    double sigma_Theta_translational;
    double sigma_x_rotational;
    double sigma_r_rotational;
    double sigma_Theta_rotational;

    // Ellipsoid projection operator
    double projection_x_max_x_translational; 
    double projection_epsilon_x_translational;

    double projection_x_max_r_translational;
    double projection_epsilon_r_translational;

    double projection_x_max_Theta_translational;
    double projection_epsilon_Theta_translational;

    double projection_x_max_x_rotational; 
    double projection_epsilon_x_rotational;

    double projection_x_max_r_rotational;
    double projection_epsilon_r_rotational;

    double projection_x_max_Theta_rotational;
    double projection_epsilon_Theta_rotational;    
};

// Structure for the members that are mapped to the rk4 vector after integration
struct controller_integrated_state_members
{
    // States for filter
    Matrix<double, 2, 1> state_roll_d_filter;
    Matrix<double, 2, 1> state_pitch_d_filter;

    Matrix<double, 2, 1> state_omega_x_d_filter;
    Matrix<double, 2, 1> state_omega_y_d_filter;
    Matrix<double, 2, 1> state_omega_z_d_filter;

    // Translational Integral error
    Matrix<double, 3, 1> e_tran_pos_I;

    // Translational Integral error between reference model and user defined trajectory
    Matrix<double, 3, 1> e_tran_pos_ref_I;

    // Reference model in I
    Matrix<double, 6, 1> x_tran_ref;

    // Translational Adaptive gains for x
    Matrix<double, 6, 3> K_hat_x_tran;

    // Translational Adaptive gains for r
    Matrix<double, 3, 3> K_hat_r_tran;

    // Translational Adaptive gains for Theta
    Matrix<double, 30, 3> Theta_hat_tran;

    // Integral error between the angular velocity reference model and the desired angular velocity
	Matrix<double, 3, 1> e_rot_omega_ref_I;

    // Integral error in orientation
    Matrix<double, 3, 1> e_rot_eta_I;

    // Angular Velocity Reference model
    Matrix<double, 3, 1> omega_rot_ref;

    // Rotational Adaptive gains for x
    Matrix<double, 3, 3> K_hat_x_rot;

    // Rotational Adaptive gains for r
    Matrix<double, 3, 3> K_hat_r_rot;

    // Rotational Adaptive gains for Theta
    Matrix<double, 12, 3> Theta_hat_rot;
    
};

// Structure for the internal members of the controller
struct controller_internal_members
{
    // Time
    double t;

    // Control execution duration
    double alg_duration;
    std::chrono::high_resolution_clock::time_point alg_start_time;
    std::chrono::high_resolution_clock::time_point alg_end_time;

    controller_internal_members_aero aero;

    // Rotation matrices
    Matrix<double, 3, 3> R_J_I;
    Matrix<double, 3, 3> R_I_J;
    Matrix<double, 3, 3> R_W_J;

    // Jacobian matrices
    Matrix<double, 3, 3> Jacobian;

    // Translational User Commands
    Matrix<double, 3, 1> r_user;
    Matrix<double, 3, 1> r_dot_user;
    Matrix<double, 3, 1> r_ddot_user;

    // Quaternion 
    Quaterniond q;

    // Translational states - position and velocity vector
    Matrix<double, 6, 1> x_tran;

    // Reference model in I
    Matrix<double, 6, 1> x_tran_ref_dot;

    // Translational velocities in J
    Matrix<double, 3, 1> x_tran_vel_J;

    // Translational Adaptive gains for x for integration
    Matrix<double, 6, 3> K_hat_x_tran_dot;

    // Translational Adaptive gains for r for integration
    Matrix<double, 3, 3> K_hat_r_tran_dot;

    // Translational Adaptive gains for Theta for integration
    Matrix<double, 30, 3> Theta_hat_tran_dot;

    // Translational regressor vector
    Matrix<double, 27, 1> outer_loop_regressor;
    Matrix<double, 30, 1> augmented_outer_loop_regressor;

    // Translational error
    Matrix<double, 6, 1> e_tran;
    Matrix<double, 3, 1> e_tran_pos;
    Matrix<double, 3, 1> e_tran_vel;

    // Translational error between reference model and the user-defined trajectory
    Matrix<double, 3, 1> e_tran_pos_ref;

    // Translational command input
    Matrix<double, 3, 1> r_cmd_tran;

    // Translational baseline control input 
    Matrix<double, 3, 1> mu_tran_baseline;

    // Translational adaptive control 
    Matrix<double, 3, 1> mu_tran_adaptive;

    // Translational control input in the inertial frame
    Matrix<double, 3, 1> mu_tran_I;

    // Translational control input in the body frame
    Matrix<double, 3, 1> mu_tran_J;

    // Internal States for filter
    Matrix<double, 2, 1> internal_state_roll_d_filter;
    Matrix<double, 2, 1> internal_state_pitch_d_filter;

    Matrix<double, 2, 1> internal_state_omega_x_d_filter;
    Matrix<double, 2, 1> internal_state_omega_y_d_filter;
    Matrix<double, 2, 1> internal_state_omega_z_d_filter;

    // Rotational states 
    // orientation
    Matrix<double, 3, 1> eta_rot;
    // angular velocity
    Matrix<double, 3, 1> omega_rot;

    // Desired Euler Angles
    Matrix<double, 3, 1> eta_rot_d;

    // Desired Euler Rates
    Matrix<double, 3, 1> eta_rot_rate_d;

    // Desired Angular Velocities
    Matrix<double, 3, 1> omega_rot_d;

    // Desired Angular Acceleration
    Matrix<double, 3, 1> alpha_rot_d;

    // Error between the orientation and desired orientation
    Matrix<double, 3, 1> e_rot_eta;

    // Error betweeen the angular velocity and the reference angular velocity
    Matrix<double, 3, 1> e_rot_omega;

    // Error between the angular velocity reference model and the desired angular velocity
	Matrix<double, 3, 1> e_rot_omega_ref;

    // Commanded Angular Velocity
    Matrix<double, 3, 1> omega_cmd;

    // Derivative of the Angular Velocity Reference model
    Matrix<double, 3, 1> omega_rot_ref_dot;

    // Baseline rotational control input
    Matrix<double, 3, 1> tau_rot_baseline;

    // Rotational regressor vector
    Matrix<double, 9, 1> inner_loop_regressor;
    Matrix<double, 12, 1> augmented_inner_loop_regressor;

    // Rotational Adaptive gains for x for integration
    Matrix<double, 3, 3> K_hat_x_rot_dot;

    // Rotational Adaptive gains for r for integration
    Matrix<double, 3, 3> K_hat_r_rot_dot;

    // Rotational Adaptive gains for Theta for integration
    Matrix<double, 12, 3> Theta_hat_rot_dot;

    // Rotational Adaptive control input
    Matrix<double, 3, 1> tau_rot_adaptive;

    // Rotational control input
    Matrix<double, 3, 1> tau_rot;

    // Final Control input
    Matrix<double, 4, 1> u;

    // Thrust 
    Matrix<double, 4,1> Thrust;

    // Thrust after Saturation
    Matrix<double, 4,1> Sat_Thrust;

    // Deadzone operator values
    double dead_zone_value_translational;
    double dead_zone_value_rotational;

    // Projection activation values
    bool proj_op_activated_K_hat_x_translational;
    bool proj_op_activated_K_hat_r_translational;
    bool proj_op_activated_Theta_hat_translational;

    bool proj_op_activated_K_hat_x_rotational;
    bool proj_op_activated_K_hat_r_rotational;
    bool proj_op_activated_Theta_hat_rotational;
};

// Structure for all parameter members of the observers
struct observer_internal_paramters
{
    Matrix<double, 3, 6> C_observer;                    // C matrix for the observer plants - Both 2L and MRAO
    Matrix<double, 6, 3> B_observer;                    // B matrix for the observer plants - Both 2L and MRAO  
    Matrix<double, 6, 3> L_observer;                    // L matrix for the observer plants - Both 2L and MRAO  
    Matrix<double, 6, 6> A_ref_y_observer;              // A matrix for the observer reference models - Both 2L and MRAO
    Matrix<double, 6, 6> A_tran_y_observer;             // A matrix for the transient error plant - Both 2L and MRAO
    Matrix<double, 3, 3> Gamma_y_observer;              // Adaptive gain matrix for the observers - Both 2L and MRAO
    Matrix<double, 4, 4> Gamma_Theta_observer;          // Adaptive gain matrix for the observers - Both 2L and MRAO
    Matrix<double, 3, 3> Gamma_g_y_observer;            // Adaptive gain matrix for the observer - Only 2L MRAO
};

// Structure for the members that are mapped to the rk4 vector after integration
struct observer_integrated_state_members
{   
    Matrix<double, 6, 1> x_hat_mrao;                // Estimated translational states for the MRAO
    Matrix<double, 6, 1> x_hat_2l_mrao;             // Estimated translational states for the 2L MRAO
    Matrix<double, 3, 3> K_hat_y_mrao;              // Adaptive gain matrix for the MRAO
    Matrix<double, 3, 3> K_hat_y_2l_mrao;           // Adaptive gain matrix for the 2L MRAO
    Matrix<double, 4, 3> Theta_hat_mrao;            // Adaptive gain matrix for the MRAO
    Matrix<double, 6, 1> Theta_hat_2l_mrao;         // Adaptive gain matrix for the 2L MRAO
    Matrix<double, 3, 3> K_hat_g_y_2l_mrao;         // Adaptive gain matrix for the 2L MRAO transient term
    Matrix<double, 6, 1> eta_2l_mrao;               // Transient error model for the 2L MRAO
};

// Structure for the internal members of the observers
struct observer_internal_members
{
    Matrix<double, 6, 1> x_hat_mrao_dot;           // Estimated translational states for the MRAO
    Matrix<double, 6, 1> x_hat_2l_mrao_dot;        // Estimated translational states for the 2L MRAO
    Matrix<double, 3, 3> K_hat_y_mrao_dot;         // Adaptive gain matrix for the MRAO
    Matrix<double, 3, 3> K_hat_y_2l_mrao_dot;      // Adaptive gain matrix for the 2L MRAO
    Matrix<double, 4, 3> Theta_hat_mrao_dot;       // Adaptive gain matrix for the MRAO
    Matrix<double, 6, 1> Theta_hat_2l_mrao_dot;    // Adaptive gain matrix for the 2L MRAO
    Matrix<double, 3, 3> K_hat_g_y_2l_mrao_dot;    // Adaptive gain matrix for the 2L MRAO transient term
    Matrix<double, 6, 1> eta_2l_mrao_dot;          // Transient error model for the 2L MRAO

    Matrix<double, 3, 1> y_measured_mrao;          // Measured outputs for the MRAO 
    Matrix<double, 3, 1> y_measured_2l_mrao;       // Measured outputs for the 2L MRAO  
    Matrix<double, 4, 1> Phi_y_mrao;               // Regressor vector for the MRAO
    Matrix<double, 4, 1> Phi_y_2l_mrao;            // Regressor vector for the 2L MRAO
    Matrix<double, 6, 1> nu_2l_mrao;               // Transient error for the 2L MRAO
};


} // namespace _mrac_observer_
} // namespace _qrbp_

#endif // CONTROLLERS_MRAC_OBSERVER_MEMBERS_HPP_