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
 * File:        MRAC_GEOMETRIC_TD_members.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        July 16, 2026
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: MRAC with SO(3) controller class members.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

/*               _           _      _                      
 $$$$$$\  $$\   $$\  $$$$$$\  $$$$$$$\  $$\      $$\ 
$$  __$$\ $$ |  $$ |$$  __$$\ $$  __$$\ $$$\    $$$ |
$$ /  $$ |$$ |  $$ |$$ /  $$ |$$ |  $$ |$$$$\  $$$$ |
$$ |  $$ |$$ |  $$ |$$$$$$$$ |$$ |  $$ |$$\$$\$$ $$ |
$$ |  $$ |$$ |  $$ |$$  __$$ |$$ |  $$ |$$ \$$$  $$ |
$$ $$\$$ |$$ |  $$ |$$ |  $$ |$$ |  $$ |$$ |\$  /$$ |
\$$$$$$ / \$$$$$$  |$$ |  $$ |$$$$$$$  |$$ | \_/ $$ |
 \___$$$\  \______/ \__|  \__|\_______/ \__|     \__|
     \___|                                           
*/

#ifndef CONTROLLERS_MRAC_GEOMETRIC_TD_MEMBERS_HPP_
#define CONTROLLERS_MRAC_GEOMETRIC_TD_MEMBERS_HPP_

#include "Eigen/Dense"
#include <chrono>
#include "quadm.hpp"                         // Header file for vehicle specific information and some other functions

using namespace Eigen;
namespace _quadm_{
namespace _mrac_geometric_{

// Structure for all parameter members of the controller
struct controller_internal_parameters {
    Matrix<double, 3, 3> Kp_refmod_tran;     // Proportional Gains for the translational reference model
    Matrix<double, 3, 3> Kd_refmod_tran;     // Derivative Gains for the translational reference model
    Matrix<double, 3, 3> Ki_refmod_tran;     // Integral Gains for the translational reference command
    Matrix<double, 3, 3> Kp_tran;            // Proportional Gains for the translational control
    Matrix<double, 3, 3> Ki_tran;            // Integral Gains for the translational control
    Matrix<double, 3, 3> Kd_tran;            // Derivative Gains for the translational control
    Matrix<double, 6, 6> Gamma_x_tran;       // Adaptive Gains for the translational control
    Matrix<double, 3, 3> Gamma_r_tran;       // Adaptive Gains for the translational control
    Matrix<double, 30, 30> Gamma_Theta_tran; // Adaptive Gains for the translational control
    Matrix<double, 6, 6> Q_tran;          	// Lyapunov weighting matrix
    Matrix<double, 6, 6> P_tran;          	// Solution matrix to continuous Lyapunov equation
    Matrix<double, 6, 6> A_tran;          	// Translational system matrix
    Matrix<double, 6, 3> B_tran;          	// Translational input matrix
    Matrix<double, 6, 6> A_ref_tran;      	// Reference model system matrix
    Matrix<double, 6, 3> B_ref_tran;      	// Reference model input matrix
    double dead_zone_delta_translational;        	// Translational deadzone delta radius
    double dead_zone_e0_translational;           	// Translational deadzone error tolerance
    double sigma_x_translational;                	// Translational E-mod gain for x (states)
    double sigma_r_translational;                	// Translational E-mod gain for r (commands)
    double sigma_Theta_translational;            	// Translational E-mod gain for Theta (parameters)
    double projection_x_max_x_translational;     	// Translational Projection limit for Kx_hat
    double projection_epsilon_x_translational;   	// Translational Projection tolerance for Kx_hat
    double projection_x_max_r_translational;     	// Translational Projection limit for Kr_hat
    double projection_epsilon_r_translational;   	// Translational Projection tolerance for Kr_hat
    double projection_x_max_Theta_translational; 	// Translational Projection limit for Theta_hat
    double projection_epsilon_Theta_translational; 	// Translational Projection tolerance for Theta_hat
    
    Matrix<double, 2, 2> A_filter_mu;        // Differentiator A matrix for \mu
    Matrix<double, 2, 1> B_filter_mu;        // Differentiator B matrix for \mu
    Matrix<double, 1, 2> C_filter_mu;        // Differentiator C matrix for q_d
    Matrix<double, 2, 2> A_filter_omega_d;   // Differentiator A matrix for omega_d
    Matrix<double, 2, 1> B_filter_omega_d;   // Differentiator B matrix for omega_d
    Matrix<double, 1, 2> C_filter_omega_d;   // Differentiator C matrix for omega_d

    Matrix<double, 3, 3> Kp_omega_ref;    	// Proportional rotational reference model gains
    Matrix<double, 3, 3> Ki_omega_ref;    	// Integral rotational reference model gains
    Matrix<double, 3, 3> Kp_att;             // Proportional Gains for the rotational control
    Matrix<double, 3, 3> Kd_att;             // Derivative Gains for the rotational control
    Matrix<double, 3, 1> Ka_att;             // The axis scaling gaisn for the rotational control
    Matrix<double, 3, 3>  Gamma_x_rot;    	// Adaptive gain for state feedback parameters
    Matrix<double, 3, 3>  Gamma_r_rot;    	// Adaptive gain for command tracking parameters
    Matrix<double, 12, 12> Gamma_Theta_rot;  // Adaptive gain for dynamic regression parameters
    Matrix<double, 3, 3> Q_rot;           	// Lyapunov weighting matrix (rotation)
    Matrix<double, 3, 3> P_rot;           	// Lyapunov solution matrix (rotation)
    Matrix<double, 3, 3> A_rot;           	// Rotational system matrix
    Matrix<double, 3, 3> B_rot;           	// Rotational input matrix
    Matrix<double, 3, 3> A_ref_rot;       	// Rotational reference model matrix
    Matrix<double, 3, 3> B_ref_rot;       	// Rotational reference input matrix
    double dead_zone_delta_rotational;           	// Rotational deadzone delta radius
    double dead_zone_e0_rotational;              	// Rotational deadzone error tolerance
    double sigma_x_rotational;                   	// Rotational E-mod gain for x (states)
    double sigma_r_rotational;                   	// Rotational E-mod gain for r (commands)
    double sigma_Theta_rotational;               	// Rotational E-mod gain for Theta (parameters)
    double projection_x_max_x_rotational;        	// Rotational Projection limit for Kx_hat
    double projection_epsilon_x_rotational;      	// Rotational Projection tolerance for Kx_hat
    double projection_x_max_r_rotational;        	// Rotational Projection limit for Kr_hat
    double projection_epsilon_r_rotational;      	// Rotational Projection tolerance for Kr_hat
    double projection_x_max_Theta_rotational;    	// Rotational Projection limit for Theta_hat
    double projection_epsilon_Theta_rotational;  	// Rotational Projection tolerance for Theta_hat 
    
};

// Structure for all the members that are mapped to the rk4 vector AFTER integration
struct controller_integrated_state_members {
  Matrix<double, 3, 1> e_tran_pos_I;               // Translational integral error
  Matrix<double, 3, 1> e_tran_pos_ref_I;		    // Translational Integral error (reference model - user cmd)
  Matrix<double, 6, 1> x_tran_ref;				    // Reference model in I
  Matrix<double, 6, 3> K_hat_x_tran;   			// Translational Adaptive gains for x
  Matrix<double, 3, 3> K_hat_r_tran;	    		// Translational Adaptive gains for r
  Matrix<double, 30, 3> Theta_hat_tran;	    	// Translational Adaptive gains for Theta

  Matrix<double, 2, 1> state_mu_x_filter;          // States for filter
  Matrix<double, 2, 1> state_mu_y_filter;          // States for filter
  Matrix<double, 2, 1> state_mu_z_filter;          // States for filter
  Matrix<double, 2, 1> state_omega_x_d_filter;     // States for filter
  Matrix<double, 2, 1> state_omega_y_d_filter;     // States for filter
  Matrix<double, 2, 1> state_omega_z_d_filter;     // States for filter

  Matrix<double, 3, 1> e_omega_ref_I;   	        // Integral error (ang vel ref model - desired angular velocity)
  Matrix<double, 3, 1> omega_ref;  			    // Angular Velocity Reference model
  Matrix<double, 3, 3> K_hat_x_rot;			    // Rotational Adaptive gains for x
  Matrix<double, 3, 3> K_hat_r_rot;			    // Rotational Adaptive gains for r
  Matrix<double, 12, 3> Theta_hat_rot;			    // Rotational Adaptive gains for Theta
};

// Structure for all the internal members of the controller
struct controller_internal_members {
  double t;                                                      // Time
  Matrix<double, 3, 1> r_user;                            // Translational user position command
  Matrix<double, 3, 1> r_dot_user;                        // Translational user veloctiy command
  Matrix<double, 3, 1> r_ddot_user;                       // Translational user acceleration command 
  double psi_user;                                               // Rotational command for psi
  double psi_user_unwrapped;                                     // Rotational command for psi without the -pi to pi jumps but as a continuous signal
  double psi_dot_user;                                           // Rotational command for psi rate
  Matrix<double, 3, 1> r_cmd_tran;                        // Translational reference command
  Matrix<double, 6, 1> x_tran_ref_dot;                    // Translational reference model
  Matrix<double, 6, 1> x_tran;                            // Translational states [position; velocity]
  Matrix<double, 3, 1> x_tran_pos;                        // Translational position
  Matrix<double, 3, 1> x_tran_vel;                        // Translational velocity
  Matrix<double, 3, 1> e_tran_pos_ref;                    // Translational reference model error
  Matrix<double, 6, 1> e_tran;                            // Translational error in states [e_pos; e_vel]
  Matrix<double, 3, 1> e_tran_pos;                        // Translational error in position
  Matrix<double, 3, 1> e_tran_vel;                        // Translational error in velocity
  Matrix<double, 6, 3> K_hat_x_tran_dot;				   // Adaptive gain to be integrated
  Matrix<double, 3, 3> K_hat_r_tran_dot;				   // Adaptive gain to be integrated
  Matrix<double, 30, 3> Theta_hat_tran_dot;			   // Adaptive gain to be integrated
  Matrix<double, 27, 1> outer_loop_regressor;			   // Outer loop regressor
  Matrix<double, 30, 1> augmented_outer_loop_regressor;   // Outer loop augmented regressor
  double dead_zone_value_translational;						   // Dead zone val - OL
  bool proj_op_activated_K_hat_x_translational;				   // Projection activation boolean - OL - K_hat_x
  bool proj_op_activated_K_hat_r_translational;				   // Projection activation boolean - OL - K_hat_r
  bool proj_op_activated_Theta_hat_translational;				   // Projection activation boolean - OL - Theta_hat
  Matrix<double, 3, 1> mu_tran_baseline;                  // Baseline control input
  Matrix<double, 3, 1> mu_tran_adaptive;                  // Adaptive control input
  Matrix<double, 3, 1> mu_tran_I;                         // Virtual control action in the inertial frame
  Matrix<double, 3, 1> mu_tran_I_dot;                     // Rate of virtual control action in the inertial frame
  Matrix<double, 3, 1> mu_tran_J;                         // Virtual control action in the body frame

  Matrix<double, 3, 3> Rji;                               // Rotation matrix from the body to the inertial frame
  Matrix<double, 3, 3> Rij;                               // Rotation matrix from the inertial to the body frame
  Matrix<double, 3, 1> c1;                                // Desired "heading" vector in the inertial frame
  Matrix<double, 3, 1> b3d;                               // Desired body z axis
  Matrix<double, 3, 1> b2d;                               // Desired body y axis
  Matrix<double, 3, 1> b1d;                               // Desired body x axis
  Matrix<double, 3, 3> R_d;                               // Desired rotation matrix

  Matrix<double, 3, 1> c1_dot;                            // Desired "heading rate" vector in the inertial frame
  Matrix<double, 3, 1> n_hat;                             // Intermediate variable to compute b2d_dot
  Matrix<double, 3, 1> n_hat_dot;                         // Intermediate variable to compute b2d_dot
  double sigma;                                                  // Intermediate variable to compute b2d_dot
  Matrix<double, 3, 1> b3d_dot;                           // Differential of Desired body z axis
  Matrix<double, 3, 1> b2d_dot;                           // Differential of Desired body y axis
  Matrix<double, 3, 1> b1d_dot;                           // Differential of Desired body x axis
  Matrix<double, 3, 3> R_d_dot;                           // Differential of Desired rotation matrix

  Matrix<double, 2, 1> internal_state_mu_x_filter;        // Internal States for filter
  Matrix<double, 2, 1> internal_state_mu_y_filter;        // Internal States for filter
  Matrix<double, 2, 1> internal_state_mu_z_filter;        // Internal States for filter
  Matrix<double, 2, 1> internal_state_omega_x_d_filter;   // Internal States for filter
  Matrix<double, 2, 1> internal_state_omega_y_d_filter;   // Internal States for filter
  Matrix<double, 2, 1> internal_state_omega_z_d_filter;   // Internal States for filter


  Matrix<double, 3, 1> omega_d_in_K;                      // Desired angular velocity in the desired frame
  Matrix<double, 3, 1> omega_d;                           // Desired angular velocity 
  Matrix<double, 3, 1> alpha_d;                           // Desired angular acceleration

  Quaterniond q;                                          // Quaternion
  Matrix<double, 3, 1> omega;                             // Angular velocity
  Matrix<double, 3, 1> omega_cmd;                         // Angular velocity reference command
  Matrix<double, 3, 1> omega_ref_dot;                     // Angular velocity reference model
  Matrix<double, 3, 1> e_omega_ref;                       // Angular velocity reference model error
  Matrix<double, 3, 1> omega_e;                           // Error in the angular velocities
  Matrix<double, 3, 1> Xi_e;                              // Error in the attitude states
  Matrix<double, 9, 1> inner_loop_regressor;			   // Inner loop regressor vector
  Matrix<double, 12, 1> augmented_inner_loop_regressor;   // Inner loop augmented regressor vector    
  Matrix<double, 3, 1> tau_rot_baseline;                  // Baseline rotational control input 
  Matrix<double, 3, 1> tau_rot_adaptive;                  // Adaptive rotational control input 
  Matrix<double, 3, 1> tau_rot;                           // Rotational Control action
  Matrix<double, 3, 3> K_hat_x_rot_dot;				   // Adaptive gain to be integrated
  Matrix<double, 3, 3> K_hat_r_rot_dot;				   // Adaptive gain to be integrated
  Matrix<double, 12, 3> Theta_hat_rot_dot;				   // Adaptive gain to be integrated
  double dead_zone_value_rotational;							   // Dead zone val - IL
  bool proj_op_activated_K_hat_x_rotational;					   // Projection activation boolean - IL - K_hat_x
  bool proj_op_activated_K_hat_r_rotational;					   // Projection activation boolean - IL - K_hat_r
  bool proj_op_activated_Theta_hat_rotational;				   // Projection activation boolean - IL - Theta_hat

  Matrix<double, 4, 1> u;                                 // [thrust; mx; my; mz]
  Matrix<double, 4, 1> Thrust;                            // Newtons
  Matrix<double, 4, 1> Sat_Thrust;                        // Saturated thrust 
  double alg_duration;                                           // Control execution duration
  std::chrono::high_resolution_clock::time_point alg_start_time; // Algorithm Start timepoint
  std::chrono::high_resolution_clock::time_point alg_end_time;   // Algorithm End timepoint
};





} // namespace _mrac_geometric_
} // namespace _quadm_

#endif // CONTROLLERS_MRAC_GEOMETRIC_TD_MEMBERS_HPP_