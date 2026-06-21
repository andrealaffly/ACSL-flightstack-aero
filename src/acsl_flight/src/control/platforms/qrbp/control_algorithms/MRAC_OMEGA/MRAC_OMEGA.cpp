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
 * File:        MRAC_OMEGA.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        August 07, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: MRAC with angular velocities for the QRBP. Inherts the
 *              class controller_base for the basic functionality that 
 *              is to be used for all control algorithms.
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

#include "MRAC_OMEGA.hpp"

namespace _qrbp_{
namespace _mrac_omega_{

// Concstructor - Take care to initilaize the logger
mrac_omega::mrac_omega(flight_params* p, const std::string & controller_log_dir_) : 
						controller_base(p), ud(p), logger(&cim, &csm, &control_input, controller_log_dir_) {

	// Reading in the paramters
	read_params("./src/acsl_flight/params/control_algorithms/qrbp/MRAC_OMEGA/gains_MRAC_OMEGA.json");

	// Initial Conditions 
	init();
	
}

// Destructor 
mrac_omega::~mrac_omega() {
	// Destructor implementation
}

// Implementing virtual functions from controller_base
void mrac_omega::read_params(const std::string& jsonFile) {
	
	// Create the file stream
	std::ifstream file(jsonFile);
	nlohmann::json j;
	file >> j;

	// Translational Parameters
	cip.Kp_refmod_tran = jsonToScaledMatrix<double, 3, 3>(j["TRANSLATIONAL"]["KP_refmod_translational"]);
	cip.Ki_refmod_tran = jsonToScaledMatrix<double, 3, 3>(j["TRANSLATIONAL"]["KI_refmod_translational"]);
	cip.Kd_refmod_tran = jsonToScaledMatrix<double, 3, 3>(j["TRANSLATIONAL"]["KD_refmod_translational"]);
	cip.Kp_tran = jsonToScaledMatrix<double, 3, 3>(j["TRANSLATIONAL"]["KP_translational"]);
	cip.Ki_tran = jsonToScaledMatrix<double, 3, 3>(j["TRANSLATIONAL"]["KI_translational"]);
	cip.Kd_tran = jsonToScaledMatrix<double, 3, 3>(j["TRANSLATIONAL"]["KD_translational"]);
	cip.Gamma_x_tran = jsonToScaledMatrix<double, 6, 6>(j["TRANSLATIONAL"]["Gamma_x_translational"]);
	cip.Gamma_r_tran = jsonToScaledMatrix<double, 3, 3>(j["TRANSLATIONAL"]["Gamma_r_translational"]);
	cip.Gamma_Theta_tran = jsonToScaledMatrix<double, 30, 30>(j["TRANSLATIONAL"]["Gamma_Theta_translational"]);
	cip.Q_tran = jsonToScaledMatrix<double, 6, 6>(j["TRANSLATIONAL"]["Q_translational"]);

	// Rotational Parameters
	cip.Kp_omega_ref = jsonToScaledMatrix<double, 3, 3>(j["ROTATIONAL"]["KP_omega_ref"]);
	cip.Ki_omega_ref = jsonToScaledMatrix<double, 3, 3>(j["ROTATIONAL"]["KI_omega_ref"]);
	cip.Kp_rot = jsonToScaledMatrix<double, 3, 3>(j["ROTATIONAL"]["KP_rotational"]);	
	cip.Ki_rot = jsonToScaledMatrix<double, 3, 3>(j["ROTATIONAL"]["KI_rotational"]);	
	cip.Kd_rot = jsonToScaledMatrix<double, 3, 3>(j["ROTATIONAL"]["KD_rotational"]);	
	cip.Gamma_x_rot = jsonToScaledMatrix<double, 3, 3>(j["ROTATIONAL"]["Gamma_x_rotational"]);
	cip.Gamma_r_rot = jsonToScaledMatrix<double, 3, 3>(j["ROTATIONAL"]["Gamma_r_rotational"]);
	cip.Gamma_Theta_rot = jsonToScaledMatrix<double, 12, 12>(j["ROTATIONAL"]["Gamma_Theta_rotational"]);
	cip.Q_rot = jsonToScaledMatrix<double, 3, 3>(j["ROTATIONAL"]["Q_rotational"]);

	// Robustification Parameters
	cip.dead_zone_delta_translational = j["ROBUSTIFICATION"]["dead_zone_delta_translational"];
	cip.dead_zone_e0_translational = j["ROBUSTIFICATION"]["dead_zone_e0_translational"];
	cip.dead_zone_delta_rotational = j["ROBUSTIFICATION"]["dead_zone_delta_rotational"];
	cip.dead_zone_e0_rotational = j["ROBUSTIFICATION"]["dead_zone_e0_rotational"];

	cip.sigma_x_translational = j["ROBUSTIFICATION"]["sigma_x_translational"];
	cip.sigma_r_translational = j["ROBUSTIFICATION"]["sigma_r_translational"];
	cip.sigma_Theta_translational = j["ROBUSTIFICATION"]["sigma_Theta_translational"];
	cip.sigma_x_rotational = j["ROBUSTIFICATION"]["sigma_x_rotational"];
	cip.sigma_r_rotational = j["ROBUSTIFICATION"]["sigma_r_rotational"];
	cip.sigma_Theta_rotational = j["ROBUSTIFICATION"]["sigma_Theta_rotational"];

	cip.projection_x_max_x_translational = j["ROBUSTIFICATION"]["projection_x_max_x_translational"];
	cip.projection_epsilon_x_translational = j["ROBUSTIFICATION"]["projection_epsilon_x_translational"];

	cip.projection_x_max_r_translational = j["ROBUSTIFICATION"]["projection_x_max_r_translational"];
	cip.projection_epsilon_r_translational = j["ROBUSTIFICATION"]["projection_epsilon_r_translational"];

	cip.projection_x_max_Theta_translational = j["ROBUSTIFICATION"]["projection_x_max_Theta_translational"];
	cip.projection_epsilon_Theta_translational = j["ROBUSTIFICATION"]["projection_epsilon_Theta_translational"];

	cip.projection_x_max_x_rotational = j["ROBUSTIFICATION"]["projection_x_max_x_rotational"]; 
	cip.projection_epsilon_x_rotational = j["ROBUSTIFICATION"]["projection_epsilon_x_rotational"];

	cip.projection_x_max_r_rotational = j["ROBUSTIFICATION"]["projection_x_max_r_rotational"];
	cip.projection_epsilon_r_rotational = j["ROBUSTIFICATION"]["projection_epsilon_r_rotational"];

	cip.projection_x_max_Theta_rotational = j["ROBUSTIFICATION"]["projection_x_max_Theta_rotational"];
	cip.projection_epsilon_Theta_rotational = j["ROBUSTIFICATION"]["projection_epsilon_Theta_rotational"];	

}

// Implementing virtual functions from controller_base
void mrac_omega::init() {
	// Initial conditions for rk4 integrator
	y.fill(0.0);
	dy.fill(0.0);

	/// Translational - Initialize matrices
	// Initialize to zero the 6x6 matrix and set the top-right 3x3 block as follows
	initMat(cip.A_tran);
	cip.A_tran.block<3, 3>(0, 3) = Matrix3d::Identity();

	// Initialize to zero the 6x3 matrix and set the bottom 3x3 block as follows
	initMat(cip.B_tran);
	cip.B_tran.block<3, 3>(3, 0) = Matrix3d::Identity();

	// Initialize to zero the 6x6 matrix
	initMat(cip.A_ref_tran);
	// Set the top-right 3x3 block as follows
	cip.A_ref_tran.block<3, 3>(0, 3) = Matrix3d::Identity();
	// Set the bottom-left 3x3 block as follows
	cip.A_ref_tran.block<3, 3>(3, 0) = -cip.Kp_refmod_tran;
	// Set the bottom-right 3x3 block as follows
	cip.A_ref_tran.block<3, 3>(3, 3) = -cip.Kd_refmod_tran;

	// Initialize to zero the 6x3 matrix and set the bottom 3x3 block as follows
	initMat(cip.B_ref_tran);
	cip.B_ref_tran.block<3, 3>(3, 0) = (1.0 / MASS) * Matrix3d::Identity();

	// Solve the continuous Lyapunov equation to compute P_translational
	cip.P_tran = RealContinuousLyapunovEquation(cip.A_ref_tran, cip.Q_tran);

	/// Rotational - Initialize matrices
	// Intialize to zero the 3x3 matrix
	initMat(cip.A_rot);

	// Initlize to identity the 3x3 matrix 
	cip.B_rot = Matrix3d::Identity();
	
	// Set the 3x3 matrix as follows
	cip.A_ref_rot << -cip.Kp_omega_ref;

	// Set the 3x3 matrix as follows
	cip.B_ref_rot = Matrix3d::Identity();

	// Solve the continuous Lyapunov eequation to compute P_rotational
	cip.P_rot = RealContinuousLyapunovEquation(cip.A_ref_rot, cip.Q_rot);
}

// Function to update all the states
void mrac_omega::update(double time,
						double x,
						double y,
						double z,
						double vx,
						double vy,
						double vz,
						double q0,
						double q1,
						double q2,
						double q3,
						double roll,
						double pitch,
						double yaw,
						double w_x,
						double w_y,
						double w_z)
{
	// 1. Assign all the states ----------------------------------------------------
	cim.t = time;

	// Assign the Translational States
	cim.x_tran.head<3>() << x,y,z;
	cim.x_tran.tail<3>() << vx,vy,vz;

	// Assign the Rotational States
	cim.eta_rot << roll,pitch,yaw;
	cim.omega_rot << w_x, w_y, w_z;

	// Assign the quaternions
	cim.q.w() = q0;
	cim.q.z() = q1;
	cim.q.y() = q2;
	cim.q.z() = q3;

	// 2. Get the reference trajectory ---------------------------------------------
	ud.updateUserDefinedTrajectory(time);
	cim.r_user = ud.getUserDefinedPosition();
	cim.r_dot_user = ud.getUserDefinedVelocity();
	cim.r_ddot_user = ud.getUserDefinedAcceleration();  

	// 3. Capture the time before the execution of the controller ------------------
	cim.alg_start_time = std::chrono::high_resolution_clock::now();

	// 4. Compute the rotation matrices
	cim.R_J_I = rotationMatrix321LocalToGlobal(cim.eta_rot(0), cim.eta_rot(1), cim.eta_rot(2));
	cim.R_I_J = rotationMatrix321GlobalToLocal(cim.eta_rot(0), cim.eta_rot(1), cim.eta_rot(2));

	// 5. Compute the body velocities from the inertial velocities
	cim.x_tran_vel_J = cim.R_I_J * cim.x_tran.tail<3>();

	// 6. Compute the square of the norm of velocity in the body frame and the aerodynamic angles
	cim.aero.states = computeAeroStatesNACA0012Quad(cim.x_tran_vel_J(0), cim.x_tran_vel_J(1), cim.x_tran_vel_J(2),
													cim.omega_rot(0), cim.omega_rot(1), cim.omega_rot(2));

	// 7. Compute the Direction Cosine Matrix at the center of gravity
	cim.R_W_J = aeroDCMQuadNED(cim.aero.states.alpha, cim.aero.states.beta);

	// 8. Compute the aerodynamic coefficients
	cim.aero.coeff = compute_aero_coefficients(cim.aero.states);

	// 9. Assign the values from the integrator -------------------------------------
  assign_from_rk4();

}

// Function to assign elements from the rk4 integrator
void mrac_omega::assign_from_rk4()
{
	//------------ assign after integration  ------------//
	int index = 0;
	assignElementsToMembers(csm.state_roll_d_filter, y, index);
	assignElementsToMembers(csm.state_pitch_d_filter, y, index);
	assignElementsToMembers(csm.state_omega_x_d_filter, y, index);
	assignElementsToMembers(csm.state_omega_y_d_filter, y, index);
	assignElementsToMembers(csm.state_omega_z_d_filter, y, index);
	assignElementsToMembers(csm.e_tran_pos_I, y, index);
	assignElementsToMembers(csm.e_tran_pos_ref_I, y, index);
	assignElementsToMembers(csm.x_tran_ref, y, index);
	assignElementsToMembers(csm.K_hat_x_tran, y, index);
	assignElementsToMembers(csm.K_hat_r_tran, y, index);
	assignElementsToMembers(csm.Theta_hat_tran, y, index);
	assignElementsToMembers(csm.e_rot_omega_ref_I, y, index);
	assignElementsToMembers(csm.e_rot_eta_I, y, index);
	assignElementsToMembers(csm.omega_rot_ref, y, index);
	assignElementsToMembers(csm.K_hat_x_rot, y, index);
	assignElementsToMembers(csm.K_hat_r_rot, y, index);
	assignElementsToMembers(csm.Theta_hat_rot, y, index);
}


// Model function for integration
void mrac_omega::model([[maybe_unused]] const rk4_array<double, NSI> &y, rk4_array<double, NSI> &dy, [[maybe_unused]] const double t)
{
	//------------ Fill up the dy for integration  ------------//
	int index = 0;
	assignElementsToDxdt(cim.internal_state_roll_d_filter, dy, index);
	assignElementsToDxdt(cim.internal_state_pitch_d_filter, dy, index);
	assignElementsToDxdt(cim.internal_state_omega_x_d_filter, dy, index);
	assignElementsToDxdt(cim.internal_state_omega_y_d_filter, dy, index);
	assignElementsToDxdt(cim.internal_state_omega_z_d_filter, dy, index);
	assignElementsToDxdt(cim.e_tran_pos, dy, index);
	assignElementsToDxdt(cim.e_tran_pos_ref, dy, index);
	assignElementsToDxdt(cim.x_tran_ref_dot, dy, index);
	assignElementsToDxdt(cim.K_hat_x_tran_dot, dy, index);
	assignElementsToDxdt(cim.K_hat_r_tran_dot, dy, index);
	assignElementsToDxdt(cim.Theta_hat_tran_dot, dy, index);
	assignElementsToDxdt(cim.e_rot_omega_ref, dy, index);
	assignElementsToDxdt(cim.e_rot_eta, dy, index);
	assignElementsToDxdt(cim.omega_rot_ref_dot, dy, index);
	assignElementsToDxdt(cim.K_hat_x_rot_dot, dy, index);
	assignElementsToDxdt(cim.K_hat_r_rot_dot, dy, index);
	assignElementsToDxdt(cim.Theta_hat_rot_dot, dy, index);
}

// Function to compute the outerloop regressor vector
void mrac_omega::compute_outer_loop_regressor()
{
    // cache the variables
    double sph = sin(cim.eta_rot(0));
    double cph = cos(cim.eta_rot(0));
    double sth = sin(cim.eta_rot(1));
    double cth = cos(cim.eta_rot(1));
    double sps = sin(cim.eta_rot(2));
    double cps = cos(cim.eta_rot(2));
    double sa = sin(cim.aero.states.alpha);
    double ca = cos(cim.aero.states.alpha);
    double sb = sin(cim.aero.states.beta);
    double cb = cos(cim.aero.states.beta);
    double v_norm_sq = cim.aero.states.v_norm_sq;

    cim.outer_loop_regressor << -v_norm_sq*(sb*(cph*sps - cps*sph*sth) - ca*cb*(sph*sps + cph*cps*sth) + cb*cps*sa*cth),
                                -v_norm_sq*(ca*cb*(cps*sph - cph*sps*sth) - sb*(cph*cps + sph*sps*sth) + cb*sa*cth*sps),
                                                                     v_norm_sq*(cb*sa*sth + sb*cth*sph + ca*cb*cph*cth),
                                -v_norm_sq*(cb*(cph*sps - cps*sph*sth) + ca*sb*(sph*sps + cph*cps*sth) - cb*cps*sa*cth),
                                 v_norm_sq*(cb*(cph*cps + sph*sps*sth) + ca*sb*(cps*sph - cph*sps*sth) + cb*sa*cth*sps),
                                                                    -v_norm_sq*(cb*sa*sth - cb*cth*sph + ca*cph*sb*cth),
                                                                   -v_norm_sq*(sa*(sph*sps + cph*cps*sth) + ca*cps*cth),
                                                                    v_norm_sq*(sa*(cps*sph - cph*sps*sth) - ca*cth*sps),
                                                                                        v_norm_sq*(ca*sth - cph*sa*cth),
                                -v_norm_sq*(sb*(cph*sps - cps*sph*sth) - ca*cb*(sph*sps + cph*cps*sth) + cb*cps*sa*cth),
                                -v_norm_sq*(ca*cb*(cps*sph - cph*sps*sth) - sb*(cph*cps + sph*sps*sth) + cb*sa*cth*sps),
                                                                     v_norm_sq*(cb*sa*sth + sb*cth*sph + ca*cb*cph*cth),
                                -v_norm_sq*(cb*(cph*sps - cps*sph*sth) + ca*sb*(sph*sps + cph*cps*sth) - cb*cps*sa*cth),
                                 v_norm_sq*(cb*(cph*cps + sph*sps*sth) + ca*sb*(cps*sph - cph*sps*sth) + cb*sa*cth*sps),
                                                                    -v_norm_sq*(cb*sa*sth - cb*cth*sph + ca*cph*sb*cth),
                                                                   -v_norm_sq*(sa*(sph*sps + cph*cps*sth) + ca*cps*cth),
                                                                    v_norm_sq*(sa*(cps*sph - cph*sps*sth) - ca*cth*sps),
                                                                                        v_norm_sq*(ca*sth - cph*sa*cth),
                                -v_norm_sq*(sb*(cph*sps - cps*sph*sth) - ca*cb*(sph*sps + cph*cps*sth) + cb*cps*sa*cth),
                                -v_norm_sq*(ca*cb*(cps*sph - cph*sps*sth) - sb*(cph*cps + sph*sps*sth) + cb*sa*cth*sps),
                                                                     v_norm_sq*(cb*sa*sth + sb*cth*sph + ca*cb*cph*cth),
                                -v_norm_sq*(cb*(cph*sps - cps*sph*sth) + ca*sb*(sph*sps + cph*cps*sth) - cb*cps*sa*cth),
                                 v_norm_sq*(cb*(cph*cps + sph*sps*sth) + ca*sb*(cps*sph - cph*sps*sth) + cb*sa*cth*sps),
                                                                    -v_norm_sq*(cb*sa*sth - cb*cth*sph + ca*cph*sb*cth),
                                                                   -v_norm_sq*(sa*(sph*sps + cph*cps*sth) + ca*cps*cth),
                                                                    v_norm_sq*(sa*(cps*sph - cph*sps*sth) - ca*cth*sps),
                                                                                        v_norm_sq*(ca*sth - cph*sa*cth);
}

// Function to compute the inner loop regressor vector
void mrac_omega::compute_innner_loop_regressor()
{
    // cache the variables
    double sa = sin(cim.aero.states.alpha);
    double ca = cos(cim.aero.states.alpha);
    double sb = sin(cim.aero.states.beta);
    double cb = cos(cim.aero.states.beta);
    double wx = cim.omega_rot(0);
    double wy = cim.omega_rot(1);
    double wz = cim.omega_rot(2);

    cim.inner_loop_regressor << wx*wy,
                                wz*wx,
                                wy*wx,
                                cim.aero.states.v_norm_sq,
                                cim.aero.states.v_norm_sq*ca,
                                cim.aero.states.v_norm_sq*sb,
                                cim.aero.states.v_norm_sq*cb,
                                cim.aero.states.v_norm_sq*sa*sb,
                                cim.aero.states.v_norm_sq*sa*cb;
}

// Function to compute the outerloop control in I
void mrac_omega::compute_translational_control_in_I()
{
	// Compute the error in the states
	cim.e_tran << cim.x_tran - csm.x_tran_ref;
	cim.e_tran_pos << cim.e_tran.head<3>();
	cim.e_tran_vel << cim.e_tran.tail<3>();

	// Compute the translational position error between the reference model and the user defined trajectory
	cim.e_tran_pos_ref << csm.x_tran_ref.head<3>() - cim.r_user;

	// Compute the reference command input [reference model - user_defined_trajectory]
	cim.r_cmd_tran << MASS * (- cip.Ki_refmod_tran * csm.e_tran_pos_ref_I		// Integral term
							  + cip.Kp_refmod_tran * cim.r_user													// Proportional term
							  + cip.Kd_refmod_tran * cim.r_dot_user											// Derivative term
							  + cim.r_ddot_user);																				// Feedforward term

	// Reference model
	cim.x_tran_ref_dot << cip.A_ref_tran * csm.x_tran_ref
											+ cip.B_ref_tran * cim.r_cmd_tran;

	// Compute the baseline control input
	cim.mu_tran_baseline << MASS * (- cip.Kp_tran * cim.e_tran_pos				// Proportional term
									- cip.Kd_tran * cim.e_tran_vel				                // Derivative term
									- cip.Ki_tran * csm.e_tran_pos_I			                // Integral term
									+ cim.x_tran_ref_dot.tail<3>()				                // Feedforward term
									- G * e3_basis)								                        // Weight inversion term
									- cim.aero.dyn.outer_loop_dynamic_inv;		            // Aerodynamic inverstion term

	// Compute the augemented regressor vector
	cim.augmented_outer_loop_regressor << cim.mu_tran_baseline,
										  									cim.outer_loop_regressor;

	// Cache the transpose of the tracking error * P * B
	Matrix<double, 1, 3> e_transpose_p_b = cim.e_tran.transpose() * cip.P_tran * cip.B_tran;

	// Computing the scalar value output from the dead-zone modification modulation function
	cim.dead_zone_value_translational = deadZoneModulationFunction(cim.e_tran.transpose(),
																																 cip.dead_zone_delta_translational,
																																 cip.dead_zone_e0_translational);

	// Adaptive laws
	cim.K_hat_x_tran_dot = AdaptiveLawDeadZoneEMod(-cip.Gamma_x_tran,
																								 	cim.dead_zone_value_translational,
																								 	cim.x_tran,
																								 	e_transpose_p_b,
																								 	cip.sigma_x_translational,
																								 	csm.K_hat_x_tran);

	cim.K_hat_r_tran_dot = AdaptiveLawDeadZoneEMod(-cip.Gamma_r_tran,
																									cim.dead_zone_value_translational,
																									cim.r_cmd_tran,
																									e_transpose_p_b,
																									cip.sigma_r_translational,
																									csm.K_hat_r_tran);

	cim.Theta_hat_tran_dot = AdaptiveLawDeadZoneEMod(cip.Gamma_Theta_tran,
																									 cim.dead_zone_value_translational,
																									 cim.augmented_outer_loop_regressor,
																									 e_transpose_p_b,
																									 cip.sigma_Theta_translational,
																									 csm.Theta_hat_tran);

	// Projection operator - Ball
	if constexpr (_config_param_::USE_PROJECTION_OPERATOR)
	{
		using namespace _projection_operator_;

		// Projection operator K_hat_x
		MatrixProjectionOutput<decltype(csm.K_hat_x_tran)> proj_op_output_K_hat_x_translational = 
			_ball_::projectionMatrix(csm.K_hat_x_tran, 
															 cim.K_hat_x_tran_dot,
															 cip.projection_x_max_x_translational,
															 cip.projection_epsilon_x_translational);

		cim.K_hat_x_tran_dot = proj_op_output_K_hat_x_translational.projected_matrix;
		cim.proj_op_activated_K_hat_x_translational = proj_op_output_K_hat_x_translational.projection_operator_activated;
		
		// Projection operator K_hat_r
		MatrixProjectionOutput<decltype(csm.K_hat_r_tran)> proj_op_output_K_hat_r_translational = 
			_ball_::projectionMatrix(csm.K_hat_r_tran,
															 cim.K_hat_r_tran_dot,
															 cip.projection_x_max_r_translational,
															 cip.projection_epsilon_r_translational);

		cim.K_hat_r_tran_dot = proj_op_output_K_hat_r_translational.projected_matrix;
		cim.proj_op_activated_K_hat_r_translational = proj_op_output_K_hat_r_translational.projection_operator_activated;															 

		// Projection operator Theta_hat
		MatrixProjectionOutput<decltype(csm.Theta_hat_tran)> proj_op_output_Theta_hat_translational =
			_ball_::projectionMatrix(csm.Theta_hat_tran,
															 cim.Theta_hat_tran_dot,
															 cip.projection_x_max_Theta_translational,
															 cip.projection_epsilon_Theta_translational);

		cim.Theta_hat_tran_dot = proj_op_output_Theta_hat_translational.projected_matrix;
		cim.proj_op_activated_Theta_hat_translational = proj_op_output_Theta_hat_translational.projection_operator_activated;

	}

	// Adaptive control law
	cim.mu_tran_adaptive << csm.K_hat_x_tran.transpose() * cim.x_tran
												+ csm.K_hat_r_tran.transpose() * cim.r_cmd_tran
												- csm.Theta_hat_tran.transpose() * cim.augmented_outer_loop_regressor;

	// Compute the total baseline + adaptive control
	cim.mu_tran_I << cim.mu_tran_baseline + cim.mu_tran_adaptive;

}

// Compute the orientation commands and the desired total thrust
void mrac_omega::compute_u1_eta_d()
{
	// Compute the virtual forces in J
	cim.mu_tran_J << cim.R_I_J * cim.mu_tran_I;

	// Compute u1 - Total Thrust
	cim.u(0) = sqrt(  pow(cim.mu_tran_J(0), 2)
							 		+ pow(cim.mu_tran_J(1), 2)
								 	+ pow(cim.mu_tran_J(2), 2));

	// Retrieve the desired yaw and yaw rate
	cim.eta_rot_d(2) = ud.getUserDefinedYaw();
	cim.eta_rot_rate_d(2) = ud.getUserDefinedYawRate();									

	// Compute the desired roll
	double r1 = cim.mu_tran_J(1) / cim.u(0);
	cim.eta_rot_d(0) = atan2(r1, sqrt(1 - pow(r1,2)));

	// Compute the desired pitch
	cim.eta_rot_d(1) = atan2( -cim.mu_tran_J(0), -cim.mu_tran_J(2) );

	// If true, the desired roll,pitch,yaw states will follow a sine wave.
	// Used to tune the inner loop reference model OFFLINE!
	if constexpr (_config_param_::USE_SINEWAVE_INL_CMD)
	{
		_test_functions_::generateSineWave(cim.t - flight_params_ptr->start_time, 
											cim.eta_rot_d(0), cim.eta_rot_d(1), cim.eta_rot_d(2), cim.eta_rot_rate_d(2));

		// Spoof the controller into thinking it is following the angluar orientation commands perfectly
		cim.eta_rot(0) = cim.eta_rot_d(0);
		cim.eta_rot(1) = cim.eta_rot_d(1);
		cim.eta_rot(2) = cim.eta_rot_d(2);

		FLIGHTSTACK_INFO_STREAM("SPOOFING ORIENTATION");
	}

	// Compute the internal state for angular rates
	cim.internal_state_roll_d_filter << A_filter_roll_ref * csm.state_roll_d_filter
                                      + B_filter_roll_ref * cim.eta_rot_d(0);

	cim.internal_state_pitch_d_filter << A_filter_pitch_ref * csm.state_pitch_d_filter
																			+ B_filter_pitch_ref * cim.eta_rot_d(1);


	// Compute the desired roll and pitch rates
	cim.eta_rot_rate_d(0) = C_filter_roll_ref * csm.state_roll_d_filter;    
	cim.eta_rot_rate_d(1) = C_filter_pitch_ref * csm.state_pitch_d_filter;	

	// Compute the Jacobian with the current orientation
	cim.Jacobian = jacobianMatrix(cim.eta_rot(0), cim.eta_rot(1));

	// Compute the desired angular velocities
	cim.omega_rot_d << cim.Jacobian * cim.eta_rot_rate_d;
	
	// Compute the internal state for angular accelration
	cim.internal_state_omega_x_d_filter << A_filter_roll_dot_ref * csm.state_omega_x_d_filter
																				+ B_filter_roll_dot_ref * cim.omega_rot_d(0);

	cim.internal_state_omega_y_d_filter << A_filter_pitch_dot_ref * csm.state_omega_y_d_filter
																				+ B_filter_pitch_dot_ref * cim.omega_rot_d(1);

	cim.internal_state_omega_z_d_filter << A_filter_yaw_dot_ref * csm.state_omega_z_d_filter
																				+ B_filter_yaw_dot_ref * cim.omega_rot_d(2);

	// Compute the desired angular acceleration
	cim.alpha_rot_d(0) = C_filter_roll_dot_ref * csm.state_omega_x_d_filter;
	cim.alpha_rot_d(1) = C_filter_pitch_dot_ref * csm.state_omega_y_d_filter;
	cim.alpha_rot_d(2) = C_filter_yaw_dot_ref * csm.state_omega_z_d_filter;	
}

// Compute the rotational control
void mrac_omega::compute_rotational_control()
{
	// Compute the error in the orientation [actual - desired]
	cim.e_rot_eta(0) = cim.eta_rot(0) - cim.eta_rot_d(0);
	cim.e_rot_eta(1) = cim.eta_rot(1) - cim.eta_rot_d(1);
	cim.e_rot_eta(2) = makeYawAngularErrorContinuous<double>(cim.eta_rot(2), cim.eta_rot_d(2));

	// Compute the error in the angular velocity
	cim.e_rot_omega << cim.omega_rot - csm.omega_rot_ref;

	// Compute the error between the angular velocity reference model and the desired angular velocity
	cim.e_rot_omega_ref << csm.omega_rot_ref - cim.omega_rot_d;

	// Compute the command angular velocity
	cim.omega_cmd << cip.Kp_omega_ref * cim.omega_rot_d - cip.Ki_omega_ref * csm.e_rot_omega_ref_I + cim.alpha_rot_d;

	// Compute the derivative of the reference angular velocity
	cim.omega_rot_ref_dot << cip.A_ref_rot * csm.omega_rot_ref + cip.B_ref_rot * cim.omega_cmd;

	// Baseline PID controller
	cim.tau_rot_baseline << inertia_matrix_q * ( -cip.Kp_rot * cim.e_rot_eta               // Proportional term
													-cip.Kd_rot * cim.e_rot_omega          						   					 // Derivative term
												 	-cip.Ki_rot * csm.e_rot_eta_I          						   					 // Integral term
												 	+ cim.omega_rot_ref_dot)                  					   				 // Feedforward term
													+ cim.omega_rot.cross(inertia_matrix_q * cim.omega_rot)        // Dynamic inversion term
													- cim.aero.dyn.inner_loop_dynamic_inv;                		   	 // Aero inversion term

	// Compute the augmented regressor vector
	cim.augmented_inner_loop_regressor << cim.tau_rot_baseline, cim.inner_loop_regressor;

	// Cache the tranpose of the tracking error * P * B
	Eigen::Matrix<double, 1, 3> e_transpose_p_b = cim.e_rot_omega.transpose() * cip.P_rot * cip.B_rot;

	// Computing the scalar value output from the dead-zone modification modulation function
	cim.dead_zone_value_rotational = deadZoneModulationFunction(cim.e_rot_omega.transpose(),
																															cip.dead_zone_delta_rotational,
																															cip.dead_zone_e0_rotational);

	// Adaptive laws
	cim.K_hat_x_rot_dot = AdaptiveLawDeadZoneEMod(-cip.Gamma_x_rot,
																								 cim.dead_zone_value_rotational,
																								 cim.omega_rot,
																								 e_transpose_p_b,
																								 cip.sigma_x_rotational,
																								 csm.K_hat_x_rot);

	cim.K_hat_r_rot_dot = AdaptiveLawDeadZoneEMod(-cip.Gamma_r_rot,
																								 cim.dead_zone_value_rotational,
																								 cim.omega_cmd,
																								 e_transpose_p_b,
																								 cip.sigma_r_rotational,
																								 csm.K_hat_r_rot);

	cim.Theta_hat_rot_dot = AdaptiveLawDeadZoneEMod(cip.Gamma_Theta_rot,
																								  cim.dead_zone_value_rotational,
																									cim.augmented_inner_loop_regressor,
																									e_transpose_p_b,
																									cip.sigma_Theta_rotational,
																									csm.Theta_hat_rot);	

	// Projection operator - Ball
	if constexpr (_config_param_::USE_PROJECTION_OPERATOR)
	{
		using namespace _projection_operator_;

		// Projection operator K_hat_x
		MatrixProjectionOutput<decltype(csm.K_hat_x_rot)> proj_op_output_K_hat_x_rotational = 
			_ball_::projectionMatrix(csm.K_hat_x_rot,
															 cim.K_hat_x_rot_dot,
															 cip.projection_x_max_x_rotational,
															 cip.projection_epsilon_x_rotational);

		cim.K_hat_x_rot_dot = proj_op_output_K_hat_x_rotational.projected_matrix;
		cim.proj_op_activated_K_hat_x_rotational = proj_op_output_K_hat_x_rotational.projection_operator_activated;

		// Projection operator K_hat_r
		MatrixProjectionOutput<decltype(csm.K_hat_r_rot)> proj_op_output_K_hat_r_rotational = 
			_ball_::projectionMatrix(csm.K_hat_r_rot,
															 cim.K_hat_r_rot_dot,
															 cip.projection_x_max_r_rotational,
															 cip.projection_epsilon_r_rotational);
						
		cim.K_hat_r_rot_dot = proj_op_output_K_hat_r_rotational.projected_matrix;
		cim.proj_op_activated_K_hat_r_rotational = proj_op_output_K_hat_r_rotational.projection_operator_activated;

		// Projection operator Theta_hat
		MatrixProjectionOutput<decltype(csm.Theta_hat_rot)> proj_op_output_Theta_hat_rotational = 
			_ball_::projectionMatrix(csm.Theta_hat_rot,
															 cim.Theta_hat_rot_dot,
															 cip.projection_x_max_Theta_rotational,
															 cip.projection_epsilon_Theta_rotational);

		cim.Theta_hat_rot_dot = proj_op_output_Theta_hat_rotational.projected_matrix;
		cim.proj_op_activated_Theta_hat_rotational = proj_op_output_Theta_hat_rotational.projection_operator_activated;
	}													

	// Adaptive control law
	cim.tau_rot_adaptive << csm.K_hat_x_rot.transpose() * cim.omega_rot
												+ csm.K_hat_r_rot.transpose() * cim.omega_cmd
												- csm.Theta_hat_rot.transpose() * cim.augmented_inner_loop_regressor;			

	// Total rotational control input
	cim.tau_rot << cim.tau_rot_baseline + cim.tau_rot_adaptive;							

	// Assign the control inputs
	cim.u(1) = cim.tau_rot(0);
	cim.u(2) = cim.tau_rot(1);
	cim.u(3) = cim.tau_rot(2);												
}

// Function to compute the normalized thrusts
void mrac_omega::compute_normalized_thrusts()
{
	// Compute the individual thrusts in Newtons
	cim.Thrust << mixer_matrix_qrbp * cim.u;

	// Saturate each element of the Thrust vector between MIN_THRUST and MAX_THRUST
	cim.Sat_Thrust = (cim.Thrust.cwiseMin(MAX_THRUST).cwiseMax(MIN_THRUST));

	// Compute the final control inputs
	control_input(0) = evaluatePolynomial(thrust_polynomial_coeff_qrbp, cim.Sat_Thrust(0));
	control_input(1) = evaluatePolynomial(thrust_polynomial_coeff_qrbp, cim.Sat_Thrust(1));
	control_input(2) = evaluatePolynomial(thrust_polynomial_coeff_qrbp, cim.Sat_Thrust(2));
	control_input(3) = evaluatePolynomial(thrust_polynomial_coeff_qrbp, cim.Sat_Thrust(3));
}

// Function to print to terminal during runtime
void mrac_omega::debug2terminal()
{
	// FLIGHTSTACK_INFO_STREAM_NO_TAG("Execution Time:", cim.alg_duration); 	
}

// Function that is called in control.cpp
void mrac_omega::run(const double time_step_rk4_) {

	// Process the dynamics --------------------------------------------------------
	// 1. Compute the aerodynamics 
	cim.aero.dyn = compute_aero_forces_moments(cim.aero.states, cim.aero.coeff, cim.R_J_I, cim.R_W_J);

	// 2. Compute the regressor vector for the outer loop
	compute_outer_loop_regressor();

	// 3. Compute the translational control input
	compute_translational_control_in_I();

	// 4. Compute the thrust needed and the desired angles
	compute_u1_eta_d();
   
	// 5. Compute the regressor vector for the inner loop
	compute_innner_loop_regressor();
	
	// 6. Compute the rotational control input
	compute_rotational_control();

	// 7. Compute the normalized thrust - Final Step
	compute_normalized_thrusts();

	// 8. Do the integration
	rk54.do_step(boost::bind(&mrac_omega::model, this, bph::_1, bph::_2, bph::_3),
							y, cim.t, time_step_rk4_);
	
	// Capture the time after the execution of the controller
	cim.alg_end_time = std::chrono::high_resolution_clock::now();
	
	// Calculate the duration of the execution 
	cim.alg_duration = std::chrono::duration_cast<std::chrono::microseconds>(
													cim.alg_end_time - cim.alg_start_time).count();

	// 9. Log the Data after all the calculataions
	logger.logLogData();

	// optional debug
	debug2terminal();
}

// Implementing getter functions from controller_base
float mrac_omega::get_t1() const {
	return control_input(0);
}

float mrac_omega::get_t2() const {
	return control_input(1);
}

float mrac_omega::get_t3() const {
	return control_input(2);
}

float mrac_omega::get_t4() const {
	return control_input(3);
}

float mrac_omega::get_t5() const {
	return control_input(4);
}

float mrac_omega::get_t6() const {
	return control_input(5);
}

float mrac_omega::get_t7() const {
	return control_input(6);
}

float mrac_omega::get_t8() const {
	return control_input(7);
}

} // namesapce _mrac_omega_
} // namespace _qrbp_