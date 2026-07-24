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
 * File:        MRAC_GEOMETRIC_TD.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        July 16, 2026
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: MRAC with SO(3) for the medium quadrotor. Inherts the
 *              class controller_base for the basic functionality that 
 *              is to be used for all control algorithms.
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

#include "MRAC_GEOMETRIC_TD.hpp"

namespace _quadm_{
namespace _mrac_geometric_{

// Concstructor - Take care to initilaize the logger
mrac_geometric::mrac_geometric(flight_params* p, const std::string & controller_log_dir_) : 
						                   controller_base(p), ud(p), logger(&cim, &csm, &control_input, controller_log_dir_) {

	// Reading in the paramters
	read_params("./src/acsl_flight/params/control_algorithms/quadm/MRAC_GEOMETRIC/gains_MRAC_GEOMETRIC.json");

	// Initial Conditions 
	init();
	
}

// Destructor 
mrac_geometric::~mrac_geometric() {
	// Destructor implementation
}

// Incrementally unwrap yaw by removing ±2*pi jumps in the difference.
double mrac_geometric::unwrapPsiSimple(double psi_wrapped, ::_quadm_::_mrac_geometric_::SimplePsiUnwrapState &state)
{
    constexpr double pi    = M_PI;
    constexpr double twoPi = 2.0 * M_PI;

    // First sample: just pass through
    if (!state.initialized) {
        state.prevUnwrapped = psi_wrapped;
        state.initialized   = true;
        return psi_wrapped;
    }

    double dp = psi_wrapped - state.prevUnwrapped;

    // Bring the increment into [-pi, pi]
    if (dp > pi) {
        dp -= twoPi;
    } else if (dp < -pi) {
        dp += twoPi;
    }

    double psi_unwrapped = state.prevUnwrapped + dp;
    state.prevUnwrapped  = psi_unwrapped;
    return psi_unwrapped;
}

// Implementing virtual functions from controller_base
void mrac_geometric::read_params(const std::string& jsonFile) {
	
	// Create the file stream
	std::ifstream file(jsonFile);
	nlohmann::json j;
	file >> j;

  // Controller matrices ----------------------------------------------------------------------------------------------
  // Translational parameters
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

  // Rotational parameters
  cip.Kp_omega_ref = jsonToScaledMatrix<double, 3, 3>(j["ROTATIONAL"]["KP_omega_ref"]);
  cip.Ki_omega_ref = jsonToScaledMatrix<double, 3, 3>(j["ROTATIONAL"]["KI_omega_ref"]);
  cip.Kp_att = jsonToScaledMatrix<double, 3, 3>(j["ROTATIONAL"]["KP_rotational"]);
  cip.Kd_att = jsonToScaledMatrix<double, 3, 3>(j["ROTATIONAL"]["KD_rotational"]);
  cip.Ka_att = jsonToScaledMatrix<double, 3, 1>(j["ROTATIONAL"]["Ka_rotational"]);
  cip.Gamma_x_rot = jsonToScaledMatrix<double, 3, 3>(j["ROTATIONAL"]["Gamma_x_rotational"]);
  cip.Gamma_r_rot = jsonToScaledMatrix<double, 3, 3>(j["ROTATIONAL"]["Gamma_r_rotational"]);
  cip.Gamma_Theta_rot = jsonToScaledMatrix<double, 12, 12>(j["ROTATIONAL"]["Gamma_Theta_rotational"]);
  cip.Q_rot = jsonToScaledMatrix<double, 3, 3>(j["ROTATIONAL"]["Q_rotational"]);

  // Differentiator matrices
  cip.A_filter_mu = jsonToScaledMatrix<double, 2, 2>(j["FILTERS"]["A_filter_mu"]);
  cip.B_filter_mu = jsonToScaledMatrix<double, 2, 1>(j["FILTERS"]["B_filter_mu"]);
  cip.C_filter_mu = jsonToScaledMatrix<double, 1, 2>(j["FILTERS"]["C_filter_mu"]);
  cip.A_filter_omega_d = jsonToScaledMatrix<double, 2, 2>(j["FILTERS"]["A_filter_omega_d"]);
  cip.B_filter_omega_d = jsonToScaledMatrix<double, 2, 1>(j["FILTERS"]["B_filter_omega_d"]);
  cip.C_filter_omega_d = jsonToScaledMatrix<double, 1, 2>(j["FILTERS"]["C_filter_omega_d"]);

  // Robustification Parameters
  cip.dead_zone_delta_translational = j["ROBUSTIFICATION"]["dead_zone_delta_translational"];
  cip.dead_zone_e0_translational = j["ROBUSTIFICATION"]["dead_zone_e0_translational"];
  cip.sigma_x_translational = j["ROBUSTIFICATION"]["sigma_x_translational"];
  cip.sigma_r_translational = j["ROBUSTIFICATION"]["sigma_r_translational"];
  cip.sigma_Theta_translational = j["ROBUSTIFICATION"]["sigma_Theta_translational"];
  cip.projection_x_max_x_translational = j["ROBUSTIFICATION"]["projection_x_max_x_translational"];
  cip.projection_epsilon_x_translational = j["ROBUSTIFICATION"]["projection_epsilon_x_translational"];
  cip.projection_x_max_r_translational = j["ROBUSTIFICATION"]["projection_x_max_r_translational"];
  cip.projection_epsilon_r_translational = j["ROBUSTIFICATION"]["projection_epsilon_r_translational"];
  cip.projection_x_max_Theta_translational = j["ROBUSTIFICATION"]["projection_x_max_Theta_translational"];
  cip.projection_epsilon_Theta_translational = j["ROBUSTIFICATION"]["projection_epsilon_Theta_translational"];

  cip.dead_zone_delta_rotational = j["ROBUSTIFICATION"]["dead_zone_delta_rotational"];
  cip.dead_zone_e0_rotational = j["ROBUSTIFICATION"]["dead_zone_e0_rotational"];
  cip.sigma_x_rotational = j["ROBUSTIFICATION"]["sigma_x_rotational"];
  cip.sigma_r_rotational = j["ROBUSTIFICATION"]["sigma_r_rotational"];
  cip.sigma_Theta_rotational = j["ROBUSTIFICATION"]["sigma_Theta_rotational"];
  cip.projection_x_max_x_rotational = j["ROBUSTIFICATION"]["projection_x_max_x_rotational"]; 
  cip.projection_epsilon_x_rotational = j["ROBUSTIFICATION"]["projection_epsilon_x_rotational"];
  cip.projection_x_max_r_rotational = j["ROBUSTIFICATION"]["projection_x_max_r_rotational"];
  cip.projection_epsilon_r_rotational = j["ROBUSTIFICATION"]["projection_epsilon_r_rotational"];
  cip.projection_x_max_Theta_rotational = j["ROBUSTIFICATION"]["projection_x_max_Theta_rotational"];
  cip.projection_epsilon_Theta_rotational = j["ROBUSTIFICATION"]["projection_epsilon_Theta_rotational"];

}

// Implementing virtual functions from controller_base
void mrac_geometric::init() {

  // Set the inital conditions
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
void mrac_geometric::update(double time,
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
  cim.x_tran_pos << x,y,z;
  cim.x_tran_vel << vx, vy, vz;
  cim.x_tran << cim.x_tran_pos,
                cim.x_tran_vel;

  // Assign the quaternions and ensure it is normalized
  // q0 = w, q1 = x, q2 = y, q3 = z
  cim.q = Quaterniond(q0, q1, q2, q3).normalized();

  // Assign the rotation matrices
  cim.Rji = cim.q.toRotationMatrix(); // rotation from the body to the inertial frame
  cim.Rij = cim.Rji.transpose();      // rotation from the inertial to the body frame

  // Asign the angular velocity
  cim.omega << w_x, w_y, w_z;

  // 2. Get the reference trajectory ---------------------------------------------
  ud.updateUserDefinedTrajectory(time);
  cim.r_user = ud.getUserDefinedPosition();
  cim.r_dot_user = ud.getUserDefinedVelocity();
  cim.r_ddot_user = ud.getUserDefinedAcceleration();  
  cim.psi_user = ud.getUserDefinedYaw();
  cim.psi_user_unwrapped = unwrapPsiSimple(cim.psi_user, this->psiState);
  cim.psi_dot_user = ud.getUserDefinedYawRate();

  // 3. Capture the time before the execution of the controller ------------------
  cim.alg_start_time = std::chrono::high_resolution_clock::now();

  // 4. Assign the values from the integrator ------------------------------------
  assign_from_rk4();

}

// Function to assign elements from the rk4 integrator
void mrac_geometric::assign_from_rk4()
{
  int index = 0;

  //------------ assign after integration  ------------//
  assignElementsToMembers(csm.state_mu_x_filter, y, index);
  assignElementsToMembers(csm.state_mu_y_filter, y, index);
  assignElementsToMembers(csm.state_mu_z_filter, y, index);
  assignElementsToMembers(csm.state_omega_x_d_filter, y, index);
  assignElementsToMembers(csm.state_omega_y_d_filter, y, index);
  assignElementsToMembers(csm.state_omega_z_d_filter, y, index);

  assignElementsToMembers(csm.e_tran_pos_I, y, index);
  assignElementsToMembers(csm.e_tran_pos_ref_I, y, index);
  assignElementsToMembers(csm.x_tran_ref, y, index);
  assignElementsToMembers(csm.K_hat_x_tran, y, index);
  assignElementsToMembers(csm.K_hat_r_tran, y, index);
  assignElementsToMembers(csm.Theta_hat_tran, y, index);

  assignElementsToMembers(csm.e_omega_ref_I, y, index);
  assignElementsToMembers(csm.omega_ref, y, index);
  assignElementsToMembers(csm.K_hat_x_rot, y, index);
  assignElementsToMembers(csm.K_hat_r_rot, y, index);
  assignElementsToMembers(csm.Theta_hat_rot, y, index);
}


// Model function for integration
void mrac_geometric::model([[maybe_unused]] const rk4_array<double, NSI> &y, rk4_array<double, NSI> &dy, [[maybe_unused]] const double t)
{
  int index = 0;

  //------------ Fill up the dy for integration  ------------//
  assignElementsToDxdt(cim.internal_state_mu_x_filter, dy, index);
  assignElementsToDxdt(cim.internal_state_mu_y_filter, dy, index);
  assignElementsToDxdt(cim.internal_state_mu_z_filter, dy, index);
  assignElementsToDxdt(cim.internal_state_omega_x_d_filter, dy, index);
  assignElementsToDxdt(cim.internal_state_omega_y_d_filter, dy, index);
  assignElementsToDxdt(cim.internal_state_omega_z_d_filter, dy, index);

  assignElementsToDxdt(cim.e_tran_pos, dy, index);
  assignElementsToDxdt(cim.e_tran_pos_ref, dy, index);
  assignElementsToDxdt(cim.x_tran_ref_dot, dy, index);
  assignElementsToDxdt(cim.K_hat_x_tran_dot, dy, index);
  assignElementsToDxdt(cim.K_hat_r_tran_dot, dy, index);
  assignElementsToDxdt(cim.Theta_hat_tran_dot, dy, index);

  assignElementsToDxdt(cim.e_omega_ref, dy, index);
  assignElementsToDxdt(cim.omega_ref_dot, dy, index);
  assignElementsToDxdt(cim.K_hat_x_rot_dot, dy, index);
  assignElementsToDxdt(cim.K_hat_r_rot_dot, dy, index);
  assignElementsToDxdt(cim.Theta_hat_rot_dot, dy, index);

}

// Function to compute the outerloop control in I
void mrac_geometric::compute_translational_control_in_I()
{
  
  // Compute the error in the states
  cim.e_tran_pos << cim.x_tran_pos - csm.x_tran_ref.head<3>();
  cim.e_tran_vel << cim.x_tran_vel - csm.x_tran_ref.tail<3>();
  cim.e_tran << cim.e_tran_pos,
                cim.e_tran_vel;

  // Compute the translational position error between the reference model and the user defined trajectory
  cim.e_tran_pos_ref << csm.x_tran_ref.head<3>() - cim.r_user;

  // Compute the refernce command input [reference model - user_defined_trajectory]
  cim.r_cmd_tran << MASS * ( - cip.Ki_refmod_tran * csm.e_tran_pos_ref_I      // Integral term
                              + cip.Kp_refmod_tran * cim.r_user                // Proportional term
                              + cip.Kd_refmod_tran * cim.r_dot_user            // Derivative term
                              + cim.r_ddot_user );                             // Feedforward term

  // Reference model
  cim.x_tran_ref_dot << cip.A_ref_tran * csm.x_tran_ref
                      + cip.B_ref_tran * cim.r_cmd_tran;

  // Compute the baseline control input
  cim.mu_tran_baseline << MASS*( - cip.Kp_tran * cim.e_tran_pos               // Proportional term
                                  - cip.Kd_tran * cim.e_tran_vel               // Derivative term
                                  - cip.Ki_tran * csm.e_tran_pos_I             // Integral term
                                  + cim.x_tran_ref_dot.tail<3>() )             // Feedforward term
                                  - MASS * G * e3_basis;                       // Weight Dynamic inversion term

  // Temp regressor
  cim.outer_loop_regressor.setZero();
                                  
  // Compute the augmented regressor vector
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

  // Adaptive control law
  cim.mu_tran_adaptive << csm.K_hat_x_tran.transpose() * cim.x_tran
                        + csm.K_hat_r_tran.transpose() * cim.r_cmd_tran;
                        - csm.Theta_hat_tran.transpose() * cim.augmented_outer_loop_regressor;

  // Compute with the dynamic inversion without aerodynamics
  cim.mu_tran_I << cim.mu_tran_baseline + cim.mu_tran_adaptive;

}

// Function to compute the outerloop translational control rate in I using the differentiator
void mrac_geometric::compute_translational_control_rate()
{

  // Compute the internal state for rate of change of mu
  cim.internal_state_mu_x_filter << cip.A_filter_mu * csm.state_mu_x_filter
                                  + cip.B_filter_mu * cim.mu_tran_I(0);

  cim.internal_state_mu_y_filter << cip.A_filter_mu * csm.state_mu_y_filter
                                  + cip.B_filter_mu * cim.mu_tran_I(1);

  cim.internal_state_mu_z_filter << cip.A_filter_mu * csm.state_mu_z_filter
                                  + cip.B_filter_mu * cim.mu_tran_I(2);

  // Compute the rate of change of the translational virtual control input
  cim.mu_tran_I_dot(0) = cip.C_filter_mu * csm.state_mu_x_filter;
  cim.mu_tran_I_dot(1) = cip.C_filter_mu * csm.state_mu_y_filter;
  cim.mu_tran_I_dot(2) = cip.C_filter_mu * csm.state_mu_z_filter;

}

// Compute the orientation commands and the desired total thrust
void mrac_geometric::compute_u1_R_d()
{

  // Compute the desired total thrust
  cim.u(0) = cim.mu_tran_I.norm();

  // Compute the desired body frame z axis
  cim.b3d = -cim.mu_tran_I / cim.u(0);

  // Compute the desired "heading" vector 
  cim.c1 << std::cos(cim.psi_user),
            std::sin(cim.psi_user),
              0.0;

  // Construct an orthonormal triad
  cim.n_hat = cim.b3d.cross(cim.c1);
  cim.sigma = cim.n_hat.norm();
  cim.b2d =  cim.n_hat / cim.sigma;

  cim.b1d = cim.b2d.cross(cim.b3d);

  // Construct the desired rotation matrix
  cim.R_d.col(0) = cim.b1d;
  cim.R_d.col(1) = cim.b2d;
  cim.R_d.col(2) = cim.b3d;

  // Compute R_d_dot
  Matrix3d I = Matrix3d::Identity();

  // compute b3d_dot
  cim.b3d_dot = -1.0 * (1/cim.mu_tran_I.norm()) * (I - cim.b3d * cim.b3d.transpose()) * cim.mu_tran_I_dot;

  // compute b2d_dot
  cim.c1_dot << -1.0 * cim.psi_dot_user * std::sin(cim.psi_user),
                  cim.psi_dot_user * std::cos(cim.psi_user),
                  0.0;

  cim.n_hat_dot = cim.b3d_dot.cross(cim.c1) + cim.b3d.cross(cim.c1_dot);

  cim.b2d_dot = (1/cim.sigma) * (I - cim.b2d * cim.b2d.transpose()) * cim.n_hat_dot;

  // compute b1d_dot
  cim.b1d_dot = cim.b2d_dot.cross(cim.b3d) + cim.b2d.cross(cim.b3d_dot);

  // assign to R_d_dot
  cim.R_d_dot.col(0) = cim.b1d_dot;
  cim.R_d_dot.col(1) = cim.b2d_dot;
  cim.R_d_dot.col(2) = cim.b3d_dot;

  Matrix3d omega_d_K_skew;
  omega_d_K_skew = cim.R_d.transpose() * cim.R_d_dot;

  // build the desired angular velocity in the desired frame R_d from the skew symmetric matric omega_d_K_skew
  // omega_d_K_skew = [   0 -wd3  wd2]
  //                  [ wd3    0 -wd1]
  //                  [-wd2  wd1    0]
  cim.omega_d_in_K(0) = omega_d_K_skew(2, 1);
  cim.omega_d_in_K(1) = omega_d_K_skew(0, 2);
  cim.omega_d_in_K(2) = omega_d_K_skew(1, 0);

  // transform the desired angular velocity in the desired frame R_d to the current body frame J
  cim.omega_d = cim.Rji.transpose() * cim.R_d * cim.omega_d_in_K;

  // Compute the internal state for angular acceleration
  cim.internal_state_omega_x_d_filter << cip.A_filter_omega_d * csm.state_omega_x_d_filter
                                        + cip.B_filter_omega_d * cim.omega_d(0);

  cim.internal_state_omega_y_d_filter << cip.A_filter_omega_d * csm.state_omega_y_d_filter
                                        + cip.B_filter_omega_d * cim.omega_d(1);

  cim.internal_state_omega_z_d_filter << cip.A_filter_omega_d * csm.state_omega_z_d_filter
                                        + cip.B_filter_omega_d * cim.omega_d(2);

  // Compute the desired angular acceleration
  Eigen::Matrix<double, 3, 1> omega_dot_d_J;
  omega_dot_d_J(0) = cip.C_filter_omega_d * csm.state_omega_x_d_filter;
  omega_dot_d_J(1) = cip.C_filter_omega_d * csm.state_omega_y_d_filter;
  omega_dot_d_J(2) = cip.C_filter_omega_d * csm.state_omega_z_d_filter;

  cim.alpha_d = omega_dot_d_J + cim.omega.cross(cim.omega_d);
}

// Compute the rotational control
void mrac_geometric::compute_rotational_control()
{

  // Compute the error in the attitude
  Matrix3d I = Matrix3d::Identity();
  Matrix<double, 3, 1> e1 = I.col(0);
  Matrix<double, 3, 1> e2 = I.col(1);
  Matrix<double, 3, 1> e3 = I.col(2);
  Vector3d a1e1 = cip.Ka_att(0) * e1;
  Vector3d a2e2 = cip.Ka_att(1) * e2;
  Vector3d a3e3 = cip.Ka_att(2) * e3;
  Vector3d r1 = cim.R_d.transpose() * cim.Rji * e1;
  Vector3d r2 = cim.R_d.transpose() * cim.Rji * e2;
  Vector3d r3 = cim.R_d.transpose() * cim.Rji * e3;

  cim.Xi_e = a1e1.cross(r1) + a2e2.cross(r2) + a3e3.cross(r3);

  // Compute the error in the angular velocities
  cim.omega_e << cim.omega - csm.omega_ref;

  // Compute the error between the angular velocity reference model and the desired angular velocity
  cim.e_omega_ref << csm.omega_ref - cim.omega_d;

  // Compute the command angular velocity
  cim.omega_cmd << cip.Kp_omega_ref * cim.omega_d - cip.Ki_omega_ref * csm.e_omega_ref_I + cim.alpha_d;

  // Compute the derivative of the refernce angular velocity
  cim.omega_ref_dot << cip.A_ref_rot * csm.omega_ref + cip.B_ref_rot * cim.omega_cmd;

  // Cache the feedforward term
  Vector3d fft;
  fft = inertia_matrix_q * ( cim.omega.cross(inertia_matrix_q * cim.omega) - cim.alpha_d );

  // Compute the baseline control input
  cim.tau_rot_baseline << inertia_matrix_q * ( - cip.Kp_att * cim.Xi_e        // Proportional term
                                                - cip.Kd_att * cim.omega_e)    // Derivative term
                                                + fft;                         // Feedforward term

  // Temp regressor
  cim.inner_loop_regressor.setZero();

  // Compute the augmented regressor vector
  cim.augmented_inner_loop_regressor << cim.tau_rot_baseline, cim.inner_loop_regressor;

  // Cache the transpose of the tracking error * P * B
  Matrix<double, 1, 3> e_transpose_p_b = cim.omega_e.transpose() * cip.P_rot * cip.B_rot;

  // Computing the scalar value output from the dead-zone modification modulation function
  cim.dead_zone_value_rotational = deadZoneModulationFunction(cim.omega_e.transpose(),
                                                              cip.dead_zone_delta_rotational,
                                                              cip.dead_zone_e0_rotational);

  // Adaptive laws
  cim.K_hat_x_rot_dot = AdaptiveLawDeadZoneEMod(-cip.Gamma_x_rot,
                                                cim.dead_zone_value_rotational,
                                                cim.omega,
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

  // Adaptive control law
  cim.tau_rot_adaptive << csm.K_hat_x_rot.transpose() * cim.omega
            + csm.K_hat_r_rot.transpose() * cim.omega_cmd
            - csm.Theta_hat_rot.transpose() * cim.augmented_inner_loop_regressor;

  // Total rotational control input
  cim.tau_rot << cim.tau_rot_baseline + cim.tau_rot_adaptive;	

  // Assing the control inputs
  cim.u(1) = cim.tau_rot(0);
  cim.u(2) = cim.tau_rot(1);
  cim.u(3) = cim.tau_rot(2);

}

// Function to compute the normalized thrusts
void mrac_geometric::compute_normalized_thrusts()
{
	// Compute the individual thrusts in Newtons
  cim.Thrust << mixer_matrix_quadm * cim.u;

	// Saturate each element of the Thrust vector between MIN_THRUST and MAX_THRUST
	cim.Sat_Thrust = (cim.Thrust.cwiseMin(MAX_THRUST).cwiseMax(MIN_THRUST));

	// Compute the final control inputs
	control_input(0) = evaluatePolynomial(thrust_polynomial_coeff_quadm, cim.Sat_Thrust(0));
	control_input(1) = evaluatePolynomial(thrust_polynomial_coeff_quadm, cim.Sat_Thrust(1));
	control_input(2) = evaluatePolynomial(thrust_polynomial_coeff_quadm, cim.Sat_Thrust(2));
	control_input(3) = evaluatePolynomial(thrust_polynomial_coeff_quadm, cim.Sat_Thrust(3));
}

// Function that is called in control.cpp
void mrac_geometric::run(const double time_step_rk4_) {

	// Process the dynamics --------------------------------------------------------
	// 1. Compute the aerodynamics 
	
  // 2. Compute the translational control input
  compute_translational_control_in_I();

  // 3. Compute the rate of change of the translational control input
  compute_translational_control_rate();

  // 4. Compute the thrust needed and the desired rotaion matrix
  compute_u1_R_d();

  // 5. Compute the rotational control input
  compute_rotational_control();

  // 6. Compute the normalized thrust - Final Step
  compute_normalized_thrusts();

  // 7. Do the integration
  rk4.do_step(boost::bind(&mrac_geometric::model, this, bph::_1, bph::_2, bph::_3),
              y, cim.t, time_step_rk4_);

  // Capture the time after the execution of the controller
  cim.alg_end_time = std::chrono::high_resolution_clock::now();

  // Calculate the duration of the execution 
  cim.alg_duration = std::chrono::duration_cast<std::chrono::microseconds>(
                          cim.alg_end_time - cim.alg_start_time).count();

	// 9. Log the Data after all the calculataions
	logger.logLogData();
}

// Implementing getter functions from controller_base
float mrac_geometric::get_t1() const {
	return control_input(0);
}

float mrac_geometric::get_t2() const {
	return control_input(1);
}

float mrac_geometric::get_t3() const {
	return control_input(2);
}

float mrac_geometric::get_t4() const {
	return control_input(3);
}

float mrac_geometric::get_t5() const {
	return control_input(4);
}

float mrac_geometric::get_t6() const {
	return control_input(5);
}

float mrac_geometric::get_t7() const {
	return control_input(6);
}

float mrac_geometric::get_t8() const {
	return control_input(7);
}

} // namesapce _mrac_geometric_
} // namespace _qrbp_